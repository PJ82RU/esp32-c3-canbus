#include "canbus/can.h"

#include <algorithm>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

namespace canbus
{
    Can::Can(const gpio_num_t txPin, const gpio_num_t rxPin) noexcept
        : mSendThread("CAN_SEND", STACK_DEPTH_SEND, 19),
          mReceiveThread("CAN_RECEIVE", STACK_DEPTH_RECEIVE, 19),
          mWatchdogThread("CAN_WATCHDOG", STACK_DEPTH_WATCHDOG, 10),
          mSendQueue(MAX_QUEUE_SIZE),
          mFilters(),
          mFrameEntries(),
          mDriverReady(false)
    {
        mDriverConfig = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
        mDriverConfig.rx_queue_len = RX_BUFFER_SIZE;
        mTimingConfig = TWAI_TIMING_CONFIG_125KBITS();
        mFilterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    }

    Can::~Can() noexcept
    {
        stop();
    }

    void Can::bind(std::unique_ptr<FrameCallback> callback, ErrorFunction error)
    {
        std::lock_guard lock(mMutex);
        mDataCallback = std::move(callback);
        mErrorCallback = std::move(error);
    }

    int16_t Can::bindFrameEntry(const FrameFunction& provider,
                                const uint16_t intervalMs,
                                const uint16_t sendCount,
                                const bool active,
                                const ErrorFunction& error)
    {
        std::lock_guard lock(mMutex);
        if (mActiveFrameEntriesCount >= MAX_FRAMES)
        {
            ESP_LOGE(TAG, "Failed to add frame entry: buffer full (max %d entries)", MAX_FRAMES);
            return -1;
        }

        mFrameEntries[mActiveFrameEntriesCount] = {
            provider,
            error,
            intervalMs,
            active ? (esp_timer_get_time() / 1000 + intervalMs) : static_cast<uint64_t>(0),
            sendCount,
            active
        };

        if (active)
        {
            ESP_LOGI(TAG, "Added periodic frame #%d: interval %ums, first send in %ums",
                     mActiveFrameEntriesCount, intervalMs, intervalMs);
        }
        else
        {
            ESP_LOGI(TAG, "Added inactive frame entry #%d", mActiveFrameEntriesCount);
        }

        return mActiveFrameEntriesCount++;
    }

    bool Can::setFrameEntryActive(const int16_t index, const bool active) noexcept
    {
        std::lock_guard lock(mMutex);

        // Проверка валидности индекса
        if (index < 0 || index >= MAX_FRAMES || index >= mActiveFrameEntriesCount)
        {
            ESP_LOGE(TAG, "Invalid frame entry index: %d (max: %d)",
                     index, mActiveFrameEntriesCount - 1);
            return false;
        }

        // Проверка изменения состояния
        if (mFrameEntries[index].isActive == active)
        {
            ESP_LOGW(TAG, "Frame entry %d already %s",
                     index, active ? "active" : "inactive");
            return true;
        }

        // Установка нового состояния
        mFrameEntries[index].isActive = active;

        // Сброс таймера при активации
        if (active)
        {
            mFrameEntries[index].nextSendTime = esp_timer_get_time() / 1000 +
                mFrameEntries[index].intervalMs;
        }

        ESP_LOGI(TAG, "Frame entry %d %s %s", index,
                 (active ? "activated" : "deactivated"),
                 (active ? ("next send in " + std::to_string(mFrameEntries[index].intervalMs) + "ms").c_str() : ""));

        return true;
    }

    bool Can::installAndStartDriver() const noexcept
    {
        if (mDriverReady.load(std::memory_order_acquire))
        {
            ESP_LOGW(TAG, "Driver already installed");
            return true;
        }

        if (twai_driver_install(&mDriverConfig, &mTimingConfig, &mFilterConfig) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to install TWAI driver");
            return false;
        }

        if (twai_start() != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start TWAI driver");
            twai_driver_uninstall();
            return false;
        }

        mDriverReady.store(true, std::memory_order_release);
        ESP_LOGI(TAG, "TWAI driver started successfully");
        return true;
    }

    void Can::stopAndUninstallDriver() const noexcept
    {
        if (!mDriverReady.exchange(false)) return;

        twai_stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        twai_driver_uninstall();

        ESP_LOGI(TAG, "TWAI driver stopped and uninstalled");
    }

    bool Can::start(const CanSpeed speed)
    {
        std::lock_guard lock(mMutex);
        if (!isInitialized() && mDriverConfig.tx_io != GPIO_NUM_NC && mDriverConfig.rx_io != GPIO_NUM_NC)
        {
            setSpeed(speed);

            // Обработчик отправки сообщений
            auto loopSend = [&]()
            {
                if (!mDriverReady.load(std::memory_order_acquire)) return esp32_c3::objects::Thread::LoopAction::STOP;
                processSendQueue();
                return esp32_c3::objects::Thread::LoopAction::CONTINUE;
            };

            // Обработчик приема сообщений
            auto loopReceive = [&]()
            {
                if (!mDriverReady.load(std::memory_order_acquire)) return esp32_c3::objects::Thread::LoopAction::STOP;
                processReceivedData();
                return esp32_c3::objects::Thread::LoopAction::CONTINUE;
            };

            // Обработчик watchdog-таймера
            auto loopWatchdog = [&]()
            {
                processWatchdog();
                return esp32_c3::objects::Thread::LoopAction::CONTINUE;
            };

            return installAndStartDriver() &&
                mSendThread.start(loopSend, SEND_INTERVAL_MS) &&
                mReceiveThread.start(loopReceive, RECEIVE_INTERVAL_MS) &&
                mWatchdogThread.start(loopWatchdog, WATCHDOG_INTERVAL_MS);
        }

        ESP_LOGW(TAG, "Invalid GPIO configuration");
        return false;
    }

    void Can::stop()
    {
        std::lock_guard lock(mMutex);
        if (isInitialized())
        {
            mWatchdogThread.stop();
            mReceiveThread.stop();
            mSendThread.stop();
            if (!mSendQueue.reset())
            {
                ESP_LOGE(TAG, "Failed to reset send queue");
            }
            stopAndUninstallDriver();
        }
    }

    bool Can::isInitialized() const noexcept
    {
        std::lock_guard lock(mMutex);
        return mDriverReady.load(std::memory_order_acquire) &&
            mSendThread.state() != esp32_c3::objects::Thread::State::NOT_RUNNING &&
            mReceiveThread.state() != esp32_c3::objects::Thread::State::NOT_RUNNING &&
            mWatchdogThread.state() != esp32_c3::objects::Thread::State::NOT_RUNNING;
    }

    twai_state_t Can::state() const noexcept
    {
        std::lock_guard lock(mMutex);
        return mStatusInfo.state;
    }

    bool Can::waitRunning(const uint32_t timeout) const noexcept
    {
        std::lock_guard lock(mMutex);
        // Быстрая проверка без ожидания
        if (mStatusInfo.state == TWAI_STATE_RUNNING) return true;

        if (timeout == 0) return false;

        const TickType_t endTicks = xTaskGetTickCount() + pdMS_TO_TICKS(timeout);
        constexpr TickType_t checkInterval = pdMS_TO_TICKS(10);

        do
        {
            vTaskDelay(checkInterval);
            // ReSharper disable once CppDFAConstantConditions
            // ReSharper disable once CppDFAUnreachableCode
            if (mStatusInfo.state == TWAI_STATE_RUNNING) return true;
        }
        while (xTaskGetTickCount() < endTicks);

        return false;
    }

    void Can::setSpeed(CanSpeed speed) noexcept
    {
        std::lock_guard lock(mMutex);
        switch (speed)
        {
        case CanSpeed::SPEED_25KBIT: mTimingConfig = TWAI_TIMING_CONFIG_25KBITS();
            break;
        case CanSpeed::SPEED_50KBIT: mTimingConfig = TWAI_TIMING_CONFIG_50KBITS();
            break;
        case CanSpeed::SPEED_100KBIT: mTimingConfig = TWAI_TIMING_CONFIG_100KBITS();
            break;
        case CanSpeed::SPEED_125KBIT: mTimingConfig = TWAI_TIMING_CONFIG_125KBITS();
            break;
        case CanSpeed::SPEED_250KBIT: mTimingConfig = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case CanSpeed::SPEED_500KBIT: mTimingConfig = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case CanSpeed::SPEED_800KBIT: mTimingConfig = TWAI_TIMING_CONFIG_800KBITS();
            break;
        case CanSpeed::SPEED_1MBIT: mTimingConfig = TWAI_TIMING_CONFIG_1MBITS();
            break;
        }

        ESP_LOGI(TAG, "CAN speed set to %d kbit", static_cast<int>(speed));
    }

    int16_t Can::setFilter(const uint32_t id,
                           const uint32_t mask,
                           const bool extended,
                           const int16_t callbackIndex) noexcept
    {
        std::lock_guard lock(mMutex);
        if (mActiveFiltersCount >= MAX_FILTER_SIZE)
        {
            ESP_LOGW(TAG, "No free filters available");
            return -1;
        }

        mFilters[mActiveFiltersCount] = {
            id & mask,
            mask,
            callbackIndex,
            true,
            extended
        };

        return mActiveFiltersCount++;
    }

    CanFilter Can::getFilter(const int16_t index) const noexcept
    {
        std::lock_guard lock(mMutex);
        return index >= 0 && index < MAX_FILTER_SIZE ? mFilters[index] : CanFilter();
    }

    void Can::clearFilters() noexcept
    {
        std::lock_guard lock(mMutex);
        mFilters.fill(CanFilter{});
        mActiveFiltersCount = 0;

        ESP_LOGI(TAG, "Cleared %u filters", MAX_FILTER_SIZE);
    }

    esp_err_t Can::send(const CanFrame& frame)
    {
        std::lock_guard lock(mMutex);

        // Валидация параметров
        if (!isInitialized() || !frame.hasData())
        {
            ESP_LOGE(TAG, "Invalid send params: init=%d, len=%zu",
                     mDriverReady.load(std::memory_order_acquire), frame.length);
            return ESP_ERR_INVALID_ARG;
        }

        return mSendQueue.send(frame, 0) ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    esp_err_t Can::sendImpl(const CanFrame& frame) const
    {
        if (!mDriverReady.load(std::memory_order_acquire) || mStatusInfo.state != TWAI_STATE_RUNNING)
        {
            ESP_LOGW(TAG, "CAN interface not ready");
            return ESP_ERR_INVALID_STATE;
        }

        const twai_message_t message = frame.getTwaiMessage();
        constexpr TickType_t sendTimeout = pdMS_TO_TICKS(SEND_MS_TO_TICKS);
        const esp_err_t err = twai_transmit(&message, sendTimeout);
        if (err == ESP_OK)
        {
            mStats.txFrames.fetch_add(1, std::memory_order_relaxed);
            ESP_LOGD(TAG, "Frame 0x%08" PRIX32 " sent successfully", frame.id);
            return ESP_OK;
        }

        mStats.errors.fetch_add(1, std::memory_order_relaxed);
        ESP_LOGW(TAG, "Failed to send frame 0x%08" PRIX32 ": %s",
                 frame.id, esp_err_to_name(err));
        return err;
    }

    void Can::processSendQueue()
    {
        // 1. Обработка очереди (срочные кадры)
        if (!mSendQueue.empty())
        {
            if (CanFrame frame; mSendQueue.receive(frame, 0))
            {
                if (const esp_err_t ret = sendImpl(frame); ret == ESP_OK)
                {
                    ESP_LOGV(TAG, "Frame 0x%03X sent successfully",
                             static_cast<unsigned int>(frame.id));
                }
                else
                {
                    ESP_LOGV(TAG, "Failed to send frame 0x%03X, error: 0x%X (%s)",
                             static_cast<unsigned int>(frame.id), ret, esp_err_to_name(ret));

                    if (mErrorCallback) mErrorCallback(frame, ret);
                }
            }
            return;
        }

        // 2. Периодические кадры (если очередь пуста)
        if (mActiveFrameEntriesCount == 0) return;

        const uint64_t now = esp_timer_get_time() / 1000;
        for (size_t i = 0; i < mActiveFrameEntriesCount; ++i)
        {
            const size_t index = (mLastProcessedEntryIndex + i) % mActiveFrameEntriesCount;
            if (auto& entry = mFrameEntries[index]; entry.isActive && now >= entry.nextSendTime)
            {
                const CanFrame frame = entry.frameProvider();
                if (const esp_err_t ret = sendImpl(frame); ret == ESP_OK)
                {
                    ESP_LOGV(TAG, "Periodic frame 0x%03X sent (next in %ums)",
                             static_cast<unsigned int>(frame.id), entry.intervalMs);
                }
                else
                {
                    ESP_LOGV(TAG, "Periodic frame 0x%03X send failed: 0x%X (%s)",
                             static_cast<unsigned int>(frame.id),
                             ret,
                             esp_err_to_name(ret));

                    if (entry.errorProvider) entry.errorProvider(frame, ret);
                }

                entry.nextSendTime = now + entry.intervalMs;
                mLastProcessedEntryIndex = static_cast<int16_t>((index + 1) % mActiveFrameEntriesCount);
                break;
            }
        }
    }

    void Can::processReceivedData() noexcept
    {
        twai_message_t message;
        if (!mDataCallback)
        {
            // Сбрасываем очередь при отсутствии обработчика
            while (twai_receive(&message, 0) == ESP_OK)
            {
            }
            return;
        }

        auto response = [&](const CanFrame& result)
        {
            send(result);
        };

        while (twai_receive(&message, 0) == ESP_OK)
        {
            CanFrame frame(message);
            bool processed = false;

            for (size_t i = 0; i < mActiveFiltersCount; ++i)
            {
                if (const auto& filter = mFilters[i]; filter.configured &&
                    (frame.id & filter.mask) == filter.id &&
                    frame.extended == filter.extended)
                {
                    mDataCallback->invoke(frame, response, static_cast<int16_t>(i));
                    ESP_LOGD(TAG, "Frame 0x%08" PRIX32 " processed by filter %zu",
                             message.identifier, i);
                    processed = true;
                    break;
                }
            }

            // Обработка кадров без фильтра
            if (!processed)
            {
                mDataCallback->invoke(frame, response);
                ESP_LOGD(TAG, "Frame 0x%08" PRIX32 " received (no filter)", message.identifier);
            }

            mStats.rxFrames.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void Can::processWatchdog()
    {
        if (twai_get_status_info(&mStatusInfo) == ESP_OK &&
            mStatusInfo.state == TWAI_STATE_BUS_OFF)
        {
            if (twai_initiate_recovery() != ESP_OK)
            {
                ESP_LOGW(TAG, "Bus recovery failed");
            }
        }
    }

    size_t Can::getQueueSize() const
    {
        return mSendQueue.waiting();
    }

    size_t Can::clearQueue()
    {
        size_t count = 0;
        CanFrame frame;
        while (mSendQueue.receive(frame, 0))
        {
            count++;
        }
        return count;
    }
} // namespace canbus
