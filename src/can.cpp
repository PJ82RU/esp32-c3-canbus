#include "canbus/can.h"

#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

namespace canbus
{
    Can::Can(gpio_num_t txPin, gpio_num_t rxPin) noexcept
        : mWatchdogThread("CAN_WATCHDOG", 2048, 10),
          mReceiveThread("CAN_RECEIVE", 4096, 19)
    {
        mDriverConfig = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
        mTimingConfig = TWAI_TIMING_CONFIG_125KBITS();
        mFilterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        clearFilters();
    }

    Can::~Can() noexcept
    {
        end();
    }

    bool Can::installAndStartDriver() noexcept
    {
        if (mDriverReady)
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

        mDriverReady = true;
        ESP_LOGI(TAG, "TWAI driver started successfully");
        return true;
    }

    void Can::stopAndUninstallDriver() noexcept
    {
        if (!mDriverReady) return;

        mDriverReady = false;
        twai_stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        twai_driver_uninstall();
        ESP_LOGI(TAG, "TWAI driver stopped and uninstalled");
    }

    bool Can::begin(esp32_c3::objects::Callback* callback) noexcept
    {
        if (callback == nullptr) return false;

        std::lock_guard lock(mMutex);
        bool result = false;

        if (mDriverConfig.tx_io != GPIO_NUM_NC && mDriverConfig.rx_io != GPIO_NUM_NC)
        {
            if (mDriverReady) stopAndUninstallDriver();

            setSpeed(mSpeed);
            mCallback = callback;
            result = installAndStartDriver() &&
                mReceiveThread.start(&CanReceiveTask, this) &&
                mWatchdogThread.start(&CanWatchdogTask, this);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid GPIO configuration");
        }

        return result;
    }

    void Can::end() noexcept
    {
        std::lock_guard lock(mMutex);

        if (mDriverReady)
        {
            mWatchdogThread.stop();
            mReceiveThread.stop();
            stopAndUninstallDriver();
        }
    }

    twai_state_t Can::getState() const noexcept
    {
        return mStatusInfo.state;
    }

    bool Can::waitRunning(const uint32_t timeout) const noexcept
    {
        // Быстрая проверка без ожидания
        if (mStatusInfo.state == TWAI_STATE_RUNNING)
        {
            return true;
        }

        if (timeout == 0)
        {
            return false;
        }

        const TickType_t endTicks = xTaskGetTickCount() + pdMS_TO_TICKS(timeout);
        constexpr TickType_t smallDelay = pdMS_TO_TICKS(5);

        do
        {
            vTaskDelay(smallDelay);
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
        mSpeed = speed;

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

    int8_t Can::setFilter(const uint8_t index,
                          const uint32_t id,
                          const uint32_t mask,
                          const bool extended,
                          const int16_t callbackIndex) noexcept
    {
        if (index >= CAN_NUM_FILTER) return -1;

        std::lock_guard lock(mMutex);
        mFilters[index] = {id & mask, mask, callbackIndex, true, extended};

        ESP_LOGD(TAG, "Filter %u set: id=0x%08" PRIX32 ", mask=0x%08" PRIX32,
                 static_cast<unsigned int>(index),
                 id,
                 mask);
        return static_cast<int8_t>(index);
    }

    int8_t Can::setFilter(const uint32_t id,
                          const uint32_t mask,
                          const bool extended,
                          const int16_t callbackIndex) noexcept
    {
        if (const auto it = std::ranges::find_if(mFilters, [](const CanFilter& f) { return !f.configured; }); it !=
            mFilters.end())
        {
            return setFilter(std::distance(mFilters.begin(), it), id, mask, extended, callbackIndex);
        }

        ESP_LOGW(TAG, "No free filters available");
        return -1;
    }

    CanFilter Can::getFilter(const int16_t index) const noexcept
    {
        if (index >= 0 && index < static_cast<int16_t>(mFilters.size()))
        {
            return mFilters[index];
        }
        return CanFilter{};
    }

    void Can::clearFilters() noexcept
    {
        std::lock_guard lock(mMutex);
        mFilters.fill(CanFilter{});
        ESP_LOGI(TAG, "Cleared %u filters", CAN_NUM_FILTER);
    }

    void Can::processFrame(const twai_message_t& message) const noexcept
    {
        if (mCallback == nullptr) return;

        CanFrame frame;
        frame.id = message.identifier;
        frame.length = message.data_length_code;
        frame.rtr = message.rtr;
        frame.extended = message.extd;
        memcpy(frame.data.bytes, message.data, CAN_FRAME_DATA_SIZE);

        for (size_t i = 0; i < mFilters.size(); ++i)
        {
            if (const auto& filter = mFilters[i]; filter.configured &&
                (message.identifier & filter.mask) == filter.id &&
                message.extd == filter.extended)
            {
                frame.filterIndex = static_cast<int8_t>(i);
                mCallback->invoke(&frame, static_cast<int16_t>(i));
                ESP_LOGD(TAG, "Frame 0x%08" PRIX32 " processed by filter %zu",
                         message.identifier, i);
                return;
            }
        }

        // Обработка кадров без фильтра
        frame.filterIndex = -1;
        mCallback->invoke(&frame);
        ESP_LOGD(TAG, "Frame 0x%08" PRIX32 " received (no filter)", message.identifier);
    }

    bool Can::send(CanFrame& frame) const noexcept
    {
        if (!frame.hasData())
        {
            ESP_LOGW(TAG, "Invalid frame data");
            return false;
        }

        const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (frame.nextSendTime > currentTime)
        {
            ESP_LOGD(TAG, "Frame 0x%08" PRIX32 " send delayed", frame.id);
            return false;
        }

        std::lock_guard lock(mMutex);

        if (!mDriverReady || mStatusInfo.state != TWAI_STATE_RUNNING)
        {
            ESP_LOGW(TAG, "CAN interface not ready");
            return false;
        }

        if (frame.frequency != 0)
        {
            frame.nextSendTime = currentTime + frame.frequency;
        }

        // Полная инициализация структуры сообщения
        twai_message_t message = {};
        message.identifier = frame.id;
        message.data_length_code = frame.length;

        // Установка битовых флагов через анонимную структуру
        message.extd = frame.extended ? 1 : 0;
        message.rtr = frame.rtr ? 1 : 0;
        message.ss = 0;           // Не используется для передачи
        message.self = 0;         // Не используется для передачи
        message.dlc_non_comp = 0; // Стандартный DLC
        message.reserved = 0;     // Резервные биты

        // Копируем данные, если это не RTR-фрейм
        if (!frame.rtr && frame.length > 0)
        {
            memcpy(message.data, frame.data.bytes,
                   (frame.length <= TWAI_FRAME_MAX_DLC) ? frame.length : TWAI_FRAME_MAX_DLC);
        }

        constexpr TickType_t sendTimeout = pdMS_TO_TICKS(CAN_SEND_MS_TO_TICKS);
        const esp_err_t err = twai_transmit(&message, sendTimeout);
        if (err == ESP_OK)
        {
            ESP_LOGD(TAG, "Frame 0x%08" PRIX32 " sent successfully", frame.id);
            return true;
        }

        ESP_LOGW(TAG, "Failed to send frame 0x%08" PRIX32 ": %s",
                 frame.id, esp_err_to_name(err));
        return false;
    }

    bool Can::receive(CanFrame& frame) const noexcept
    {
        return mCallback != nullptr && mCallback->read(&frame);
    }

    void Can::CanWatchdogTask(void* params)
    {
        if (auto* can = static_cast<Can*>(params)) can->handleWatchdog();
        vTaskDelete(nullptr);
    }

    void Can::CanReceiveTask(void* params)
    {
        if (const auto* can = static_cast<Can*>(params)) can->handleReceive();
        vTaskDelete(nullptr);
    }

    [[noreturn]] void Can::handleWatchdog() noexcept
    {
        constexpr TickType_t delay = 200 / portTICK_PERIOD_MS;
        while (true)
        {
            if (twai_get_status_info(&mStatusInfo) == ESP_OK &&
                mStatusInfo.state == TWAI_STATE_BUS_OFF)
            {
                if (twai_initiate_recovery() != ESP_OK)
                {
                    ESP_LOGW(TAG, "Bus recovery failed");
                }
            }
            vTaskDelay(delay);
        }
    }

    [[noreturn]] void Can::handleReceive() const noexcept
    {
        while (true)
        {
            if (mDriverReady)
            {
                twai_message_t message;
                if (twai_receive(&message, pdMS_TO_TICKS(CAN_RECEIVE_MS_TO_TICKS)) == ESP_OK)
                {
                    processFrame(message);
                }
                vTaskDelay(pdMS_TO_TICKS(CAN_RECEIVE_MS_TO_TICKS));
            }
        }
    }
} // namespace canbus
