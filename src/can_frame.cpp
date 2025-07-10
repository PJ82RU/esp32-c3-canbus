#include "canbus/can_frame.h"

#include <cstring>
#include <esp_log.h>

namespace canbus
{
    CanFrame::CanFrame() noexcept
    {
        clear();
        ESP_LOGD(TAG, "CAN frame created");
    }

    void CanFrame::clear() noexcept
    {
        id = 0;
        length = 0;
        extended = 0;
        rtr = 0;
        filterIndex = -1;
        memset(&data, 0, sizeof(data));
        ESP_LOGD(TAG, "CAN frame cleared");
    }

    bool CanFrame::hasData() const noexcept
    {
        return id > 0 && length > 0 && rtr == 0;
    }

    uint16_t CanFrame::getWord(const int index) const noexcept
    {
        if (index >= 0 && index + 1 < length)
        {
            const uint16_t result = (data.bytes[index] << 8) | data.bytes[index + 1];
            ESP_LOGD(TAG, "Get word: %u", result);
            return result;
        }
        ESP_LOGW(TAG, "Get word: index out of range");
        return 0;
    }

    bool CanFrame::compare(const CanFrame& frame) const noexcept
    {
        // Быстрая проверка наиболее вероятных различий
        if (frame.id != id || frame.length != length ||
            frame.extended != extended || frame.rtr != rtr)
        {
            ESP_LOGD(TAG, "Compare: header mismatch");
            return false;
        }

        // Оптимизированное сравнение данных через memcmp
        if (length > 0 && memcmp(frame.data.bytes, data.bytes, length) != 0)
        {
            ESP_LOGD(TAG, "Compare: data mismatch");
            return false;
        }

        return true;
    }

    Bytes CanFrame::getBytes(const int indexes[], const size_t size) noexcept
    {
        Bytes result = {};
        for (size_t i = 0; i < size; i++)
        {
            if (const auto idx = indexes[i]; idx >= 0 && idx < 64)
            {
                result.bit[static_cast<uint8_t>(i)] = data.bit[idx];
            }
        }

        ESP_LOGD(TAG, "Get bytes: %zu bits processed", size);
        return result;
    }
} // namespace canbus
