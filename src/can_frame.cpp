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

    CanFrame::CanFrame(const twai_message_t& msg)
    {
        id = msg.identifier;
        extended = msg.extd;
        rtr = msg.rtr;
        length = msg.data_length_code;

        // Копируем данные (если это не RTR-фрейм)
        if (!msg.rtr && msg.data_length_code > 0)
        {
            memcpy(data.bytes, msg.data, msg.data_length_code);
        }
        ESP_LOGD(TAG, "Creating from TWAI message, id: 0x%X, length: %d",
                 static_cast<unsigned int>(msg.identifier),
                 msg.data_length_code);
    }

    void CanFrame::clear() noexcept
    {
        id = 0;
        length = 0;
        extended = false;
        rtr = false;
        memset(&data, 0, sizeof(data));

        ESP_LOGD(TAG, "CAN frame cleared");
    }

    bool CanFrame::hasData() const noexcept
    {
        return id > 0 && length > 0 && length <= TWAI_FRAME_MAX_DLC && !rtr;
    }

    twai_message_t CanFrame::getTwaiMessage() const
    {
        // Полная инициализация структуры сообщения
        twai_message_t message = {};
        message.identifier = id;
        message.data_length_code = length;

        // Установка битовых флагов через анонимную структуру
        message.extd = extended ? 1 : 0;
        message.rtr = rtr ? 1 : 0;
        message.ss = 0;           // Не используется для передачи
        message.self = 0;         // Не используется для передачи
        message.dlc_non_comp = 0; // Стандартный DLC
        message.reserved = 0;     // Резервные биты

        // Копируем данные, если это не RTR-фрейм
        if (!rtr && length > 0)
        {
            memcpy(message.data, data.bytes, (length <= TWAI_FRAME_MAX_DLC) ? length : TWAI_FRAME_MAX_DLC);
        }
        return message;
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
