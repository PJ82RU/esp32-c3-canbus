#ifndef CANBUS_CAN_FRAME_H
#define CANBUS_CAN_FRAME_H

#include <cstddef>
#include <cstdint>
#include <hal/twai_types.h>

namespace canbus
{
    /**
     * @brief Константы CAN-фрейма
     */
    constexpr uint8_t CAN_FRAME_DATA_SIZE = 8; ///< Размер данных CAN-фрейма
    constexpr uint16_t CAN_FRAME_FREQ = 250;   ///< Частота отправки по умолчанию (мс)

    /**
     * @brief Класс для работы с отдельными битами
     */
    class BitRef
    {
    public:
        /**
         * @brief Конструктор
         * @param[in] ref Указатель на байт
         * @param[in] pos Позиция бита в байте (0-7)
         */
        BitRef(uint8_t* ref, const int pos) noexcept : mRef(ref), mPos(pos)
        {
        }

        /**
         * @brief Оператор присваивания значения биту
         * @param[in] x Значение для установки
         * @return Ссылка на текущий объект
         */
        BitRef& operator=(const bool x) noexcept
        {
            *mRef = (*mRef & ~(1 << mPos));
            if (x) *mRef = *mRef | (1 << mPos);
            return *this;
        }

        /**
         * @brief Оператор приведения к bool
         * @return Значение бита
         */
        explicit operator bool() const noexcept
        {
            return (*mRef & (1 << mPos)) != 0;
        }

    private:
        uint8_t* mRef; ///< Указатель на байт
        int mPos;      ///< Позиция бита
    };

    /**
     * @brief Объединение для работы с данными CAN-фрейма
     * @details Позволяет интерпретировать данные фрейма различными способами:
     * как целые числа разной разрядности, массивы чисел или отдельные биты
     */
    union Bytes
    {
        uint64_t uint64;                    ///< 64-битное беззнаковое целое
        uint32_t uint32[2];                 ///< Массив 32-битных беззнаковых
        uint16_t uint16[4];                 ///< Массив 16-битных беззнаковых
        uint8_t uint8[8];                   ///< Массив 8-битных беззнаковых
        int64_t int64;                      ///< 64-битное знаковое целое
        int32_t int32[2];                   ///< Массив 32-битных знаковых
        int16_t int16[4];                   ///< Массив 16-битных знаковых
        int8_t int8[8];                     ///< Массив 8-битных знаковых
        uint8_t bytes[CAN_FRAME_DATA_SIZE]; ///< Массив байт

        /**
         * @brief Структура для работы с битами
         */
        struct
        {
            uint8_t field[8]; ///< Поле для битовых операций

            /**
             * @brief Оператор чтения бита
             * @param[in] pos Позиция бита (0-63)
             * @return Значение бита
             * @note Для недопустимых позиций возвращает false
             */
            bool operator[](const int pos) const noexcept
            {
                return (pos >= 0 && pos <= 63) ? (field[pos / 8] >> (pos % 8)) & 1 : false;
            }

            /**
             * @brief Оператор записи бита
             * @param[in] pos Позиция бита (0-63)
             * @return Ссылка на бит
             * @note Для недопустимых позиций возвращает ссылку на первый бит
             */
            BitRef operator[](const int pos) noexcept
            {
                return (pos >= 0 && pos <= 63) ? BitRef(&field[pos / 8], pos % 8) : BitRef(&field[0], 0);
            }
        } bit;
    };

    /**
     * @brief Класс CAN-фрейма
     * @details Представляет CAN-сообщение с идентификатором, флагами и данными.
     * Поддерживает как стандартные (11-битные), так и расширенные (29-битные) идентификаторы.
     */
    class CanFrame
    {
    public:
        /// @brief Тег для логирования
        static constexpr auto TAG = "CANFrame";

        /**
         * @brief Конструктор по умолчанию
         * @details Создает пустой CAN-фрейм с нулевыми значениями всех полей
         */
        CanFrame() noexcept;

        /**
         * @brief Конструктор из структуры twai_message_t
         * @param[in] msg Сообщение в формате TWAI
         */
        explicit CanFrame(const twai_message_t& msg);

        /**
         * @brief Очистка фрейма
         * @details Устанавливает все поля фрейма в нулевые значения
         */
        void clear() noexcept;

        /**
         * @brief Проверка наличия данных
         * @return true если фрейм содержит данные (length > 0)
         */
        [[nodiscard]] bool hasData() const noexcept;

        /**
         * @brief Преобразует CAN-фрейм в структуру TWAI сообщения
         * @return twai_message_t Готовая структура сообщения для передачи через TWAI драйвер
         */
        [[nodiscard]] twai_message_t getTwaiMessage() const;

        /**
         * @brief Получение 16-битного значения
         * @param[in] index Начальный индекс в массиве данных (0-6)
         * @return 16-битное значение
         */
        [[nodiscard]] uint16_t getWord(int index) const noexcept;

        /**
         * @brief Сравнение фреймов
         * @param[in] frame Фрейм для сравнения
         * @return true если фреймы идентичны (совпадают все поля)
         */
        [[nodiscard]] bool compare(const CanFrame& frame) const noexcept;

        /**
         * @brief Получение байтов по битовым индексам
         * @param[in] indexes Массив индексов битов
         * @param[in] size Размер массива индексов
         * @return Объединение с полученными данными
         */
        Bytes getBytes(const int indexes[], size_t size) noexcept;

        uint32_t id = 0;       ///< 11- или 29-битный идентификатор
        Bytes data{};          ///< Данные фрейма (до 8 байт)
        uint8_t length = 0;    ///< Длина данных (0-8)
        bool extended = false; ///< Флаг расширенного формата (29 бит)
        bool rtr = false;      ///< Флаг удаленного запроса (Remote Transmission Request)
    };
} // namespace canbus

#endif // CANBUS_CAN_FRAME_H
