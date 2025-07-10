#ifndef HARDWARE_CAN_H
#define HARDWARE_CAN_H

#include "can_frame.h"
#include "esp32_c3_objects/thread.h"
#include "esp32_c3_objects/callback.h"
#include "driver/twai.h"

#include <mutex>
#include <cstdint>

namespace canbus
{
    /**
     * @brief Константы CAN-интерфейса
     */
    constexpr uint8_t CAN_NUM_FILTER = 32;            ///< Количество фильтров
    constexpr uint8_t CAN_RX_BUFFER_SIZE = 64;        ///< Размер буфера приема
    constexpr uint16_t CAN_RECEIVE_MS_TO_TICKS = 100; ///< Таймаут приема (мс)
    constexpr uint16_t CAN_SEND_MS_TO_TICKS = 4;      ///< Таймаут отправки (мс)

    /**
     * @brief Скорости CAN-шины
     */
    enum class CanSpeed
    {
        SPEED_25KBIT,  ///< 25 кбит/с
        SPEED_50KBIT,  ///< 50 кбит/с
        SPEED_100KBIT, ///< 100 кбит/с
        SPEED_125KBIT, ///< 125 кбит/с
        SPEED_250KBIT, ///< 250 кбит/с
        SPEED_500KBIT, ///< 500 кбит/с
        SPEED_800KBIT, ///< 800 кбит/с
        SPEED_1MBIT    ///< 1 Мбит/с
    };

    /**
     * @brief Структура фильтра CAN
     */
    struct CanFilter
    {
        uint32_t id = 0;            ///< Идентификатор
        uint32_t mask = 0;          ///< Маска
        int16_t callbackIndex = -1; ///< Индекс callback-функции
        bool configured = false;    ///< Флаг настройки фильтра
        bool extended = false;      ///< Флаг расширенного формата
    };

    /**
     * @brief Класс для работы с CAN-интерфейсом
     */
    class Can
    {
    public:
        /// @brief Тег для логирования
        static constexpr auto TAG = "Can";

        /**
         * @brief Конструктор
         * @param txPin Пин TX
         * @param rxPin Пин RX
         */
        Can(gpio_num_t txPin, gpio_num_t rxPin) noexcept;

        /**
         * @brief Деструктор
         */
        ~Can() noexcept;

        // Запрет копирования
        Can(const Can&) = delete;
        Can& operator=(const Can&) = delete;

        /**
         * @brief Инициализация CAN-интерфейса
         * @param callback Функция обратного вызова
         * @return true если инициализация прошла успешно
         */
        bool begin(esp32_c3::objects::Callback* callback) noexcept;

        /**
         * @brief Деинициализация CAN-интерфейса
         */
        void end() noexcept;

        /**
         * @brief Получить текущее состояние CAN-интерфейса
         * @return Состояние интерфейса
         */
        twai_state_t getState() const noexcept;

        /**
         * @brief Ожидание перехода в рабочее состояние
         * @param timeout Таймаут ожидания (мс)
         * @return true если интерфейс в рабочем состоянии
         */
        bool waitRunning(uint32_t timeout = 0) const noexcept;

        /**
         * @brief Установка скорости CAN-шины
         * @param speed Скорость передачи
         */
        void setSpeed(CanSpeed speed) noexcept;

        /**
         * @brief Установка фильтра
         * @param index Индекс фильтра
         * @param id Идентификатор
         * @param mask Маска
         * @param extended Флаг расширенного формата
         * @param callbackIndex Индекс callback-функции
         * @return Индекс установленного фильтра или -1 при ошибке
         */
        int8_t setFilter(uint8_t index, uint32_t id, uint32_t mask, bool extended, int16_t callbackIndex = -1) noexcept;

        /**
         * @brief Установка фильтра (автовыбор индекса)
         * @param id Идентификатор
         * @param mask Маска
         * @param extended Флаг расширенного формата
         * @param callbackIndex Индекс callback-функции
         * @return Индекс установленного фильтра или -1 при ошибке
         */
        int8_t setFilter(uint32_t id, uint32_t mask, bool extended, int16_t callbackIndex = -1) noexcept;

        /**
         * @brief Получить параметры фильтра
         * @param index Индекс фильтра
         * @return Структура с параметрами фильтра
         */
        CanFilter getFilter(int16_t index) const noexcept;

        /**
         * @brief Очистить все фильтры
         */
        void clearFilters() noexcept;

        /**
         * @brief Отправить CAN-кадр
         * @param frame CAN-кадр для отправки
         * @return true если отправка прошла успешно
         */
        bool send(CanFrame& frame) const noexcept;

        /**
         * @brief Получить CAN-кадр из буфера
         * @param frame CAN-кадр для заполнения
         * @return true если кадр получен
         */
        bool receive(CanFrame& frame) const noexcept;

    protected:
        /**
         * @brief Задача мониторинга состояния CAN-интерфейса (watchdog)
         * @details Бесконечно проверяет состояние шины, инициирует восстановление
         *          при переходе в состояние BUS_OFF
         * @param params Указатель на экземпляр класса Can (this)
         */
        static void CanWatchdogTask(void* params);

        /**
         * @brief Задача приема сообщений с CAN-шины
         * @details Бесконечно обрабатывает входящие сообщения, применяя фильтрацию
         *          и передавая данные через callback-механизм
         * @param params Указатель на экземпляр класса Can (this)
         */
        static void CanReceiveTask(void* params);

        /**
         * @brief Обработчик watchdog-таймера
         */
        [[noreturn]] void handleWatchdog() noexcept;

        /**
         * @brief Обработчик приема сообщений
         * @return true если интерфейс готов к работе
         */
        [[noreturn]] void handleReceive() const noexcept;

    private:
        /**
         * @brief Установка и запуск драйвера TWAI
         * @return true если операция выполнена успешно
         */
        bool installAndStartDriver() noexcept;

        /**
         * @brief Остановка и удаление драйвера TWAI
         */
        void stopAndUninstallDriver() noexcept;

        /**
         * @brief Обработка входящего сообщения
         * @param message Входящее сообщение
         */
        void processFrame(const twai_message_t& message) const noexcept;

        // Потоки
        esp32_c3::objects::Thread mWatchdogThread; ///< Поток мониторинга состояния
        esp32_c3::objects::Thread mReceiveThread;  ///< Поток приема сообщений

        // Примитивные типы
        bool mDriverReady = false;                 ///< Флаг готовности драйвера
        CanSpeed mSpeed = CanSpeed::SPEED_125KBIT; ///< Текущая скорость

        // Указатели
        esp32_c3::objects::Callback* mCallback = nullptr; ///< Callback-механизм

        // Контейнеры
        std::array<CanFilter, CAN_NUM_FILTER> mFilters; ///< Массив фильтров

        // Пользовательские типы
        twai_status_info_t mStatusInfo = {};      ///< Информация о состоянии
        twai_general_config_t mDriverConfig = {}; ///< Конфигурация драйвера
        twai_timing_config_t mTimingConfig = {};  ///< Конфигурация таймингов
        twai_filter_config_t mFilterConfig = {};  ///< Конфигурация фильтра

        // Синхронизация
        mutable std::mutex mMutex; ///< Мьютекс для синхронизации
    };
} // namespace can_bus

#endif // HARDWARE_CAN_H
