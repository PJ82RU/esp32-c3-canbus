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

        static constexpr uint32_t STACK_DEPTH_SEND = 2560;
        static constexpr uint32_t STACK_DEPTH_RECEIVE = 3072;
        static constexpr uint32_t STACK_DEPTH_WATCHDOG = 1536;

        static constexpr uint8_t RX_BUFFER_SIZE = 32;   ///< Размер буфера приема
        static constexpr uint16_t SEND_MS_TO_TICKS = 4; ///< Таймаут отправки (мс)

        static constexpr uint8_t MAX_FILTER_SIZE = 32; ///< Количество фильтров
        static constexpr size_t MAX_QUEUE_SIZE = 16;   ///< Максимальный размер очереди отправки
        static constexpr size_t MAX_FRAMES = 32;       ///< Фиксированный лимит

        static constexpr uint16_t SEND_INTERVAL_MS = 50;      ///< Таймаут отправки (мс)
        static constexpr uint16_t RECEIVE_INTERVAL_MS = 50;   ///< Таймаут приема (мс)
        static constexpr uint16_t WATCHDOG_INTERVAL_MS = 200; ///< Таймаут Watchdog (мс)

        using FrameCallback = esp32_c3::objects::Callback<CanFrame>;
        using ErrorFunction = std::function<void(const CanFrame& frame, esp_err_t ret)>;
        using FrameQueue = esp32_c3::objects::BufferedQueue<CanFrame, MAX_QUEUE_SIZE>;
        using FrameFunction = std::function<CanFrame()>;

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
         * @brief Привязать callback для обработки входящих данных
         * @param callback Уникальный указатель на callback для входящих данных
         * @param error Уникальный указатель на callback для ошибок отправки
         */
        void bind(std::unique_ptr<FrameCallback> callback, ErrorFunction error = nullptr);

        /**
         * @brief Привязывает функцию-генератор CAN-фрейма для периодической отправки
         * @param provider Функция, возвращающая CAN-фрейм для отправки
         * @param intervalMs Интервал отправки в миллисекундах
         * @param sendCount Количество отправок (0 - бесконечно)
         * @param active Флаг активности (true - включить отправку, false - выключить)
         * @param error Функция обработки ошибок отправки
         * @return Индекс созданной записи или -1 при ошибке
         */
        int16_t bindFrameEntry(const FrameFunction& provider,
                               uint16_t intervalMs,
                               uint16_t sendCount = 0,
                               bool active = true,
                               const ErrorFunction& error = nullptr);

        /**
         * @brief Устанавливает активность периодической отправки фрейма
         * @param index Индекс записи в mFrameEntries
         * @param active Флаг активности (true - включить отправку, false - выключить)
         * @return true если изменение прошло успешно, false при неверном индексе
         */
        bool setFrameEntryActive(int16_t index, bool active) noexcept;

        /**
         * @brief Запуск/проверка рабочего потока CAN-интерфейса
         * @return true если поток успешно запущен или уже работает
         */
        bool start(CanSpeed speed = CanSpeed::SPEED_125KBIT);

        /**
         * @brief Остановка рабочего потока CAN-интерфейса
         */
        void stop();

        /**
         * @brief Проверить, инициализирован ли CAN-интерфейс
         * @return bool true если инициализирован
         */
        [[nodiscard]] bool isInitialized() const noexcept;

        /**
         * @brief Получить текущее состояние CAN-интерфейса
         * @return Состояние интерфейса
         */
        [[nodiscard]] twai_state_t state() const noexcept;

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
         * @brief Установка фильтра (автовыбор индекса)
         * @param id Идентификатор
         * @param mask Маска
         * @param extended Флаг расширенного формата
         * @param callbackIndex Индекс callback-функции
         * @return Индекс установленного фильтра или -1 при ошибке
         */
        [[nodiscard]] int16_t setFilter(uint32_t id, uint32_t mask, bool extended, int16_t callbackIndex = -1) noexcept;

        /**
         * @brief Получить параметры фильтра
         * @param index Индекс фильтра
         * @return Структура с параметрами фильтра
         */
        [[nodiscard]] CanFilter getFilter(int16_t index) const noexcept;

        /**
         * @brief Очистить все фильтры
         */
        void clearFilters() noexcept;

        /**
         * @brief Добавить CAN-кадр в очередь отправки (потокобезопасно)
         * @param frame CAN-кадр для отправки
         * @return esp_err_t ESP_OK при успешном добавлении в очередь
         */
        esp_err_t send(const CanFrame& frame);

        /**
         * @brief Получить текущий размер очереди отправки
         * @return size_t Количество пакетов в очереди
         */
        [[nodiscard]] size_t getQueueSize() const;

        /**
         * @brief Очистить очередь отправки
         * @return size_t Количество удалённых пакетов
         */
        size_t clearQueue();

        struct
        {
            mutable std::atomic<uint32_t> txFrames{0};
            mutable std::atomic<uint32_t> rxFrames{0};
            mutable std::atomic<uint32_t> errors{0};
        } mStats;

    protected:
        /**
         * @brief Структура для хранения информации о периодически отправляемых CAN-фреймах
         */
        struct FrameEntry
        {
            FrameFunction frameProvider; ///< Функция-генератор CAN-фрейма
            ErrorFunction errorProvider; ///< Функция обработки ошибок отправки
            uint16_t intervalMs;         ///< Интервал отправки в миллисекундах
            uint64_t nextSendTime;       ///< Время следующей отправки, мс
            uint16_t remainingSends;     ///< Оставшееся количество отправок (0 - бесконечно)
            bool isActive;               ///< Флаг активности записи
        };

        /**
         * @brief Установка и запуск драйвера TWAI
         * @return true если операция выполнена успешно
         */
        bool installAndStartDriver() const noexcept;

        /**
         * @brief Остановка и удаление драйвера TWAI
         */
        void stopAndUninstallDriver() const noexcept;

        /**
         * @brief Внутренняя реализация отправки CAN-кадра
         * @param frame CAN-кадр для отправки
         * @return esp_err_t Результат отправки
         */
        [[nodiscard]] esp_err_t sendImpl(const CanFrame& frame) const;

        /**
         * @brief Обработать очередь отправки
         * @note Отправляет не чаще чем раз в SEND_INTERVAL_US
         */
        void processSendQueue();

        /**
         * @brief Обработка входящего сообщения
         */
        void processReceivedData() noexcept;

        /**
         * @brief Мониторинг состояния CAN-шины и восстановление после ошибок
         */
        void processWatchdog();

        mutable std::recursive_mutex mMutex;       ///< Мьютекс для потокобезопасности
        esp32_c3::objects::Thread mSendThread;     ///< Поток отправки сообщений
        esp32_c3::objects::Thread mReceiveThread;  ///< Поток приема сообщений
        esp32_c3::objects::Thread mWatchdogThread; ///< Поток мониторинга состояния

        std::unique_ptr<FrameCallback> mDataCallback; ///< Callback для данных
        ErrorFunction mErrorCallback;                 ///< Callback для ошибок отправки
        FrameQueue mSendQueue;                        ///< Очередь пакетов на отправку

        std::array<CanFilter, MAX_FILTER_SIZE> mFilters; ///< Массив фильтров для входящих сообщений
        int16_t mActiveFiltersCount = 0;                 ///< Текущее количество активных фильтров

        std::array<FrameEntry, MAX_FRAMES> mFrameEntries; ///< Массив записей периодических отправок
        int16_t mActiveFrameEntriesCount = 0; ///< Текущее количество активных записей периодических отправок
        int16_t mLastProcessedEntryIndex = 0; ///< Индекс последней обработанной записи (для round-robin)

        twai_status_info_t mStatusInfo = {};      ///< Информация о состоянии
        twai_general_config_t mDriverConfig = {}; ///< Конфигурация драйвера
        twai_timing_config_t mTimingConfig = {};  ///< Конфигурация таймингов
        twai_filter_config_t mFilterConfig = {};  ///< Конфигурация фильтра

        uint64_t mNextSendTime = 0; ///< Время следующей отправки (мкс)

    private:
        mutable std::atomic<bool> mDriverReady; ///< Флаг готовности драйвера
    };
} // namespace can_bus

#endif // HARDWARE_CAN_H
