# ESP32-C3 CANBus Library

![Лицензия](https://img.shields.io/badge/license-Unlicense-blue.svg)
![PlatformIO](https://img.shields.io/badge/platform-ESP32--C3-green.svg)
![Version](https://img.shields.io/badge/version-1.1.0-orange)

Высокопроизводительная библиотека для работы с CAN-шиной через контроллер TWAI на ESP32-C3.

## 🔥 Особенности

- Поддержка стандартных (11-bit) и расширенных (29-bit) идентификаторов
- Гибкая система фильтрации сообщений (до 32 фильтров)
- Асинхронная обработка входящих сообщений через callback-функции
- Поддержка периодической отправки фреймов
- Потокобезопасная очередь отправки
- Встроенный watchdog для автоматического восстановления после ошибок
- Поддержка скоростей от 25 kbit/s до 1 Mbit/s

## 📦 Установка

### Через PlatformIO

Добавьте в ваш `platformio.ini`:
```ini
lib_deps =
    PJ82RU/esp32-c3-canbus
```

### Через Arduino IDE

1. Скачайте [последний релиз](https://github.com/PJ82RU/esp32-c3-canbus/releases)
2. Распакуйте в папку `~/Arduino/libraries/`

## 🚀 Быстрый старт

```cpp
#include "canbus/can.h"

// Инициализация CAN (TX: GPIO5, RX: GPIO6)
canbus::Can can(GPIO_NUM_5, GPIO_NUM_6);

void app_main() {
    // Настройка callback для приема сообщений
    can.bind(std::make_unique<esp32_c3::objects::Callback<canbus::CanFrame>>(
        "CAN_Callback", 5, 10, 3072, 18));
    
    // Запуск CAN на скорости 500 kbit/s
    can.start(canbus::CanSpeed::SPEED_500KBIT);
    
    // Отправка тестового фрейма
    canbus::CanFrame frame;
    frame.id = 0x123;
    frame.data.bytes[0] = 0xAA;
    frame.length = 1;
    can.send(frame);
}
```

## ⚙️ Конфигурация

### Поддерживаемые скорости:

```cpp
enum class CanSpeed {
    SPEED_25KBIT,   // 25 kbit/s
    SPEED_50KBIT,   // 50 kbit/s
    SPEED_100KBIT,  // 100 kbit/s
    SPEED_125KBIT,  // 125 kbit/s
    SPEED_250KBIT,  // 250 kbit/s
    SPEED_500KBIT,  // 500 kbit/s
    SPEED_800KBIT,  // 800 kbit/s
    SPEED_1MBIT     // 1 Mbit/s
};
```

### Настройка фильтров:

```cpp
// Фильтр для ID 0x100-0x1FF (стандартный)
can.setFilter(0x100, 0x700, false);

// Фильтр для ID 0x18000000-0x180000FF (расширенный)
can.setFilter(0x18000000, 0x180000FF, true);
```

## 📚 Документация

### Основные методы:

| Метод                           | Описание                          |
|---------------------------------|-----------------------------------|
| `start(speed)`                  | Запуск CAN-контроллера            |
| `stop()`                        | Остановка CAN-контроллера         |
| `send(frame)`                   | Отправка CAN-фрейма               |
| `bind(callback)`                | Регистрация обработчика сообщений |
| `setFilter(id, mask, extended)` | Установка фильтра                 |

## 🔧 Примеры

### Периодическая отправка:

```cpp
can.bindFrameEntry(
    []() { // Генератор фрейма
        static uint8_t counter = 0;
        canbus::CanFrame frame;
        frame.id = 0x200;
        frame.data.bytes[0] = counter++;
        frame.length = 1;
        return frame;
    },
    100, // Интервал (мс)
    10   // Количество отправок (0 = бесконечно)
);
```

## 🤝 Совместимость

- Поддерживаемые чипы: **ESP32-C3**
- Поддерживаемые фреймворки:
    - ESP-IDF
    - Arduino (с ограничениями)

## 📜 Лицензия

Проект распространяется под лицензией [Unlicense](https://unlicense.org/).

---
Разработано [PJ82](mailto:project82@mail.ru) | [GitHub](https://github.com/PJ82RU/esp32-c3-canbus)

```
