# ESP32-C3 CANBus Library

![Лицензия](https://img.shields.io/badge/license-Unlicense-blue.svg)
![PlatformIO](https://img.shields.io/badge/platform-ESP32--C3-green.svg)
![Version](https://img.shields.io/badge/version-1.1.0-orange)

Библиотека для работы с CAN-шиной на ESP32-C3 через TWAI-контроллер.

## Особенности

- Поддержка стандартных (11-bit) и расширенных (29-bit) идентификаторов
- Гибкая система фильтрации сообщений (32 фильтра)
- Callback-механизм для обработки входящих сообщений
- Поддержка всех стандартных скоростей CAN (25 кбит/с - 1 Мбит/с)
- Потокобезопасная реализация
- Интеграция с FreeRTOS

## Установка

### PlatformIO

Добавьте в platformio.ini:

```ini
lib_deps =
    https://github.com/PJ82RU/esp32-c3-canbus.git
```

## Быстрый старт

```cpp
#include "hardware_can.h"

// Пины CAN (замените на актуальные для вашей платы)
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_6

void setupCan() {
    canbus::Can can(CAN_TX_PIN, CAN_RX_PIN);
    
    if(can.begin(nullptr)) { // Без callback
        // Установка скорости (по умолчанию 125 кбит/с)
        can.setSpeed(canbus::CanSpeed::SPEED_250KBIT);
        
        // Отправка тестового фрейма
        canbus::CanFrame frame;
        frame.id = 0x123;
        frame.data.uint32[0] = 0xDEADBEEF;
        frame.length = 4;
        can.send(frame);
    }
}
```

## Лицензия

Данная библиотека распространяется как свободное и бесплатное программное обеспечение, переданное в общественное достояние.

Вы можете свободно копировать, изменять, публиковать, использовать, компилировать, продавать или распространять это программное обеспечение в исходном коде или в виде скомпилированного бинарного файла для любых целей, коммерческих или некоммерческих, и любыми средствами.