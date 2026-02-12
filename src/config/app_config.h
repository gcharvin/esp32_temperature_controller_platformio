#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

#ifndef APP_DOMAIN_TEMPERATURE
#define APP_DOMAIN_TEMPERATURE 1
#endif

struct UiPinConfig {
    uint8_t encoderA;
    uint8_t encoderB;
    uint8_t encoderButton;
};

struct TemperaturePinConfig {
    uint8_t heaterPwm;
    uint8_t thermistorAdc;
};

struct I2cConfig {
    int8_t sda;
    int8_t scl;
    uint32_t clockHz;
    uint16_t timeoutMs;
};

inline constexpr UiPinConfig kUiPins = {4, 19, 23};
inline constexpr TemperaturePinConfig kTemperaturePins = {5, 15};
inline constexpr I2cConfig kI2cConfig = {-1, -1, 50000, 50};

inline constexpr uint32_t kSerialBaud = 9600;
inline constexpr unsigned long kControlIntervalMs = 500;
inline constexpr unsigned long kDisplayIntervalMs = 500;

#endif
