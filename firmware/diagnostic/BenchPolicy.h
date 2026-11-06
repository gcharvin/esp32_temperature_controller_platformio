#pragma once
#include <stdint.h>

namespace bench {
constexpr uint8_t pwmPin = 25, enablePin = 27, faultPin = 33;
constexpr uint8_t encoderA = 4, encoderB = 19, encoderButton = 23;
constexpr uint32_t probeMs = 2000;
constexpr uint8_t maxProbePercent = 20;
constexpr bool probeExpired(uint32_t now, uint32_t start) {
    return uint32_t(now - start) >= probeMs;
}
constexpr uint8_t dutyForPercent(uint8_t percent) {
    return uint8_t((uint16_t(percent) * 255 + 50) / 100);
}
constexpr int32_t signedAdc(uint16_t raw) {
    return raw & 0x8000 ? int32_t(raw) - 65536 : int32_t(raw);
}
static_assert(!probeExpired(1999, 0), "Probe must last at most two seconds");
static_assert(probeExpired(2000, 0), "Probe must stop at deadline");
static_assert(probeExpired(0x7c0, 0xfffffff0), "Timeout must survive millis wrap");
static_assert(dutyForPercent(0) == 0 && dutyForPercent(20) == 51, "PWM bounds");
static_assert(signedAdc(0xffff) == -1 && signedAdc(0x7fff) == 32767, "Signed ADC decode");
}
