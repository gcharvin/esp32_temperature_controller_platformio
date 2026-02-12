#include "pwm_hal.h"

static int8_t g_pwmChannelForPin[40];
static uint8_t g_pwmResolutionBits = 8;
static bool g_pwmMapInit = false;

uint32_t pwmMaxDuty(uint8_t resolutionBits) {
  if (resolutionBits >= 31) {
    return 0x7FFFFFFFUL;
  }
  return (1UL << resolutionBits) - 1UL;
}

static void pwmEnsureMapInit() {
  if (g_pwmMapInit) {
    return;
  }
  for (int i = 0; i < 40; ++i) {
    g_pwmChannelForPin[i] = -1;
  }
  g_pwmMapInit = true;
}

bool pwmInit(uint8_t pin, uint32_t freqHz, uint8_t resolutionBits, int8_t preferredChannel) {
  pwmEnsureMapInit();
  g_pwmResolutionBits = resolutionBits;

  if (pin >= 40) {
    return false;
  }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  bool ok = false;
  if (preferredChannel >= 0) {
    ok = ledcAttachChannel(pin, freqHz, resolutionBits, preferredChannel);
    if (ok) {
      g_pwmChannelForPin[pin] = preferredChannel;
    }
  } else {
    ok = ledcAttach(pin, freqHz, resolutionBits);
    g_pwmChannelForPin[pin] = -1;
  }
  return ok;
#else
  int8_t ch = preferredChannel;
  if (ch < 0) {
    ch = 0;
  }
  ledcSetup((uint8_t)ch, freqHz, resolutionBits);
  ledcAttachPin(pin, (uint8_t)ch);
  g_pwmChannelForPin[pin] = ch;
  return true;
#endif
}

void pwmWriteDuty(uint8_t pin, uint32_t duty) {
  pwmEnsureMapInit();
  const uint32_t maxDuty = pwmMaxDuty(g_pwmResolutionBits);
  if (duty > maxDuty) {
    duty = maxDuty;
  }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(pin, duty);
#else
  if (pin < 40 && g_pwmChannelForPin[pin] >= 0) {
    ledcWrite((uint8_t)g_pwmChannelForPin[pin], duty);
  } else {
    ledcWrite((uint8_t)0, duty);
  }
#endif
}

void pwmDetach(uint8_t pin) {
  pwmEnsureMapInit();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcDetach(pin);
#else
  ledcDetachPin(pin);
#endif
  if (pin < 40) {
    g_pwmChannelForPin[pin] = -1;
  }
}
