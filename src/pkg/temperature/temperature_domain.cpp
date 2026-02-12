#include "temperature_domain.h"

#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <BH1750.h>
#include <PID_v2.h>
#include <math.h>
#include <string.h>

#include "../../config/app_config.h"
#include "../../core/pwm_hal.h"

namespace temperature_domain {

static constexpr uint8_t kPwmResolutionBits = 8;
static constexpr uint32_t kPwmFrequencyHz = 500;
static constexpr int8_t kPwmChannel = 0;
static constexpr float kVoltageRef = 3.3f;
static constexpr float kAdcMax = 4095.0f;
static constexpr float kMaxSafeTempC = 45.0f;

static float g_setpoint = 32.0f;
static float g_kp = 40.0f;
static float g_ki = 8.0f;
static float g_kd = 0.0f;
static float g_resistorValue = 15000.0f;

static float g_divisorValue = 8400.0f;
static float g_bValue = 3920.0f;

static double g_input = 0.0;
static double g_output = 0.0;
static float g_roomTemperatureC = NAN;

static bool g_error = true;
static bool g_tmp117Initialized = false;
static bool g_bh1750Initialized = false;
static uint8_t g_tmp117Address = 0;
static uint8_t g_bh1750Address = 0;
static unsigned long g_lastControlTime = 0;

static Adafruit_TMP117 g_tmp117;
static BH1750 g_lightMeter;
static PID_v2 g_pid(g_kp, g_ki, g_kd, PID::Direct);

static Parameter g_parameters[] = {
    {"Setpoint", &g_setpoint, 32.0f, 20.0f, 40.0f, 0.1f},
    {"Kp", &g_kp, 40.0f, 0.0f, 100.0f, 0.1f},
    {"Ki", &g_ki, 8.0f, 0.0f, 50.0f, 0.1f},
    {"Kd", &g_kd, 0.0f, 0.0f, 10.0f, 0.1f},
    {"resistorValue", &g_resistorValue, 15000.0f, 1000.0f, 30000.0f, 500.0f},
};

static UiSnapshot g_snapshot = {
    "Set",
    "Current",
    "Room",
    "C",
    0.0f,
    0.0f,
    0.0f,
    0,
    false,
    false,
};

static bool initTMP117() {
    const uint8_t addresses[] = {0x48, 0x49, 0x4A, 0x4B};
    for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
        if (g_tmp117.begin(addresses[i])) {
            g_tmp117Address = addresses[i];
            return true;
        }
    }
    return false;
}

static bool initBH1750() {
    const uint8_t addresses[] = {0x23, 0x5C};
    for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
        if (g_lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, addresses[i])) {
            g_bh1750Address = addresses[i];
            return true;
        }
    }
    return false;
}

static bool readThermistor() {
    int adcValue = analogRead(kTemperaturePins.thermistorAdc);
    float voltage = adcValue * (kVoltageRef / kAdcMax);

    if (voltage == 0.0f) {
        Serial.println("Thermistor: voltage is 0, sensor error");
        return true;
    }

    float thermistorResistance = g_divisorValue * (kVoltageRef / voltage - 1.0f);
    if (thermistorResistance <= 0.0f) {
        return true;
    }

    float tempC = 1.0f / (log(thermistorResistance / g_resistorValue) / g_bValue + 1.0f / 298.15f) - 273.15f;
    if (tempC <= 0.0f) {
        return true;
    }

    g_input = tempC;
    if (tempC > kMaxSafeTempC) {
        Serial.println("Error: process temperature above 45C");
        return true;
    }

    return false;
}

static void updatePid() {
    g_output = g_pid.Run(g_input);
    pwmWriteDuty(kTemperaturePins.heaterPwm, (uint32_t)g_output);
}

static void publishTelemetry() {
    Serial.print("Setpoint: ");
    Serial.print(g_setpoint);
    Serial.print(", Input: ");
    Serial.print(g_input);
    Serial.print(", Output: ");
    Serial.print(g_output);

    for (int i = 0; i < parameterCount(); i++) {
        Serial.print(", ");
        Serial.print(g_parameters[i].name);
        Serial.print(": ");
        Serial.print(*(g_parameters[i].value));
    }
    Serial.println();
}

static void refreshSnapshot() {
    g_snapshot.setpoint = g_setpoint;
    g_snapshot.processValue = (float)g_input;
    g_snapshot.auxValue = g_roomTemperatureC;
    g_snapshot.outputPercent = g_error ? 0 : (int)(g_output / 255.0 * 100.0);
    g_snapshot.processValid = !g_error;
    g_snapshot.auxValid = !isnan(g_roomTemperatureC);
}

const char* name() {
    return "temperature";
}

Parameter* parameters() {
    return g_parameters;
}

int parameterCount() {
    return (int)(sizeof(g_parameters) / sizeof(g_parameters[0]));
}

bool setup() {
    pinMode(kTemperaturePins.heaterPwm, OUTPUT);

    if (!pwmInit(kTemperaturePins.heaterPwm, kPwmFrequencyHz, kPwmResolutionBits, kPwmChannel)) {
        Serial.println("PWM init failed");
        return false;
    }
    pwmWriteDuty(kTemperaturePins.heaterPwm, 0);

    g_pid.SetTunings(g_kp, g_ki, g_kd);
    g_pid.SetOutputLimits(0, 255);

    g_error = readThermistor();
    if (g_error) {
        Serial.println("Thermistor sensor failed");
    } else {
        Serial.println("Thermistor OK");
    }

    g_pid.Start(g_input, 0, g_setpoint);

    g_tmp117Initialized = initTMP117();
    if (g_tmp117Initialized) {
        Serial.print("TMP117 addr: 0x");
        Serial.println(g_tmp117Address, HEX);
    } else {
        Serial.println("TMP117 not detected");
    }

    g_bh1750Initialized = initBH1750();
    if (g_bh1750Initialized) {
        Serial.print("BH1750 addr: 0x");
        Serial.println(g_bh1750Address, HEX);
    } else {
        Serial.println("BH1750 not detected");
    }

    g_roomTemperatureC = NAN;
    refreshSnapshot();
    return true;
}

void tick(unsigned long currentTimeMs, bool uiEditing) {
    if (uiEditing) {
        return;
    }

    if (currentTimeMs - g_lastControlTime < kControlIntervalMs) {
        return;
    }
    g_lastControlTime = currentTimeMs;

    g_error = readThermistor();
    if (!g_error) {
        updatePid();
    } else {
        g_output = 0;
        pwmWriteDuty(kTemperaturePins.heaterPwm, 0);
    }

    if (g_tmp117Initialized) {
        sensors_event_t tempEvent;
        if (g_tmp117.getEvent(&tempEvent)) {
            g_roomTemperatureC = tempEvent.temperature;
        } else {
            g_roomTemperatureC = NAN;
        }
    } else {
        g_roomTemperatureC = NAN;
    }

    if (g_bh1750Initialized) {
        float lux = g_lightMeter.readLightLevel();
        Serial.print("Lux: ");
        Serial.println(lux);
    }

    publishTelemetry();
    refreshSnapshot();
}

void onParameterChanged(int index) {
    if (index < 0 || index >= parameterCount()) {
        return;
    }

    const char* namePtr = g_parameters[index].name;
    if (!namePtr) {
        return;
    }

    if (strcmp(namePtr, "Kp") == 0 || strcmp(namePtr, "Ki") == 0 || strcmp(namePtr, "Kd") == 0 || strcmp(namePtr, "Setpoint") == 0) {
        g_output = 0;
        g_pid.SetTunings(g_kp, g_ki, g_kd);
        g_pid.SetOutputLimits(-1, 0);
        g_pid.SetOutputLimits(0, 255);
        g_pid.Start(g_input, g_output, g_setpoint);
    }

    refreshSnapshot();
}

bool hasError() {
    return g_error;
}

UiSnapshot snapshot() {
    return g_snapshot;
}

}  // namespace temperature_domain
