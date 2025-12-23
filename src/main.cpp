#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <PID_v2.h>
#include <math.h>
#include <stdlib.h>

#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH 128
#endif

#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT 64
#endif

#ifndef OLED_RESET_PIN
#define OLED_RESET_PIN -1
#endif

#ifndef OLED_I2C_ADDRESS
#define OLED_I2C_ADDRESS 0x3C
#endif

#ifndef DHT_SENSOR_PIN
#define DHT_SENSOR_PIN 4
#endif

#ifndef DHT_SENSOR_TYPE
#define DHT_SENSOR_TYPE DHT22
#endif

#ifndef HEATER_PIN
#define HEATER_PIN 5
#endif

#ifndef PID_PWM_CHANNEL
#define PID_PWM_CHANNEL 0
#endif

#ifndef PID_PWM_FREQUENCY
#define PID_PWM_FREQUENCY 5000
#endif

#ifndef PID_PWM_RESOLUTION
#define PID_PWM_RESOLUTION 10
#endif

#ifndef PID_KP
#define PID_KP 12.0
#endif

#ifndef PID_KI
#define PID_KI 0.5
#endif

#ifndef PID_KD
#define PID_KD 1.2
#endif

#ifndef PID_SAMPLE_TIME_MS
#define PID_SAMPLE_TIME_MS 1000
#endif

#ifndef PID_MIN_SETPOINT
#define PID_MIN_SETPOINT 10.0
#endif

#ifndef PID_MAX_SETPOINT
#define PID_MAX_SETPOINT 80.0
#endif

#ifndef PID_DEFAULT_SETPOINT
#define PID_DEFAULT_SETPOINT 25.0
#endif

#ifndef SENSOR_READ_INTERVAL_MS
#define SENSOR_READ_INTERVAL_MS 2000
#endif

#ifndef DISPLAY_REFRESH_INTERVAL_MS
#define DISPLAY_REFRESH_INTERVAL_MS 1000
#endif

#if defined(PID_v2_h) || defined(PID_v1_h)
#ifndef PID_DIRECTION_DIRECT
#define PID_DIRECTION_DIRECT PID::DIRECT
#endif
#ifndef PID_MODE_AUTOMATIC
#define PID_MODE_AUTOMATIC PID::AUTOMATIC
#endif
#endif

#ifndef PID_DIRECTION_DIRECT
#ifdef DIRECT
#define PID_DIRECTION_DIRECT DIRECT
#else
#define PID_DIRECTION_DIRECT 0
#endif
#endif

#ifndef PID_MODE_AUTOMATIC
#ifdef AUTOMATIC
#define PID_MODE_AUTOMATIC AUTOMATIC
#else
#define PID_MODE_AUTOMATIC 1
#endif
#endif

namespace
{
constexpr uint32_t kPwmMaxDuty = (1UL << PID_PWM_RESOLUTION) - 1UL;
const double kPidOutputMax = static_cast<double>(kPwmMaxDuty);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
DHT dht(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

double targetTemperatureC = PID_DEFAULT_SETPOINT;
double measuredTemperatureC = PID_DEFAULT_SETPOINT;
double pidOutputValue = 0.0;
float lastHumidity = NAN;

PID temperatureController(&measuredTemperatureC,
                          &pidOutputValue,
                          &targetTemperatureC,
                          PID_KP,
                          PID_KI,
                          PID_KD,
                          PID_DIRECTION_DIRECT);

bool displayReady = false;

uint32_t lastSensorReadMs = 0;
uint32_t lastDisplayRefreshMs = 0;

void initializeDisplay()
{
    displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
    if (!displayReady)
    {
        Serial.println(F("[OLED] Unable to initialize display"));
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("ESP32 Temp PID"));
    display.println(F("Display initialised"));
    display.display();
}

void initializeHeaterPwm()
{
    ledcSetup(PID_PWM_CHANNEL, PID_PWM_FREQUENCY, PID_PWM_RESOLUTION);
    ledcAttachPin(HEATER_PIN, PID_PWM_CHANNEL);
    ledcWrite(PID_PWM_CHANNEL, 0);
}

void updateSetpointFromSerial()
{
    if (!Serial.available())
    {
        return;
    }

    const String input = Serial.readStringUntil('\n');
    String trimmed = input;
    trimmed.trim();

    if (trimmed.isEmpty())
    {
        return;
    }

    char *endPtr = nullptr;
    const double candidate = strtod(trimmed.c_str(), &endPtr);

    if (endPtr == trimmed.c_str())
    {
        Serial.println(F("[PID] Ignoring invalid setpoint"));
        return;
    }

    const double clamped = constrain(candidate, PID_MIN_SETPOINT, PID_MAX_SETPOINT);

    targetTemperatureC = clamped;
    Serial.print(F("[PID] New setpoint: "));
    Serial.print(targetTemperatureC, 1);
    Serial.println(F(" C"));
}

void readSensors()
{
    const float temperature = dht.readTemperature();
    const float humidity = dht.readHumidity();

    if (!isnan(temperature))
    {
        measuredTemperatureC = temperature;
    }
    else
    {
        Serial.println(F("[Sensor] Temperature read failed"));
    }

    if (!isnan(humidity))
    {
        lastHumidity = humidity;
    }
    else
    {
        Serial.println(F("[Sensor] Humidity read failed"));
    }
}

void applyPidOutput()
{
    const double clampedOutput = constrain(pidOutputValue, 0.0, kPidOutputMax);
    ledcWrite(PID_PWM_CHANNEL, static_cast<uint32_t>(clampedOutput));
}

void updateDisplay()
{
    if (!displayReady)
    {
        return;
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("Temp: "));
    display.print(measuredTemperatureC, 1);
    display.println(F(" C"));

    display.print(F("Target: "));
    display.print(targetTemperatureC, 1);
    display.println(F(" C"));

    display.print(F("Output: "));
    display.print(pidOutputValue, 0);

    if (!isnan(lastHumidity))
    {
        display.println();
        display.print(F("Humidity: "));
        display.print(lastHumidity, 1);
        display.println(F(" %"));
    }

    display.display();
}

void logStatus()
{
    Serial.print(F("[Status] Temp="));
    Serial.print(measuredTemperatureC, 2);
    Serial.print(F("C, Target="));
    Serial.print(targetTemperatureC, 2);
    Serial.print(F("C, Output="));
    Serial.print(pidOutputValue, 2);

    if (!isnan(lastHumidity))
    {
        Serial.print(F(", Humidity="));
        Serial.print(lastHumidity, 1);
        Serial.print(F("%"));
    }

    Serial.println();
}

} // namespace

void setup()
{
    Serial.begin(9600);

    Serial.println(F("Starting ESP32 temperature controller"));

    dht.begin();
    initializeDisplay();
    pinMode(HEATER_PIN, OUTPUT);
    initializeHeaterPwm();

    temperatureController.SetOutputLimits(0.0, kPidOutputMax);
    temperatureController.SetSampleTime(PID_SAMPLE_TIME_MS);
    temperatureController.SetMode(PID_MODE_AUTOMATIC);

    updateDisplay();
    logStatus();
}

void loop()
{
    const uint32_t now = millis();

    updateSetpointFromSerial();

    if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS)
    {
        lastSensorReadMs = now;
        readSensors();
    }

    if (temperatureController.Compute())
    {
        applyPidOutput();
    }

    if (now - lastDisplayRefreshMs >= DISPLAY_REFRESH_INTERVAL_MS)
    {
        lastDisplayRefreshMs = now;
        updateDisplay();
        logStatus();
    }
}
