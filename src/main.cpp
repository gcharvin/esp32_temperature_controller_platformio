#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <ESP32RotaryEncoder.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include "config/app_config.h"
#include "app_state.h"
#include "menu.h"
#include "pkg/domain/domain_api.h"

#define DEBUG true

#define LCD_COLS 20
#define LCD_ROWS 4

Preferences preferences;
hd44780_I2Cexp lcd;
RotaryEncoder rotaryEncoder(kUiPins.encoderA, kUiPins.encoderB, kUiPins.encoderButton);

bool initSuccess = true;
unsigned long lastDisplayTime = 0;
String serialBuffer = "";
uint8_t detectedI2cAddresses[16] = {0};
uint8_t detectedI2cCount = 0;

void debugPrint(const char* message);
void setupLCD();
void scanI2C();
void showDetectedI2CDevices();
void handleSerialCommand();
void processCommand(const String& command);

void setup() {
    Serial.begin(kSerialBaud);
    debugPrint("Init setup");

    setupLCD();

    parameters = domainParameters();
    numParameters = domainParameterCount();
    onParameterChanged = domainOnParameterChanged;

    preferences.begin("my-app", false);
    for (int i = 0; i < numParameters; i++) {
        *(parameters[i].value) = preferences.getFloat(parameters[i].name, parameters[i].defaultValue);
    }

    pinMode(kUiPins.encoderButton, INPUT_PULLUP);
    rotaryEncoder.setEncoderType(EncoderType::HAS_PULLUP);
    rotaryEncoder.setBoundaries(0, numParameters, true);
    rotaryEncoder.onTurned(&knobCallback);
    rotaryEncoder.onPressed(&buttonCallback);
    rotaryEncoder.begin();

    debugPrint("UI ready");

    initSuccess = domainSetup();
    if (!initSuccess) {
        debugPrint("Domain init failed");
        return;
    }

    uiSnapshot = domainSnapshot();
    debugPrint(domainName());
    debugPrint("Init OK");
    delay(500);

    updateDisplay(domainHasError());
}

void loop() {
    if (!initSuccess) {
        return;
    }

    unsigned long currentTime = millis();
    if (Serial.available() > 0) {
        handleSerialCommand();
    }

    bool uiEditingLocked = menuActive && editing;
    domainTick(currentTime, uiEditingLocked);
    uiSnapshot = domainSnapshot();
    bool error = domainHasError();

    if (menuActive) {
        if (editing) {
            if (displayNeedsUpdate) {
                showSingleParameter(editIndex);
                displayNeedsUpdate = false;
            }
        } else {
            if (displayNeedsUpdate) {
                showMenu();
                displayNeedsUpdate = false;
            }
        }
    } else {
        if (currentTime - lastDisplayTime >= kDisplayIntervalMs) {
            lastDisplayTime = currentTime;
            updateDisplay(error);
        }
    }
}

void debugPrint(const char* message) {
    if (DEBUG) {
        Serial.println(message);
    }

    if (lcdInitialized) {
        displayTextLine(message);
        delay(300);
    }
}

void setupLCD() {
    debugPrint("Init LCD");

    if (kI2cConfig.sda < 0 || kI2cConfig.scl < 0) {
        Wire.begin();
    } else {
        Wire.begin(kI2cConfig.sda, kI2cConfig.scl);
    }

    Wire.setClock(kI2cConfig.clockHz);
    Wire.setTimeOut(kI2cConfig.timeoutMs);
    scanI2C();

    int status = lcd.begin(LCD_COLS, LCD_ROWS);
    if (status) {
        lcdInitialized = false;
        debugPrint("LCD init failed");
        return;
    }

    lcdInitialized = true;
    lcd.backlight();
    lcd.clear();
    debugPrint("LCD ready");
    showDetectedI2CDevices();
}

void scanI2C() {
    byte error;
    int nDevices = 0;
    detectedI2cCount = 0;

    Serial.println("I2C scan start");
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            if (detectedI2cCount < sizeof(detectedI2cAddresses)) {
                detectedI2cAddresses[detectedI2cCount++] = address;
            }
            nDevices++;
        }
    }

    if (nDevices == 0) {
        Serial.println("I2C scan: no devices found");
    }
    Serial.println("I2C scan done");
}

void showDetectedI2CDevices() {
    if (!lcdInitialized) {
        return;
    }

    if (detectedI2cCount == 0) {
        debugPrint("I2C: no devices");
        return;
    }

    debugPrint("I2C devices found");

    for (uint8_t i = 0; i < detectedI2cCount; ++i) {
        char line[LCD_COLS + 1];
        uint8_t address = detectedI2cAddresses[i];
        const char* label = "I2C";

        if (address == 0x27) {
            label = "LCD";
        } else if (address == 0x44 || address == 0x45) {
            label = "SHT31";
        } else if (address == 0x5C || address == 0x23) {
            label = "BH1750";
        } else if (address >= 0x48 && address <= 0x4B) {
            label = "TMP117";
        }

        snprintf(line, sizeof(line), "%s: 0x%02X", label, address);
        debugPrint(line);
    }
}

void handleSerialCommand() {
    while (Serial.available() > 0) {
        char receivedChar = Serial.read();
        if (receivedChar == '\n') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                processCommand(serialBuffer);
            }
            serialBuffer = "";
        } else {
            serialBuffer += receivedChar;
        }
    }
}

void processCommand(const String& command) {
    int delimiterIndex = command.indexOf(':');
    if (delimiterIndex == -1) {
        Serial.println("Invalid command. Format: paramname : value");
        return;
    }

    String paramName = command.substring(0, delimiterIndex);
    String paramValueStr = command.substring(delimiterIndex + 1);
    paramName.trim();
    paramValueStr.trim();

    float paramValue = paramValueStr.toFloat();

    for (int i = 0; i < numParameters; i++) {
        if (paramName == parameters[i].name) {
            if (paramValue < parameters[i].minValue || paramValue > parameters[i].maxValue) {
                Serial.print("Value outside bounds for ");
                Serial.print(paramName);
                Serial.print(" [");
                Serial.print(parameters[i].minValue);
                Serial.print(", ");
                Serial.print(parameters[i].maxValue);
                Serial.println("]");
                return;
            }

            *(parameters[i].value) = paramValue;
            preferences.putFloat(parameters[i].name, paramValue);

            if (onParameterChanged != nullptr) {
                onParameterChanged(i);
            }

            Serial.print("Parameter ");
            Serial.print(paramName);
            Serial.print(" updated to ");
            Serial.println(paramValue);
            return;
        }
    }

    Serial.print("Unknown parameter: ");
    Serial.println(paramName);
}
