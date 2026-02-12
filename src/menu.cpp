#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "menu.h"

#define LCD_COLS 20
#define LCD_ROWS 4
#define MENU_NAME_WIDTH 8

volatile int menuIndex = 0;
bool menuActive = false;
bool editing = false;
int editIndex = -1;
int lastEncoderPosition = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;
int cursorRow = 0;
bool lcdInitialized = false;
bool displayNeedsUpdate = true;

static void lcdPrintLine(uint8_t row, const char* text) {
    char buf[LCD_COLS + 1];
    size_t len = strlen(text);
    if (len > LCD_COLS) {
        len = LCD_COLS;
    }
    memset(buf, ' ', LCD_COLS);
    memcpy(buf, text, len);
    buf[LCD_COLS] = '\0';
    lcd.setCursor(0, row);
    lcd.print(buf);
}

static void lcdClearRows() {
    for (uint8_t row = 0; row < LCD_ROWS; ++row) {
        lcdPrintLine(row, "");
    }
}

void knobCallback(long value) {
    int delta = value - lastEncoderPosition;
    lastEncoderPosition = value;

    if (numParameters > 0 && delta == numParameters) {
        delta = -1;
    }
    if (numParameters > 0 && delta == -numParameters) {
        delta = 1;
    }

    if (editing) {
        if (delta != 0) {
            adjustParameter(editIndex, delta > 0 ? 1 : -1);
            showSingleParameter(editIndex);
        }
    } else if (menuActive) {
        menuIndex = value;
        showMenu();
    }
}

void buttonCallback(unsigned long duration) {
    if (duration > 1000) {
        return;
    }

    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) <= debounceDelay) {
        return;
    }
    lastDebounceTime = currentTime;

    if (!menuActive) {
        menuActive = true;
        menuIndex = 0;
        showMenu();
        delay(100);
        showMenu();
        return;
    }

    if (!editing) {
        if (menuIndex == 0) {
            menuActive = false;
        } else {
            editing = true;
            editIndex = menuIndex - 1;
            showSingleParameter(editIndex);
        }
        return;
    }

    lastEncoderPosition = menuIndex;
    editing = false;
    editIndex = -1;
    showMenu();
}

void showMenu() {
    int visibleItems = LCD_ROWS;
    int topItem = 0;

    if (menuIndex >= visibleItems) {
        topItem = menuIndex - visibleItems + 1;
    }

    for (int i = 0; i < visibleItems; i++) {
        int itemIndex = topItem + i;

        if (itemIndex > numParameters || parameters == nullptr) {
            lcdPrintLine(i, "");
            continue;
        }

        char line[LCD_COLS + 1];
        char selector = (itemIndex == menuIndex) ? '>' : ' ';

        if (itemIndex == 0) {
            snprintf(line, sizeof(line), "%c Back", selector);
        } else {
            char nameBuf[MENU_NAME_WIDTH + 1];
            strncpy(nameBuf, parameters[itemIndex - 1].name, MENU_NAME_WIDTH);
            nameBuf[MENU_NAME_WIDTH] = '\0';
            snprintf(line, sizeof(line), "%c %-8s:%8.2f", selector, nameBuf, *(parameters[itemIndex - 1].value));
        }

        lcdPrintLine(i, line);
    }
}

void showSingleParameter(int index) {
    if (parameters == nullptr || index < 0 || index >= numParameters) {
        return;
    }

    char line[LCD_COLS + 1];

    snprintf(line, sizeof(line), "Set %s", parameters[index].name);
    lcdPrintLine(0, line);

    snprintf(line, sizeof(line), "Value: %.2f", *(parameters[index].value));
    lcdPrintLine(1, line);
    lcdPrintLine(2, "Rotate=Adj");
    lcdPrintLine(3, "Press=Back");
}

void adjustParameter(int index, int direction) {
    if (parameters == nullptr || index < 0 || index >= numParameters) {
        return;
    }

    float newValue = *(parameters[index].value) + direction * parameters[index].increment;
    newValue = constrain(newValue, parameters[index].minValue, parameters[index].maxValue);
    *(parameters[index].value) = newValue;

    preferences.putFloat(parameters[index].name, newValue);
    if (onParameterChanged != nullptr) {
        onParameterChanged(index);
    }
}

void updateDisplay(bool error) {
    char line[LCD_COLS + 1];
    char processBuf[8];
    char auxBuf[8];
    char degreeChar = (char)0xDF;

    if (error || !uiSnapshot.processValid) {
        snprintf(processBuf, sizeof(processBuf), "Err");
    } else {
        snprintf(processBuf, sizeof(processBuf), "%.1f", uiSnapshot.processValue);
    }

    if (!uiSnapshot.auxValid || isnan(uiSnapshot.auxValue)) {
        snprintf(auxBuf, sizeof(auxBuf), "Err");
    } else {
        snprintf(auxBuf, sizeof(auxBuf), "%.1f", uiSnapshot.auxValue);
    }

    const char* setLabel = uiSnapshot.setLabel ? uiSnapshot.setLabel : "Set";
    const char* processLabel = uiSnapshot.processLabel ? uiSnapshot.processLabel : "Current";
    const char* auxLabel = uiSnapshot.auxLabel ? uiSnapshot.auxLabel : "Aux";
    const char* unit = uiSnapshot.unit ? uiSnapshot.unit : "";

    if (strcmp(unit, "C") == 0) {
        snprintf(line, sizeof(line), "%s : %.1f%cC", setLabel, uiSnapshot.setpoint, degreeChar);
        lcdPrintLine(0, line);
        snprintf(line, sizeof(line), "%s : %-5s%cC", processLabel, processBuf, degreeChar);
        lcdPrintLine(1, line);
        snprintf(line, sizeof(line), "%s : %-5s%cC", auxLabel, auxBuf, degreeChar);
        lcdPrintLine(2, line);
    } else {
        snprintf(line, sizeof(line), "%s : %.1f %s", setLabel, uiSnapshot.setpoint, unit);
        lcdPrintLine(0, line);
        snprintf(line, sizeof(line), "%s : %-5s %s", processLabel, processBuf, unit);
        lcdPrintLine(1, line);
        snprintf(line, sizeof(line), "%s : %-5s %s", auxLabel, auxBuf, unit);
        lcdPrintLine(2, line);
    }

    int outputPercentage = error ? 0 : uiSnapshot.outputPercent;
    snprintf(line, sizeof(line), "PWM: %3d%%", outputPercentage);
    lcdPrintLine(3, line);
}

void displayTextLine(const char* text) {
    if (cursorRow >= LCD_ROWS) {
        cursorRow = 0;
    }

    if (cursorRow == 0) {
        lcdClearRows();
    }

    lcdPrintLine(cursorRow, text);
    cursorRow += 1;
}
