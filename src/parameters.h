#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h>

struct Parameter {
    const char* name;
    float* value;
    float defaultValue;
    float minValue;
    float maxValue;
    float increment;
};

struct UiSnapshot {
    const char* setLabel;
    const char* processLabel;
    const char* auxLabel;
    const char* unit;
    float setpoint;
    float processValue;
    float auxValue;
    int outputPercent;
    bool processValid;
    bool auxValid;
};

typedef void (*ParameterChangedCallback)(int index);

extern Parameter* parameters;
extern int numParameters;
extern UiSnapshot uiSnapshot;
extern ParameterChangedCallback onParameterChanged;

extern volatile int menuIndex;
extern bool menuActive;
extern bool editing;
extern int editIndex;
extern int lastEncoderPosition;
extern unsigned long lastDebounceTime;
extern unsigned long debounceDelay;
extern int cursorRow;
extern bool lcdInitialized;
extern bool displayNeedsUpdate;

#endif
