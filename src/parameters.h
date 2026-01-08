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

// Paramètres PID / mesures
extern float Setpoint;
extern float Kp, Ki, Kd;
extern Parameter parameters[];
extern int numParameters;          // <- plus de const ici
extern double Input, Output;
extern float roomTemperatureC;

// (temporaire) état menu/UI
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
