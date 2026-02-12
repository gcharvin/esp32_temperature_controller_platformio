#ifndef MENU_H
#define MENU_H

#include <Preferences.h>
#include <ESP32RotaryEncoder.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include "parameters.h"

extern hd44780_I2Cexp lcd;
extern RotaryEncoder rotaryEncoder;
extern Preferences preferences;

void knobCallback(long value);
void buttonCallback(unsigned long duration);
void showMenu();
void showSingleParameter(int index);
void adjustParameter(int index, int direction);
void updateDisplay(bool error);
void displayTextLine(const char* text);

#endif
