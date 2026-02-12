#ifndef APP_STATE_H
#define APP_STATE_H

#include "parameters.h"

extern Parameter* parameters;
extern int numParameters;
extern UiSnapshot uiSnapshot;
extern ParameterChangedCallback onParameterChanged;

#endif
