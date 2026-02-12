#ifndef TEMPERATURE_DOMAIN_H
#define TEMPERATURE_DOMAIN_H

#include <Arduino.h>
#include "../../parameters.h"

namespace temperature_domain {

const char* name();
Parameter* parameters();
int parameterCount();
bool setup();
void tick(unsigned long currentTimeMs, bool uiEditing);
void onParameterChanged(int index);
bool hasError();
UiSnapshot snapshot();

}  // namespace temperature_domain

#endif
