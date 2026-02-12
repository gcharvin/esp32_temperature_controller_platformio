#ifndef DOMAIN_API_H
#define DOMAIN_API_H

#include <Arduino.h>
#include "../../parameters.h"

const char* domainName();
Parameter* domainParameters();
int domainParameterCount();
bool domainSetup();
void domainTick(unsigned long currentTimeMs, bool uiEditing);
void domainOnParameterChanged(int index);
bool domainHasError();
UiSnapshot domainSnapshot();

#endif
