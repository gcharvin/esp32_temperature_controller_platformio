#include "app_state.h"

Parameter* parameters = nullptr;
int numParameters = 0;
UiSnapshot uiSnapshot = {
    "Set",
    "Process",
    "Aux",
    "",
    0.0f,
    0.0f,
    0.0f,
    0,
    false,
    false
};
ParameterChangedCallback onParameterChanged = nullptr;
