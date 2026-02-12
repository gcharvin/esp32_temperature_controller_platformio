#include "domain_api.h"
#include "../../config/app_config.h"

#if APP_DOMAIN_TEMPERATURE
#include "../temperature/temperature_domain.h"
#else
#error "No business domain selected. Define APP_DOMAIN_TEMPERATURE or another domain macro."
#endif

const char* domainName() {
#if APP_DOMAIN_TEMPERATURE
    return temperature_domain::name();
#endif
    return "undefined";
}

Parameter* domainParameters() {
#if APP_DOMAIN_TEMPERATURE
    return temperature_domain::parameters();
#endif
    return nullptr;
}

int domainParameterCount() {
#if APP_DOMAIN_TEMPERATURE
    return temperature_domain::parameterCount();
#endif
    return 0;
}

bool domainSetup() {
#if APP_DOMAIN_TEMPERATURE
    return temperature_domain::setup();
#endif
    return false;
}

void domainTick(unsigned long currentTimeMs, bool uiEditing) {
#if APP_DOMAIN_TEMPERATURE
    temperature_domain::tick(currentTimeMs, uiEditing);
#endif
}

void domainOnParameterChanged(int index) {
#if APP_DOMAIN_TEMPERATURE
    temperature_domain::onParameterChanged(index);
#endif
}

bool domainHasError() {
#if APP_DOMAIN_TEMPERATURE
    return temperature_domain::hasError();
#endif
    return true;
}

UiSnapshot domainSnapshot() {
#if APP_DOMAIN_TEMPERATURE
    return temperature_domain::snapshot();
#endif
    return UiSnapshot{"Set", "Process", "Aux", "", 0.0f, 0.0f, 0.0f, 0, false, false};
}
