#ifndef PWM_HAL_H
#define PWM_HAL_H

#include <Arduino.h>

bool pwmInit(uint8_t pin, uint32_t freqHz, uint8_t resolutionBits, int8_t preferredChannel = -1);
void pwmWriteDuty(uint8_t pin, uint32_t duty);
void pwmDetach(uint8_t pin);
uint32_t pwmMaxDuty(uint8_t resolutionBits);

#endif
