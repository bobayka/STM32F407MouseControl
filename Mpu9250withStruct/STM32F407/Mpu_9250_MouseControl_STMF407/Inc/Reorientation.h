#include "stm32f4xx_hal.h"
#include "main.h"
#ifndef Reorientation_H
#define Reorientation_H

extern volatile float q0, q1, q2, q3;

void Reorientation_by_quaternion (struct Accel* AccData);
#endif
