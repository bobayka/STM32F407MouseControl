#include "stm32f4xx_hal.h"
#include "main.h"
#ifndef _Reorientation_H
#define _Reorientation_H

extern volatile float q0, q1, q2, q3;

void Reorientation_by_quaternion (struct Accel* AccData);
#endif
