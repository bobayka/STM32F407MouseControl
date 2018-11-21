#include "main.h"
#include "stdint.h"
#include "KalmanFilter.h"
#ifndef _HEADING_H
#define _HEADING_H

void Heading(struct AccelAngle const* finangl, struct Magnet* magnData);
void getMagnetAngles(struct Magnet const* magnData,float* magnangle);
#endif
