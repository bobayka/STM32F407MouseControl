#include "main.h"
#include "stdint.h"
#ifndef _HEADING_H
#define _HEADING_H

void Heading(struct FinishAngle const* finangl, struct Magnet* magnData);
void getMagnetAngles(struct Magnet const* magnData,float* magnangle);
#endif
