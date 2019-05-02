#include "stdint.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_hid.h"

#ifndef _MOUSECONTROL_H
#define _MOUSECONTROL_H

extern USBD_HandleTypeDef hUsbDeviceFS;

void MouseControl(uint8_t* mbuf, struct FinishAngle const* finangl, \
										struct Gyro const* gyrodata ,uint8_t angleMax, float coefX,\
											float coefY, int16_t sensLEFT, int16_t sensRIGHT );
#endif
