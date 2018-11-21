#include "Reorientation.h"

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
extern uint8_t on;
void Reorientation_by_quaternion (struct Accel* AccData){
  float ax = AccData->x;
  float ay = AccData->y;
  float az = AccData->z;
  float Amplif = sqrt(ax * ax + ay * ay + az * az);
  static float a = 0;
  float x = ax,y = ay,z = az;
  float  q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float magnitude = sqrt(ax * ax + az * az);
  //if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_RESET){
	if(!on){
    a = acos(- ay / Amplif);
    q0 = cos(a/2);
    q1 = sin(a/2) * az / magnitude;
    q2 = 0;
    q3 = -sin(a/2) * ax / magnitude;
  }
//    *ax=(*ax)/Amplif;
//  *ay=(*ay)/Amplif;
//  *az=(*az)/Amplif;
//  q0=cos(a/2);
//  q1=sin(a/2)*(*az)/magnitude;
////  q2=sin(a/2)*(*ay);
//  q2=0;
//  q3=-sin(a/2)*(*ax)/magnitude;
  
  
  q0q1 = q0*q1;
  q0q2 = q0*q2;
  q0q3 = q0*q3;
  q1q1 = q1*q1;
  q1q2 = q1*q2;
  q1q3 = q1*q3;
  q2q2 = q2*q2;
  q2q3 = q2*q3;
  q3q3 = q3*q3;
  
  x = ax * (1 - 2 * (q2q2+q3q3)) + ay * 2 * (q1q2-q0q3) + az * 2 * (q0q2+q1q3);
  y = ax * 2 * (q1q2+q0q3) + ay * (1 - 2 * (q1q1+q3q3)) + az * 2 * (q2q3-q0q1);
  z = ax * 2 * (q1q3+q0q2) + ay * 2 * (q0q1+q2q3) + az * (1 - 2 * (q1q1-q2q2));
  
  AccData->x = x;
  AccData->y = y;
  AccData->z = z;
}















