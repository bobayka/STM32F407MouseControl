#include "Heading.h"



void Heading(struct AccelAngle const* finangl, struct Magnet* magnData){
  float* mx = &magnData->x;
	float* my = &magnData->y;
	float* mz = &magnData->z;
	
  float values[3] = {*mx, *my, *mz};
  float z_angle = finangl->rotate.z * Pi/180;
  float x_angle = finangl->rotate.x * Pi/180;

  *mx = values[0]*cos(-z_angle)-values[1]*sin(-z_angle);
  *mz = values[0]*sin(-z_angle)*sin(x_angle)+values[2]*cos(x_angle)+values[1]*sin(x_angle)*cos(-z_angle);

}

void getMagnetAngles(struct Magnet const* magnData,float* magnangle){

  uint16_t f=0;
  if (magnData->x>0)
    f=360;
    
   *magnangle = f-atan2(magnData->x,magnData->z)*180/Pi;
}
