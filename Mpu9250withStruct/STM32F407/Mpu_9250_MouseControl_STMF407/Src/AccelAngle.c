#include "AccelAngle.h"

float get_X_Rotation(struct Accel const* AccData){
    float radians;
    float ax = AccData->x;
    float ay = AccData->y;
    float az = AccData->z;
  
  radians=atan2(az,sqrt(ax*ax+ay*ay));  
  if (ay>0){
      if (radians < 0)
        radians=-Pi-atan2(az,sqrt(ax*ax+ay*ay));
      else
        radians=Pi-atan2(az,sqrt(ax*ax+ay*ay));
  }

  return ((float)(radians/Pi)*180);

}

float get_Y_Rotation(struct Accel const* AccData){
    float radians;
    float ax = AccData->x;
    float ay = AccData->y;
    float az = AccData->z;
  radians=atan2(ay,sqrt(ax*ax+az*az));

      return (-(float)(radians/Pi)*180);
}
float get_Z_Rotation(struct Accel const* AccData){
    float radians;
    float ax = AccData->x;
    float ay = AccData->y;
    float az = AccData->z;
    radians=atan2(ax,sqrt(ay*ay+az*az));
  if (ay>0){
      if (radians < 0)
        radians=-Pi-atan2(ax,sqrt(ay*ay+az*az));
      else
        radians=Pi-atan2(ax,sqrt(ay*ay+az*az));
  }

  return ((float)(radians/Pi)*180);
}
