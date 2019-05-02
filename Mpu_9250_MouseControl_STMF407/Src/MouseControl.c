#include "MouseControl.h"

void MouseControl(uint8_t* mbuf, struct FinishAngle const* finangl, struct Gyro const* gyrodata ,uint8_t angleMax, float coefX, float coefY, int16_t sensLEFT, int16_t sensRIGHT ){
  
    int8_t signZ, signX;    
		float final_X_Angle = finangl->rotate.x;
		float final_Z_Angle = finangl->rotate.z;
		float gx = gyrodata->x;
		float gy = gyrodata->y;
		float gz = gyrodata->z;
    signZ = (int8_t) copysign(1, final_Z_Angle);
    signX = (int8_t) copysign(1, final_X_Angle);
    mbuf[0]= 0;//button
    mbuf[1]= 0;//x coordinate
    mbuf[2]= 0;//y coordinate  

    static uint32_t tickBut=0, tickAct=0;
    if ((gy<-sensLEFT || gy>sensRIGHT)&& final_X_Angle<9){
      if (gy<-sensLEFT){
        mbuf[0] = 1;
      } 
      if (gy>sensRIGHT){
        mbuf[0] = 2;
      }
      tickBut=HAL_GetTick();
    }     
//    else if (gz>20 && final_Z_Angle>0){
//      tickAct=HAL_GetTick();
//    }
    else if (HAL_GetTick()-tickBut >= 500 && HAL_GetTick()-tickAct >=400 ) {
    
      mbuf[2]= -(int)(coefY*(final_Z_Angle*final_Z_Angle*signZ + final_Z_Angle) );
      mbuf[1]= -(int)(coefX*(final_X_Angle*final_X_Angle*signX + final_X_Angle) );
      if (fabsf(final_Z_Angle)>(angleMax))
        mbuf[2]= -(int)((signZ*angleMax*angleMax+ angleMax)*coefY);
      
      if (fabsf(final_X_Angle)>angleMax)
        mbuf[1]= -(int)((signX*angleMax*angleMax+ angleMax)*coefX);
    }    
   
    USBD_HID_SendReport(&hUsbDeviceFS, mbuf, 4);
}