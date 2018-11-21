#include "Wheelchair.h"


void Wheelchair( uint16_t* ADC_data, struct FinishAngle const* finangl, uint8_t angleMax, uint8_t angleMin, float coefX, float coefZ){
  
    int8_t signZ, signX;    
		float final_X_Angle = finangl->rotate.x;
		float final_Z_Angle = finangl->rotate.z;
	
    signZ = (int8_t) copysign(1, final_Z_Angle);
    signX = (int8_t) copysign(1, final_X_Angle);
//    else if (gz>20 && final_Z_Angle>0){
//      tickAct=HAL_GetTick();
//    }
    
      ADC_data[1]= 2085 + (int)(coefZ*(final_Z_Angle*final_Z_Angle*signZ + final_Z_Angle) );
      ADC_data[0]= 2200 + (int)(coefX*(final_X_Angle*final_X_Angle*signX + final_X_Angle) );
			if (fabsf(final_Z_Angle)> angleMin)
        ADC_data[1]= 2085 + (int)( final_Z_Angle* 5 *coefZ);
      if (fabsf(final_Z_Angle)>(angleMax))
        ADC_data[1]= 2085 + (int)((signZ * angleMax * angleMax + angleMax) * coefZ);
       if (fabsf(final_X_Angle)>angleMin)
        ADC_data[0]= 2300 + (int)( final_X_Angle* 5* coefX);
      if (fabsf(final_X_Angle)>angleMax)
        ADC_data[0]= 2300 + (int)((signX * angleMax * angleMax + angleMax) * coefX);
       
   
}
