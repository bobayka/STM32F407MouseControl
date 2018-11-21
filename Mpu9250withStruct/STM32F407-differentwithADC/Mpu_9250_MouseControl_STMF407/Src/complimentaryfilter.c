#include "complimentaryfilter.h"


void Final(float* finangl, float const g, float* state, float const alpha, float dt){ //complementary filter
	
	float gyro_Angle = g * dt + *finangl;
	//float gyro_Z_Angle = gyrodata->z * dt + finangl->rotate.z;
		
	*finangl = alpha * gyro_Angle +(1.0f - alpha) * (*state );//just make struct for KAlman
	//finangl->rotate.z = ALPHA * gyro_Z_Angle +(1.0f - ALPHA) * kalman->Kf_Z.state;  
  
}
