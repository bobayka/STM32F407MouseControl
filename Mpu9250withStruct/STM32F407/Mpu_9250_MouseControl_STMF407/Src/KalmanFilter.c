#include "KalmanFilter.h"

void KalmanFilterSimple1D(float const data, KalmanFilter* kalman){
		float X0 ,P0;

		X0=F*(kalman->state);
		P0=F*(kalman->covariance)*F+q;
		//measurement update - correction
		float K = H*P0/(H*P0*H + R);
    kalman->state = X0 + K*(data - H*X0);// It is returned value
    kalman->covariance = (1 - K*H)*P0;  	
}
