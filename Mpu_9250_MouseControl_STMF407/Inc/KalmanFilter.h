#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#define q 	1.0f
#define R   10.0f
#define F 	1.0f
#define H   1.0f

typedef struct  {
    
		float state ,covariance;
		
}KalmanFilter;
struct Kfilterforaxis{
	KalmanFilter Kf_X;
	KalmanFilter Kf_Y;
	KalmanFilter Kf_Z;
	KalmanFilter Kf_Magn;
};


void KalmanFilterSimple1D(float const data,KalmanFilter* kalman); //r - ковариация шума измерений
#endif
