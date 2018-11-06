/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/


//http://www.avislab.com/blog/mpu-6050_ru/

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Reorientation.h"  
#include "Magnitometer_Calibration.h"

/* USER CODE BEGIN 0 */
#include "usb_device.h"
#include "usbd_hid.h"

#define CURSOR_STEP 1

extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t HID_Buffer[4];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern MPU6050_GYROResult    IntGyroData;
extern MPU6050_ACCResult     IntAccData;
extern MPU6050_MAGNETResult  IntMagnetData;
extern MPU6050_StatusReg     Status;
MPU6050_SensMAGNETResult MagnetSens;

KalmanFilter Kf_X;
KalmanFilter Kf_Y;
KalmanFilter Kf_Z;
KalmanFilter Kf_Magn;



float Magnetbias[]={-95.116583,130.829950,-178.951652};
//Общежитие {-115.930463,124.522963,-197.024123};
float calibration_matrix[3][3]={{3.278374,0.178001,-0.343766},{0.178001,3.385584,0.151403},{-0.343766,0.151403,3.144175}};
//Общежитие  {{2.008512,-0.043947,-0.096401},{-0.043947,1.907540,-0.056098},{-0.096401,-0.056098,1.820542}};

//float a;


float t;
//int cnt=0;

float accel_X_Angle=0,accel_Y_Angle=0,accel_Z_Angle=0;
float gyro_X_Angle=0,gyro_Y_Angle=0,gyro_Z_Angle=0;
float final_X_Angle=0,final_Y_Angle=0,final_Z_Angle=0;
float state=0 , covariance=0;
float magnet_X_Angle,magnet_Y_Angle,magnet_Z_Angle;


float cmx=0,cmy=0,cmz=0;
//int mx,my,mz;

float m;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/


void TIM1_UP_TIM10_IRQHandler(void)
{
  static float  offset_X_Angle, offset_Z_Angle;
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        
        struct Finish finish;
        struct AccelAngle accelAngle;
        
        GetRawGyro(&IntGyroData);// Get three projections of the gyroscope
				GetRawAcc(&IntAccData);// Get three projections of the accelerometer
        ReadStatusReg (&Status);//Get a status bit of magnetometer
        
        if(Status.Data_Ready&0x01){// Interrogation of the status bit of the Magnetometer(If the bit is set, then read the data, and return the sensor to the single mode (see Datasheet on AK8963 6.4.2.)) 
          GetRawMagnet(&IntMagnetData);//read the data
          WriteBits(AK8963_I2C_ADDR, AK8963_CNTL1,7, 8, 0x11) ;//return the sensor to the single mode
        }
        
        finish.magnData.x = convertMagnetData(IntMagnetData.Magnet_X)*((float)(MagnetSens.ASAX-128)/256.0f+1.0f)*4912.0f / 32760.0f;// Data calibrated by the sensor itself
        finish.magnData.y = convertMagnetData(IntMagnetData.Magnet_Y)*((float)(MagnetSens.ASAY-128)/256.0f+1.0f)*4912.0f / 32760.0f;
        finish.magnData.z = convertMagnetData(IntMagnetData.Magnet_Z)*((float)(MagnetSens.ASAZ-128)/256.0f+1.0f)*4912.0f / 32760.0f;
        
        //Magnitometer_Calibration ( &mx, &my,  &mz);
        finish.magnData.x = koefx *(finish.magnData.x + xmOffset);
        finish.magnData.y = koefy *(finish.magnData.y + ymOffset);
        finish.magnData.z = koefz *(finish.magnData.z + zmOffset);
        //rotation of the coordinate system of the magnetometer into the coordinate system of the accelerometer
        float x = finish.magnData.x, y = finish.magnData.y;
        finish.magnData.x = y;
        finish.magnData.y = x;
        finish.magnData.z = - finish.magnData.z;
        //-------------------------------------------------------------------------        
        finish.accelData.x = convertAccData(IntAccData.Accel_X) - 0.0123291016f;// It's raw data without calibration factors
				finish.accelData.y = convertAccData(IntAccData.Accel_Y) - 0.0107421875f;// You can get these coefficients in previous versions of the program 
				finish.accelData.z = convertAccData(IntAccData.Accel_Z) - 0.0264587402f;        
        
        Reorientation_by_quaternion (&finish.accelData);
        //---------------------------------------------------------------------------    
				t = convertTempData(IntAccData.Temperature);// The value of temperature
        
        finish.gyroData.x = convertGyroData(IntGyroData.Gyro_X) - 0.197288513f;// It's raw data without calibration factors
				finish.gyroData.y = convertGyroData(IntGyroData.Gyro_Y) - 0.118911743f;// You can get these coefficients in previous versions of the program
				finish.gyroData.z = convertGyroData(IntGyroData.Gyro_Z) - 0.557388306f;
        //-------------------------------------------------------------------------
        accelAngle.rotate.x = get_X_Rotation(&finish.accelData);// Angles of the accel
        accelAngle.rotate.y = get_Y_Rotation(&finish.accelData);
        accelAngle.rotate.z = get_Z_Rotation(&finish.accelData);
        
        KalmanFilterSimple1D(accelAngle.rotate.x,&Kf_X, 10);//filtered value 
        KalmanFilterSimple1D(accelAngle.rotate.y,&Kf_Y, 10);
        KalmanFilterSimple1D(accelAngle.rotate.z,&Kf_Z, 10);
        //--------------------------------------------------------------------------------
        Final();// final angels
        Heading(Kf_Z.state,Kf_X.state);
       // Reorientation_by_quaternion (&mx, &my, &mz);//функцию нужно доработать
        getMagnetAngles();
        KalmanFilterSimple1D(magnet_Z_Angle,&Kf_Magn.state,&Kf_Magn.covariance, 100); 
        float ALPHA = 0.995f;
        gyro_Y_Angle=gy*dt+final_Y_Angle;
        final_Y_Angle=ALPHA*gyro_Y_Angle+(1.0f-ALPHA)*Kf_Magn.state;       
       //------------------------------------------------------------------------
        MouseControl(HID_Buffer, 15, 0.04, 0.03 , 40, 120);// Function of mouse control
        }
    } 
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

static void Reorientation( float* offset_X_Angle,float* offset_Z_Angle){
  if (HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15)==GPIO_PIN_RESET){
    *offset_X_Angle=final_X_Angle; 
    *offset_Z_Angle=final_Z_Angle; 
  }
  final_X_Angle-=*offset_X_Angle;
  final_Z_Angle-=*offset_Z_Angle;
}


static void MouseControl(uint8_t* mbuf, uint8_t angleMax, float coefX, float coefY, int16_t sensLEFT, int16_t sensRIGHT ){
  
    int8_t signZ, signX;    

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

float convertAccData(int16_t acc) {
	float f_acc = ((float) (acc * 2.0f) / 32768.0f);
	return f_acc;
}

float convertGyroData(int16_t gyro) {
//	float f_gyro = (float) (gyro * 250.0f) / 32768.0f;
  	float f_gyro = (float) (gyro / 131.0f);
	return f_gyro;
}

float convertTempData(int16_t temp)
{
  float f_temp = (temp)/340.0+17;///333.87+21
  return f_temp;
}

float convertMagnetData(uint16_t magn){
//  float f_magn=((((float)magn) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
//    float f_magn=(float)magn * 4912.0f / 32760.0f;
      if(magn>=0x8000)
        	return -((65535 - magn) + 1);
      else
        return magn;
}

float getLength(float ax, float ay, float az) {  
	return ax*ax + ay*ay + az*az;
}
void getMagnetAngles(void){

  uint16_t f=0;
  if (mx>0)
    f=360;
    
   magnet_Z_Angle=f-atan2(mx,mz)*180/Pi;
    

  
}
float get_X_Rotation(struct Accel* AccData){
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

float get_Y_Rotation(struct Accel* AccData){
    float radians;
    float ax = AccData->x;
    float ay = AccData->y;
    float az = AccData->z;
  radians=atan2(ay,sqrt(ax*ax+az*az));

      return (-(float)(radians/Pi)*180);
}
float get_Z_Rotation(struct Accel* AccData){
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
void KalmanFilterSimple1D(float data,KalmanFilter* kalman, int r){
		float X0 ,P0;

		X0=F*(kalman->state);
		P0=F*(kalman->covariance)*F+q;
		//measurement update - correction
		float K = H*P0/(H*P0*H + r);
    kalman->state = X0 + K*(data - H*X0);// It is returned value
    kalman->covariance = (1 - K*H)*P0;  	
}

void Final(void){//complementary filter
  static uint8_t i=0;
  float ALPHA = 0.96f;
  if (i==0){
    gyro_X_Angle=gx*dt+accel_X_Angle;
   
    gyro_Z_Angle=gz*dt+accel_Z_Angle;
    ++i;
  }
  else{
    gyro_X_Angle=gx*dt+final_X_Angle;
      
    if (abs((int)(gyro_X_Angle - accel_X_Angle) > 180))
    gyro_X_Angle = accel_X_Angle;    
            
    gyro_Z_Angle=gz*dt+final_Z_Angle;
      
    final_X_Angle=ALPHA*gyro_X_Angle+(1.0f-ALPHA)*Kf_X.state;  
        
    
        
    
    final_Z_Angle=ALPHA*gyro_Z_Angle+(1.0f-ALPHA)*Kf_Z.state;
  }
  
}
void Heading(float z_angle,float x_angle){
  
  float values[3]={mx,my,mz};
  z_angle*=Pi/180;
  x_angle*=Pi/180;

  mx=values[0]*cos(-z_angle)-values[1]*sin(-z_angle);
  mz=values[0]*sin(-z_angle)*sin(x_angle)+values[2]*cos(x_angle)+values[1]*sin(x_angle)*cos(-z_angle);

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
