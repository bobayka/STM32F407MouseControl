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
#include "AccelAngle.h"
#include "KalmanFilter.h"
#include "complimentaryfilter.h"
#include "Heading.h"
#include "MouseControl.h"

/* USER CODE BEGIN 0 */


#define CURSOR_STEP 1

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

struct Kfilterforaxis Kalman = {0};
struct FinishAngle finangl;
float t;
float magnet_Y_Angle;
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
        
        if(Status.Data_Ready & 0x01){// Interrogation of the status bit of the Magnetometer(If the bit is set, then read the data, and return the sensor to the single mode (see Datasheet on AK8963 6.4.2.)) 
          GetRawMagnet(&IntMagnetData);//read the data
          WriteBits(AK8963_I2C_ADDR, AK8963_CNTL1,7, 8, 0x11) ;//return the sensor to the single mode
        }
        
        finish.magnData.x = convertMagnetData(IntMagnetData.Magnet_X)*((float)(MagnetSens.ASAX-128)/256.0f+1.0f)*4912.0f / 32760.0f;// Data calibrated by the sensor itself
        finish.magnData.y = convertMagnetData(IntMagnetData.Magnet_Y)*((float)(MagnetSens.ASAY-128)/256.0f+1.0f)*4912.0f / 32760.0f;
        finish.magnData.z = convertMagnetData(IntMagnetData.Magnet_Z)*((float)(MagnetSens.ASAZ-128)/256.0f+1.0f)*4912.0f / 32760.0f;
        
        //Magnitometer_Calibration (&finish.magnData);
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
        //------------------------------GetAccelAngles-------------------------------------
        accelAngle.rotate.x = get_X_Rotation(&finish.accelData);// Angles of the accel
        accelAngle.rotate.y = get_Y_Rotation(&finish.accelData);
        accelAngle.rotate.z = get_Z_Rotation(&finish.accelData);
        //---------------------KalmanFilter-----------------------------------------------
        KalmanFilterSimple1D(accelAngle.rotate.x,&Kalman.Kf_X);//filtered value 
        //KalmanFilterSimple1D(accelAngle.rotate.y,&Kalman.Kf_Y);
        KalmanFilterSimple1D(accelAngle.rotate.z,&Kalman.Kf_Z);
        //---------------------AnglesAfterAllFilter---------------------------------------
        Final(&finangl.rotate.x, finish.gyroData.x, &Kalman.Kf_X.state, 0.96f);// final angels
				Final(&finangl.rotate.z, finish.gyroData.z, &Kalman.Kf_Z.state, 0.96f);// final angels
				//--------------------Heading-----------------------------------------------------
        Heading(&finangl, &finish.magnData);
       // Reorientation_by_quaternion (&mx, &my, &mz);//функцию нужно доработать
        getMagnetAngles(&finish.magnData, &magnet_Y_Angle);
        KalmanFilterSimple1D(magnet_Y_Angle,&Kalman.Kf_Magn); 
				
				Final(&magnet_Y_Angle, finish.gyroData.y, &Kalman.Kf_Magn.state, 0.995f);   
       //------------------------------------------------------------------------
        MouseControl(HID_Buffer, &finangl, &finish.gyroData, 15, 0.04, 0.03 , 40, 120);// Function of mouse control
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

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
