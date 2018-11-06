
#include "MPU6050.h"
extern MPU6050_SensMAGNETResult MagnetSens;
HAL_StatusTypeDef GetRawAcc(MPU6050_ACCResult *result) 
{
	uint8_t I2C1_Buffer_Rx[8];

	/*Read the 6  accel registers from the device*/
	HAL_StatusTypeDef statResult = I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,
			I2C1_Buffer_Rx, MPU6050_RA_ACCEL_XOUT_H, 8);

	result->Accel_X = (((int16_t) I2C1_Buffer_Rx[0]) << 8)
			| I2C1_Buffer_Rx[1];
	result->Accel_Y = (((int16_t) I2C1_Buffer_Rx[2]) << 8)
			| I2C1_Buffer_Rx[3];
	result->Accel_Z = (((int16_t) I2C1_Buffer_Rx[4]) << 8)
			| I2C1_Buffer_Rx[5];
  result->Temperature = (int16_t)(((int16_t) I2C1_Buffer_Rx[6]) << 8)
			| I2C1_Buffer_Rx[7];
	return statResult;
}
HAL_StatusTypeDef GetTemp(MPU6050_ACCResult *result) 
{
	uint8_t I2C1_Buffer_Rx[2];

	/*Read the 2  temp registers from the device*/
	HAL_StatusTypeDef statResult = I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,
			I2C1_Buffer_Rx, MPU6050_RA_TEMP_OUT_H, 2);

	result->Temperature = (((int16_t) I2C1_Buffer_Rx[0]) << 8)
			| I2C1_Buffer_Rx[1];
	return statResult;
}

HAL_StatusTypeDef GetRawGyro(MPU6050_GYROResult *result) {
	uint8_t I2C1_Buffer_Rx[6];

	/*Read the 6  gyro registers from the device*/
	HAL_StatusTypeDef statResult = I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,
			I2C1_Buffer_Rx, MPU6050_RA_GYRO_XOUT_H, 6);

	result->Gyro_X = (((int16_t) I2C1_Buffer_Rx[0]) << 8)
			| I2C1_Buffer_Rx[1];
	result->Gyro_Y = (((int16_t) I2C1_Buffer_Rx[2]) << 8)
			| I2C1_Buffer_Rx[3];
	result->Gyro_Z = (((int16_t) I2C1_Buffer_Rx[4]) << 8)
			| I2C1_Buffer_Rx[5];
	return statResult;
}
HAL_StatusTypeDef GetRawMagnet(MPU6050_MAGNETResult *result) 
{
	uint8_t I2C1_Buffer_Rx[6];

	/*Read the 6  magnet registers from the device*/
	HAL_StatusTypeDef statResult = I2C_BufferRead(AK8963_I2C_ADDR,
			I2C1_Buffer_Rx, AK8963_HXL, 6);

	result->Magnet_X = (((int16_t) I2C1_Buffer_Rx[1]) << 8)
			| I2C1_Buffer_Rx[0];
	result->Magnet_Y = (((int16_t) I2C1_Buffer_Rx[3]) << 8)
			| I2C1_Buffer_Rx[2];
	result->Magnet_Z = (((int16_t) I2C1_Buffer_Rx[5]) << 8)
			| I2C1_Buffer_Rx[4];
	return statResult;
}

HAL_StatusTypeDef ReadStatusReg (MPU6050_StatusReg *result) 
{
	uint8_t I2C1_Buffer_Rx[1];

	/*Read the 6  magnet registers from the device*/
	HAL_StatusTypeDef statResult = I2C_BufferRead(AK8963_I2C_ADDR,
			I2C1_Buffer_Rx, AK8963_Status_1, 1);

	result->Data_Ready=  I2C1_Buffer_Rx[0] ;

	return statResult;
}

HAL_StatusTypeDef GetSensivityMagnet(MPU6050_SensMAGNETResult *result) 
{
	uint8_t I2C1_Buffer_Rx[3];

	/*Read the 6  magnet registers from the device*/
	HAL_StatusTypeDef statResult = I2C_BufferRead(AK8963_I2C_ADDR,
			I2C1_Buffer_Rx, AK8963_ASA, 3);

	result->ASAX = I2C1_Buffer_Rx[0];
	result->ASAY = I2C1_Buffer_Rx[1];
	result->ASAZ = I2C1_Buffer_Rx[2];
	return statResult;
}

HAL_StatusTypeDef Initialize() {
	HAL_StatusTypeDef result = SetClockSource(MPU6050_CLOCK_INTERNAL);
	if (result == HAL_OK) {
		result = SetFullScaleAccelRange(MPU6050_GYRO_FS_250);
		if (result == HAL_OK) {
			result = SetFullScaleGyroRange(MPU6050_ACCEL_FS_2);
			if (result == HAL_OK) {
        result= WriteBits(MPU6050_DEFAULT_ADDRESS , MPU6050_RA_INT_PIN_CFG,7, 8, 0x02);//By Pass disable				
        if (result == HAL_OK) {
          result = SetSleepModeStatus(DISABLE); /*Take chip off sleepmode*/           
          if (result == HAL_OK) {
            result = SetBandwidthAccel(MPU6050_DLPF_BW_5);            
            if (result == HAL_OK) {
              result = SetBandwidthGyro(MPU6050_DLPF_BW_5);
              if (result == HAL_OK) {
//                result= WriteBits(AK8963_I2C_ADDR, AK8963_CNTL1,7, 8, 0);//magnetometr Записываю все 0 ,для Power-down mode
//                if (result == HAL_OK) {
                  result= WriteBits(AK8963_I2C_ADDR, AK8963_CNTL2,7, 8, AK8963_RESET);
                  HAL_Delay(1);
                  if (result == HAL_OK) {
                    result= WriteBits(AK8963_I2C_ADDR, AK8963_CNTL1,7, 8, 0x0F);
                    if (result == HAL_OK) {
                      result=GetSensivityMagnet(&MagnetSens);                     
                      if (result == HAL_OK) {
                        result= WriteBits(AK8963_I2C_ADDR, AK8963_CNTL1,7, 8, 0x11);
                        return result; //return our result                  
                      }
                    }
                  }
                }
              }
            }
          }
        }
			}
		}
	
	return result;
}

HAL_StatusTypeDef SetBandwidthGyro(uint8_t source) {
	return WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG,
	MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, source);
}
HAL_StatusTypeDef SetBandwidthAccel(uint8_t source) {
	return WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FF_THR,
	MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, source);
}
HAL_StatusTypeDef SetClockSource(uint8_t source) {
	return WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
	MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

HAL_StatusTypeDef SetFullScaleAccelRange(uint8_t range) {
	return WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG,
	MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

HAL_StatusTypeDef SetFullScaleGyroRange(uint8_t range) {
	return WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,
	MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

HAL_StatusTypeDef SetSleepModeStatus(FunctionalState State) {
	return WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
	MPU6050_PWR1_SLEEP_BIT, State);
}
HAL_StatusTypeDef SetModeStatus(FunctionalState State) {
	return WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
	MPU6050_PWR1_SLEEP_BIT, State);
}

HAL_StatusTypeDef WriteBits(uint8_t slaveAddr, uint8_t regAddr,
		uint8_t bitStart, uint8_t length, uint8_t data) {
	uint8_t tmp;
	HAL_StatusTypeDef result = I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);

	if (result != HAL_OK)
		return result;

	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	return I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

HAL_StatusTypeDef WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum,
		uint8_t data) {
	uint8_t tmp;
	HAL_StatusTypeDef result = I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);

	if (result != HAL_OK)
		return result;

	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));

	return I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

HAL_StatusTypeDef ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart,
		uint8_t length, uint8_t *data) {
	//I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
	uint8_t tmp;

	HAL_StatusTypeDef result = I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);

	if (result != HAL_OK)
		return result;

	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	tmp &= mask;
	tmp >>= (bitStart - length + 1);
	*data = tmp;

	return result;
}

HAL_StatusTypeDef I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer,
		uint8_t writeAddr) {
	/*Write data to MPU6050's internal memory*/
	return HAL_I2C_Mem_Write(&hi2c1, (uint16_t) slaveAddr, (uint16_t) writeAddr,
	I2C_MEMADD_SIZE_8BIT, pBuffer, 1, MPU6050_I2C_TIMEOUTLEN);
}

HAL_StatusTypeDef I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer,
		uint8_t readAddr, uint16_t NumByteToRead) {
	/*Read data from MPU6050's internal memory*/
	return HAL_I2C_Mem_Read(&hi2c1, (uint16_t) slaveAddr, (uint16_t) readAddr,
	I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead,
	MPU6050_I2C_TIMEOUTLEN);
}
