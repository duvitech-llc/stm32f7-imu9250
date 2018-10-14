/**
  ******************************************************************************
  * @file    BMP180.c
  * @author  Waveshare Team
  * @version V1.0
  * @date    20-January-2014
  * @brief   This file includes the BMP180 driver functions
  
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
 */
 
 
#include "BMP180.h"

int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD, _oss;  
uint16_t AC4, AC5, AC6;
int32_t B5, UT, UP, Pressure0 = MSLP; 
int32_t PressureVal = 0, TemperatureVal = 0, AltitudeVal = 0;

/**
  * @brief  write an byte to the register adrress of BMP180
  * @param  RegAddr: the register adrress of BMP180
  * @param Data: the data would be writen to the specified register address
  * @retval None
  */
void BMP180_WriteReg(uint8_t RegAddr, uint8_t Val) 
{
	HAL_I2C_Mem_Write(&hi2c1,BMP180_ADDR+1,RegAddr,I2C_MEMADD_SIZE_8BIT,&Val,1,100);
}

/**
  * @brief  Read couples of bytes from the specified register address continuously
  * @param  RegAddr: the specified register adrress of BMP180
  * @param  Num: buffer sizes
  * @param  *pBuffer: points to a buffer to which will be writen 
  * @retval None
  */
void BMP180_ReadReg(uint8_t RegAddr, uint8_t Num, uint8_t *pBuffer) 
{
	HAL_I2C_Mem_Read(&hi2c1,BMP180_ADDR,RegAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,Num,100);
}


///**
//  * @brief  Digital filter
//  * @param *pIndex:
//  * @param *pAvgBuffer:
//  * @param InVal:
//  * @param pOutVal:
//  *
//  * @retval None
//  *               
//  */
//void BMP180_Filter(int32_t InVal, int32_t *pOutVal)
//{	
//	uint8_t i; 
//	uint32_t sum = 0;
//	static uint8_t count = 0,pIndex = 0;
//	static int32_t pBuffer[10];

//	pBuffer[pIndex++] = InVal;
//	if(count < 10 )count++;	
//	if(pIndex == 10)pIndex = 0;
//	for(i = 0; i < count; i ++) 
//	{
//		sum += pBuffer[i];
//	}
//	*pOutVal = sum/count;
//}


/**
  * @brief  Read uncompensated temperature
  * @param  None
  * @retval None
  */
void BMP180_ReadUncompensatedTemperature(void)
{
	uint8_t RegBuff[2];
	
	BMP180_WriteReg(CONTROL, READ_TEMPERATURE); 
	HAL_Delay(5);    //4.5ms
	BMP180_ReadReg(CONTROL_OUTPUT, 2, &RegBuff[0]); 

	UT = ((int32_t)RegBuff[0] << 8) + (int32_t)RegBuff[1]; 
}


/**
  * @brief  Read uncompensated pressure
  * @param  None
  * @retval None
  */
void BMP180_ReadUncompensatedPressure(void)
{
	uint8_t RegBuff[3];
	
	BMP180_WriteReg(CONTROL, READ_PRESSURE + (_oss << 6)); 
	HAL_Delay(8);    //7.5ms
	BMP180_ReadReg(CONTROL_OUTPUT, 3, &RegBuff[0]); 

	UP = (((int32_t)RegBuff[0] << 16) + ((int32_t)RegBuff[1] << 8) + ((int32_t)RegBuff[2])) >> (8 -_oss); // uncompensated pressure value
}


/**
  * @brief  Calculate true temperature
  * @param  *pTrueTemperature: true temperature 
  * @retval None
  */
void BMP180_CalculateTrueTemperature(int32_t *pTrueTemperature)
{
	int32_t X1, X2;
	
	X1 = ((UT - AC6) * AC5) >> 15;
	X2 = (MC << 11) / (X1 + MD);
	B5 = X1 + X2;
	*pTrueTemperature = (B5 + 8) >> 4;
}


/**
  * @brief  Calculate true pressure
  * @param  *pTruePressure: true pressure
  * @retval None
  */
void BMP180_CalculateTruePressure(int32_t *pTruePressure)
{
	int32_t X1, X2, X3, B3, B6, P, Temp;
	uint32_t  B4, B7;
	
	B6 = B5 - 4000;             
	X1 = (B2* ((B6 * B6) >> 12)) >> 11;
	X2 = AC2 * B6 >> 11;
	X3 = X1 + X2;
	Temp = (((int32_t)AC1 << 2) + X3) << _oss;
	B3 = (Temp + 2) >> 2;
	X1 = (AC3 * B6) >> 13;
	X2 = (B1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (AC4 * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> _oss);
	if(B7 < 0x80000000)
	{
		P = (B7 << 1) / B4;
	}	
	else
	{
		P = (B7 / B4) << 1;
	}

	X1 = (P >> 8) * (P >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * P) >> 16;
	
  *pTruePressure = P + ((X1 + X2 + 3791) >> 4);
}


/** 
  * @brief  Calculating pressure at sea level
  * @param  None
  * @retval None
  */
void BMP180_PressureAtSeaLevel(void)
{  
	double Temp = 0.0f;
	int32_t Sum = 0;
	uint8_t i;
	
	for(i = 0; i < 10; i ++)
	{
		
		BMP180_ReadUncompensatedTemperature();
		BMP180_ReadUncompensatedPressure();
		BMP180_CalculateTrueTemperature(&TemperatureVal);
		BMP180_CalculateTruePressure(&PressureVal);

		HAL_Delay(100);
		Sum += PressureVal;
	}
	PressureVal = Sum / 10;
	
	Temp = (float)LOCAL_ADS_ALTITUDE / 4433000;
	Temp = (float)(pow((1 - Temp),5.255f));
	Pressure0 = (PressureVal - PRESSURE_OFFSET) / Temp;
	
}


/** 
  * @brief  Calculating absolute altitude
  * @param  *pAltitude: absolute altitude
  * @param  PressureVal: the pressure at the absolute altitude
  * @retval None
  */
void BMP180_CalculateAbsoluteAltitude(int32_t *pAltitude, int32_t PressureVal)
{
	*pAltitude = 4433000.0 * (1 - pow((PressureVal / (float)Pressure0), 0.1903)); 
}

/**
  * @brief  Read calibration data from the EEPROM of the BMP180
  * @param  None
  * @retval None
  */
void BMP180_ReadCalibrationData(void) 
{
	uint8_t RegBuff[2];

	BMP180_ReadReg(CAL_AC1, 2, RegBuff);
	AC1 = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1]));
	
	BMP180_ReadReg(CAL_AC2, 2, RegBuff);
	AC2 = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1]));
	
	BMP180_ReadReg(CAL_AC3, 2, RegBuff);
	AC3 = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1]));
	
	BMP180_ReadReg(CAL_AC4, 2, RegBuff);
	AC4 = ((uint16_t)RegBuff[0] <<8 | ((uint16_t)RegBuff[1]));
	
	BMP180_ReadReg(CAL_AC5, 2, RegBuff);
	AC5 = ((uint16_t)RegBuff[0] <<8 | ((uint16_t)RegBuff[1]));
	
	BMP180_ReadReg(CAL_AC6, 2, RegBuff);
	AC6 = ((uint16_t)RegBuff[0] <<8 | ((uint16_t)RegBuff[1])); 
	
	BMP180_ReadReg(CAL_B1, 2, RegBuff);
	B1 = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1])); 
	
	BMP180_ReadReg(CAL_B2, 2, RegBuff);
	B2 = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1])); 
	
	BMP180_ReadReg(CAL_MB, 2, RegBuff);
	MB = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1]));
	
	BMP180_ReadReg(CAL_MC, 2, RegBuff);
	MC = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1]));
	
	BMP180_ReadReg(CAL_MD, 2, RegBuff);
	MD = ((int16_t)RegBuff[0] <<8 | ((int16_t)RegBuff[1])); 
		
}


/**
  * @brief  initializes BMP180
  * @param  None
  * @retval None
  */
void BMP180_Init(void) 
{  
	BMP180_ReadCalibrationData();
	BMP180_PressureAtSeaLevel();
}


/**
  * @brief  Calculation of pressure and temperature and altitude for BMP180
  * @param  None
  * @retval None
  */
void CalTemperatureAndPressureAndAltitude(void)
{

	BMP180_ReadUncompensatedTemperature();
	BMP180_ReadUncompensatedPressure();
	
	BMP180_CalculateTrueTemperature(&TemperatureVal);
	BMP180_CalculateTruePressure(&PressureVal);
	BMP180_CalculateAbsoluteAltitude(&AltitudeVal, PressureVal - PRESSURE_OFFSET);
	
}

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/


