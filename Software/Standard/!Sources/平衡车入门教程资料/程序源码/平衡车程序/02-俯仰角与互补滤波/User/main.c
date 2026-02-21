#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include <math.h>

int16_t AX, AY, AZ, GX, GY, GZ;
uint8_t TimerErrorFlag;
uint16_t TimerCount;

float AngleAcc;
float AngleGyro;
float Angle;

int main(void)
{
	OLED_Init();
	MPU6050_Init();
	BlueSerial_Init();
	
	Timer_Init();
	
	while (1)
	{
		
		OLED_Printf(0, 0, OLED_8X16, "%+06d", AX);
		OLED_Printf(0, 16, OLED_8X16, "%+06d", AY);
		OLED_Printf(0, 32, OLED_8X16, "%+06d", AZ);
		OLED_Printf(64, 0, OLED_8X16, "%+06d", GX);
		OLED_Printf(64, 16, OLED_8X16, "%+06d", GY);
		OLED_Printf(64, 32, OLED_8X16, "%+06d", GZ);
		OLED_Printf(0, 48, OLED_8X16, "Flag:%1d", TimerErrorFlag);
		OLED_Printf(64, 48, OLED_8X16, "C:%05d", TimerCount);
		OLED_Update();
		
		BlueSerial_Printf("[plot,%f,%f,%f]", AngleAcc, AngleGyro, Angle);
	}
}

void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		GY -= 16;
		
		AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
		
		AngleGyro = Angle + GY / 32768.0 * 2000 * 0.001;
		
		float Alpha = 0.001;
		Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
		
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TimerErrorFlag = 1;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		TimerCount = TIM_GetCounter(TIM1);
	}
}
