#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "string.h"
#include "control.h"
#include "MyCAN.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "PID.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

uint8_t RunFlag;

int plotnum = 0;

float data[5] = {0,0,0,0,0};

float LmPos = 0;
float LmVel = 0;
float LmKp = 0;
float LmKd = 5;
float LmTor = 0;

float RmPos = 0;
float RmVel = 0;
float RmKp = 0;
float RmKd = 5;
float RmTor = 0;

float AngleActual2 = 0;

float SpeedActual2 = 0;

float TurnActual2 = 0;

float LeftSpeed1 = 0;

float RightSpeed1 = 0;

motor_t *m1;
motor_t *m2;

float AveVel, DifVel;

float LeftSpeed, RightSpeed;
float LeftSpeedFiltered, RightSpeedFiltered;
float AveSpeed;

uint32_t RxID;
uint8_t RxLength;
uint8_t RxData[8];

int16_t AX, AY, AZ, GX, GY, GZ;
uint8_t TimerErrorFlag;
uint16_t TimerCount;

float AngleAcc;
float AngleGyro;
float Angle;

PID_t AnglePID = {
	.Kp = 0.2, //0.2
	.Ki = 0.03,  //0.03
	.Kd = 0.1,  //0.1
	
	.OutMax = 100,
	.OutMin = -100,
	
	.OutOffset = 0,
	
	.ErrorDecayFactor = 1,
	.ErrorIntMax = 400,
	.ErrorIntMin = -400,
};

PID_t SpeedPID = {
	.Kp = 0.8,  //0.8
	.Ki = 0.1,  //0.1
	.Kd = 0.5,  //0.5
	
	.OutMax = 7,
	.OutMin = -7,
	
	.ErrorDecayFactor = 3,
	.ErrorIntMax = 50,
	.ErrorIntMin = -50,
};

int main(void)
{
	OLED_Init();
	Key_Init();
	LED_Init();
	MPU6050_Init();
	Serial_Init();
	MyCAN_Init();
	dm_motor_init();
	
	Timer_Init();

	m1 = &motor[Motor1];
	m1->id = 0x01;
	m1->ctrl.mode = mit_mode;
	m1->para.pos = 0;
	m1->para.vel = 0;
	m1->para.tor = 0;
	
	m2 = &motor[Motor2];
	m2->id = 0x02;
	m2->ctrl.mode = mit_mode;
	m2->para.pos = 0;
	m2->para.vel = 0;
	m2->para.tor = 0;
	
	while (1)
	{
		if (RunFlag) {
		dm_motor_enable(m1);
		dm_motor_enable(m2);
		} 
		else {
		dm_motor_disable(m1);
		dm_motor_disable(m2);
		}
		
		if (RunFlag) {LED1_ON();} else {LED1_OFF();}
		
		if (Key_Check(KEY_1, KEY_DOWN))
		{
			if (RunFlag == 0)
			{
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				RunFlag = 1;
			}
			else
			{
				RunFlag = 0;
			}
		}
		
		OLED_Clear();
		
		if (Serial_RxFlag == 1){
			Control_Rx(data);
		}
		
		if(data[0] == 2){
				SpeedPID.Target = data[2] / 20.0;
				DifVel = -data[3] / 50.0;
		}
		
		if(data[0] == 2){
			if(fabsf(data[1]) > 95 || fabsf(data[4]) > 95){
				RunFlag = 0;
			}
		}
		
		if(data[0] == 3) {plotnum = data[1];}
		
		if(plotnum != 0){
			if(plotnum == 1){
				printf("[plot,%f,%f]", m1->para.pos, m2->para.pos);
			}
			if(plotnum == 2){
				printf("[plot,%f,%f]", m1->para.vel, m2->para.vel);
			}
			if(plotnum == 3){
				printf("[plot,%f,%f]", m1->para.tor, m2->para.tor);
			}
			if(plotnum == 4){
				printf("[plot,%f,%f,%f]", AngleAcc, AngleGyro, Angle);
			}
			if(plotnum == 5){
				printf("[plot,%f,%f,%f,%f,%f]", AnglePID.Kp * AnglePID.Error0, AnglePID.Ki * AnglePID.ErrorInt, AnglePID.Kd * AngleActual2, AnglePID.Target, AnglePID.Actual);
			}
			if(plotnum == 6){
				printf("[plot,%f,%f,%f,%f,%f]", SpeedPID.Kp * SpeedPID.Error0, SpeedPID.Ki * SpeedPID.ErrorInt, SpeedPID.Kd * SpeedActual2, SpeedPID.Target, SpeedPID.Actual);
			}
		}
		
		OLED_Printf(0, 0, OLED_6X8, "P:%05.2fr", m1->para.pos);
		OLED_Printf(0, 8, OLED_6X8, "V:%05.2fr/s", m1->para.vel);
		OLED_Printf(0, 16, OLED_6X8, "T:%05.2fNm", m1->para.tor);
		
		OLED_Printf(0, 32, OLED_6X8, "P:%05.2fr", m2->para.pos);
		OLED_Printf(0, 40, OLED_6X8, "V:%05.2fr/s", m2->para.vel);
		OLED_Printf(0, 48, OLED_6X8, "T:%05.2fNm", m2->para.tor);
		
		OLED_Printf(64, 0, OLED_6X8, "Aa:%05.2f", AngleAcc);
		OLED_Printf(64, 8, OLED_6X8, "Ag:%05.2f", AngleGyro);
		OLED_Printf(64, 16, OLED_6X8, "Af:%05.2f", Angle);
		
		OLED_Printf(64, 32, OLED_6X8, "%05d", AX);
		OLED_Printf(64, 40, OLED_6X8, "%05d",	AY);
		OLED_Printf(64, 48, OLED_6X8, "%05d", AZ);
		OLED_Printf(96, 32, OLED_6X8, "%05d", GX);
		OLED_Printf(96, 40, OLED_6X8, "%05d", GY);
		OLED_Printf(96, 48, OLED_6X8, "%05d", GZ);
		
		OLED_Printf(0, 56, OLED_6X8, "%01d", Serial_RxFlag);
		OLED_Printf(64, 56, OLED_6X8, "%02.2f", SpeedPID.Target);
		OLED_Printf(96, 56, OLED_6X8, "%02.2f", DifVel);

		OLED_Update();
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0, Count1, Count2;
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Key_Tick();
		
		Count2 ++;
		if (Serial_RxFlag == 1) {Count2 = 0;}
		if (Count2 >= 500) 
			{
				Count2 = 0; 
				data[0] = 0; data[1] = 0; data[2] = 0; data[3] = 0; data[4] = 0;
				SpeedPID.Target = 0; 
				DifVel = 0;
			}
		
		Count0 ++;
		if (Count0 >= 10)
		{
			Count0 = 0;
			
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			GY = -GY; 
      GY -= 114; 
			
			AX = -AX;
      AZ = -AZ;
			
			AngleAcc = atan2(AX, AZ) / 3.14159 * 180;
			
			AngleAcc += 3.55;
			
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
			
			float Alpha = 0.003;
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
			
			if (Angle > 15 || Angle < -15)
			{
				RunFlag = 0;
			}
			
			if (RunFlag)
			{
				AnglePID.Actual = Angle;
				AngleActual2 = (AnglePID.Actual - AnglePID.Actual1);
				PID_Update(&AnglePID);
				AveVel = -AnglePID.Out;
				
				LmVel = AveVel + DifVel / 2;
				RmVel = AveVel - DifVel / 2;
				
				if (LmVel > 10) {LmVel = 10;} else if (LmVel < -10) {LmVel = -10;}
				if (RmVel > 10) {RmVel = 10;} else if (RmVel < -10) {RmVel = -10;}
				
				mit_ctrl(m1, 0x01, LmPos, LmVel, LmKp, LmKd, LmTor);
				mit_ctrl(m2, 0x02, RmPos, -RmVel, RmKp, RmKd, RmTor);
			}
			else
			{
				mit_ctrl(m1, 0x01, 0, 0, 0, 0, 0);
				mit_ctrl(m2, 0x02, 0, 0, 0, 0, 0);
			}
		}
		
		Count1 ++;
		if (Count1 >= 10)
		{
			Count1 = 0;
			
			read_motor_ctrl_fbdata(0x01);
			MyCAN_Receive(&RxID, &RxLength, RxData);
			dm_motor_fbdata(m1, RxData);
		
			read_motor_ctrl_fbdata(0x02);
			MyCAN_Receive(&RxID, &RxLength, RxData);
			dm_motor_fbdata(m2, RxData);
		
			float LeftSpeedRaw = m1->para.vel;
			float RightSpeedRaw = m2->para.vel;
    
			float Beta = 0.1;
			LeftSpeedFiltered = Beta * LeftSpeedRaw + (1 - Beta) * LeftSpeedFiltered;
			RightSpeedFiltered = Beta * RightSpeedRaw + (1 - Beta) * RightSpeedFiltered;
    
			LeftSpeed = LeftSpeedFiltered;
			RightSpeed = RightSpeedFiltered;
			
			AveSpeed = (LeftSpeed - RightSpeed) / 2.0;
			
			if (RunFlag)
			{
				SpeedPID.Actual = AveSpeed;
				SpeedActual2 = (SpeedPID.Actual - SpeedPID.Actual1);
				PID_Update(&SpeedPID);
				AnglePID.Target = -SpeedPID.Out;
				
				if(AveSpeed * AnglePID.Target < 0) {
						if(AnglePID.Target > 5) {AnglePID.Target = 5;}
						if(AnglePID.Target < -5) {AnglePID.Target = -5;}
				}
			}
		}
		
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TimerErrorFlag = 1;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		TimerCount = TIM_GetCounter(TIM1);
	}
}
