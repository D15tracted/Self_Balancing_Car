#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Serial.h"
#include "string.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

void Control_Rx(float data[5])
{
	if (Serial_RxFlag == 1){
			char *Tag = strtok(Serial_RxPacket, ",");
			/*
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
				if (strcmp(Name, "1") == 0 && strcmp(Action, "up") == 0)
				{
					printf("key,1,up\r\n");
				}
				else if (strcmp(Name, "2") == 0 && strcmp(Action, "down") == 0)
				{
					printf("key,2,down\r\n");
				}
			}
			*/
			if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				float Value = atof(strtok(NULL, ","));
				
				data[0] = 1;
				
				if (strcmp(Name, "LmPos") == 0)
				{
					data[1] = 1;
					printf("joystick: LmPos,%f\r\n", Value);
				}
				else if (strcmp(Name, "LmVel") == 0)
				{
					data[1] = 2;
					printf("joystick: LmVel,%f\r\n", Value);
				}
				else if (strcmp(Name, "LmKp") == 0)
				{
					data[1] = 3;
					printf("joystick: LmKp,%f\r\n", Value);
				}
				else if (strcmp(Name, "LmKd") == 0)
				{
					data[1] = 4;
					printf("joystick: LmKd,%f\r\n", Value);
				}
				else if (strcmp(Name, "LmTor") == 0)
				{
					data[1] = 5;
					printf("joystick: LmTor,%f\r\n", Value);
				}
				else{
					data[1] = atoi(Name);
				}
				
				data[2] = Value;
				
			}
			
			else if (strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				printf("joystick,%d,%d,%d,%d\r\n", LH, LV, RH, RV);
				
				data[0] = 2;
				data[1] = LH;
				data[2] = LV;
				data[3] = RH;
				data[4] = RV;
			}
			
			else{
				if (strcmp(Serial_RxPacket, "StopAllShow") == 0){
					data[0] = 3;
					data[1] = 0;
					printf("command: StopAllShow");
				}
				
				if (strcmp(Serial_RxPacket, "ShowPos") == 0){
					data[0] = 3;
					data[1] = 1;
					printf("command: ShowPos");
				}
				
				if (strcmp(Serial_RxPacket, "ShowVel") == 0){
					data[0] = 3;
					data[1] = 2;
					printf("command: ShowVel");
				}
				
				if (strcmp(Serial_RxPacket, "ShowTor") == 0){
					data[0] = 3;
					data[1] = 3;
					printf("command: ShowTor");
				}
				
				if (strcmp(Serial_RxPacket, "ShowAngle") == 0){
					data[0] = 3;
					data[1] = 4;
					printf("command: ShowAngle");
				}
				
				if (strcmp(Serial_RxPacket, "ShowAnglePID") == 0){
					data[0] = 3;
					data[1] = 5;
					printf("command: ShowAnglePID");
				}
				
				if (strcmp(Serial_RxPacket, "ShowSpeedPID") == 0){
					data[0] = 3;
					data[1] = 6;
					printf("command: ShowSpeedPID");
				}
			}
		Serial_RxFlag = 0;
	}
}
