#include "Led.h"
#include "Delay.h"
#include "Key.h"
#include "Sys.h"
#include "Lcd.h"
#include "Usart.h"
#include "MPU6050.h"

uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;

int main(void)
{	 
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
 	LED_Init();			     //LED端口初始化
	LCD_Init();
	MPU6050_Init();
	POINT_COLOR=RED;
	LCD_Scan_Dir(3);
	
	LCD_ShowString(20, 20, 60, 24, 24, "ID:");
	ID = MPU6050_GetID();
	LCD_ShowHexNum(80, 20, ID, 2, 24, 0x80);
	LCD_ShowString(20, 100,  20, 24, 24, "X");
	LCD_ShowString(20, 140, 20, 24, 24, "Y");
	LCD_ShowString(20, 180, 20, 24, 24, "Z");
	LCD_ShowString(60, 60, 100, 24, 24, "ACCEL");
	LCD_ShowString(160, 60, 80, 24, 24, "GYRO");
	
	while(1) 
	{		 
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		LCD_ShowSignedNum(60, 100, AX, 6, 24, 0);
		LCD_ShowSignedNum(60, 140, AY, 6, 24, 0);
		LCD_ShowSignedNum(60, 180, AZ, 6, 24, 0);
		LCD_ShowSignedNum(160, 100, GX, 6, 24, 0);
		LCD_ShowSignedNum(160, 140, GY, 6, 24, 0);
		LCD_ShowSignedNum(160, 180, GZ, 6, 24, 0);
	} 
}
 