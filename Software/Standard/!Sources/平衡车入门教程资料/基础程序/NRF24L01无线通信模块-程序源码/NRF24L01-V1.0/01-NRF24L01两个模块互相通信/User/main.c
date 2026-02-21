#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "NRF24L01.h"

uint8_t KeyNum;

uint8_t SendFlag;								//发送标志位
uint8_t SendSuccessCount, SendFailedCount;		//发送成功计次，发送失败计次

uint8_t ReceiveFlag;							//接收标志位
uint8_t ReceiveSuccessCount, ReceiveFailedCount;//接收成功计次，接收失败计次

int main(void)
{
	/*初始化*/
	OLED_Init();
	Key_Init();
	NRF24L01_Init();
	
	/*OLED显示静态字符串*/
	OLED_ShowString(1, 1, "T:000-000-0");		//格式为：T:发送成功计次-发送失败计次-发送标志位
	OLED_ShowString(3, 1, "R:000-000-0");		//格式为：R:接收成功计次-接收失败计次-接收标志位
	
	/*初始化测试数据，此处值为任意设定，便于观察实验现象*/
	NRF24L01_TxPacket[0] = 0x00;
	NRF24L01_TxPacket[1] = 0x01;
	NRF24L01_TxPacket[2] = 0x02;
	NRF24L01_TxPacket[3] = 0x03;
	
	while (1)
	{
		KeyNum = Key_GetNum();			//读取按键，获取键码
		
		if (KeyNum == 1)				//K1按下
		{
			/*变换测试数据，便于观察实验现象*/
			/*实际项目中，可以将待发送的数据赋值给NRF24L01_TxPacket数组*/
			NRF24L01_TxPacket[0] ++;
			NRF24L01_TxPacket[1] ++;
			NRF24L01_TxPacket[2] ++;
			NRF24L01_TxPacket[3] ++;
			
			/*调用NRF24L01_Send函数，发送数据，同时返回发送标志位，方便用户了解发送状态*/
			/*发送标志位与发送状态的对应关系，可以转到此函数定义上方查看*/
			SendFlag = NRF24L01_Send();
			
			if (SendFlag == 1)			//发送标志位为1，表示发送成功
			{
				SendSuccessCount ++;	//发送成功计次变量自增
			}
			else						//发送标志位不为1，即2/3/4，表示发送不成功
			{
				SendFailedCount ++;		//发送失败计次变量自增
			}
			
			OLED_ShowNum(1, 3, SendSuccessCount, 3);	//显示发送成功次数
			OLED_ShowNum(1, 7, SendFailedCount, 3);		//显示发送失败次数
			OLED_ShowNum(1, 11, SendFlag, 1);			//显示最近一次的发送标志位
			
			/*显示发送数据*/
			OLED_ShowHexNum(2, 1, NRF24L01_TxPacket[0], 2);
			OLED_ShowHexNum(2, 4, NRF24L01_TxPacket[1], 2);
			OLED_ShowHexNum(2, 7, NRF24L01_TxPacket[2], 2);
			OLED_ShowHexNum(2, 10, NRF24L01_TxPacket[3], 2);
			
			/*TX字符串闪烁一次，表明发送了一次数据*/
			OLED_ShowString(1, 15, "TX");
			Delay_ms(100);
			OLED_ShowString(1, 15, "  ");
		}
		
		/*主循环内循环执行NRF24L01_Receive函数，接收数据，同时返回接收标志位，方便用户了解接收状态*/
		/*接收标志位与接收状态的对应关系，可以转到此函数定义上方查看*/
		ReceiveFlag = NRF24L01_Receive();
		
		if (ReceiveFlag)				//接收标志位不为0，表示收到了一个数据包
		{
			if (ReceiveFlag == 1)		//接收标志位为1，表示接收成功
			{
				ReceiveSuccessCount ++;	//接收成功计次变量自增
			}
			else	//接收标志位不为0也不为1，即2/3，表示此次接收产生了错误，错误接收的数据不应该使用
			{
				ReceiveFailedCount ++;	//接收失败计次变量自增
			}
			
			OLED_ShowNum(3, 3, ReceiveSuccessCount, 3);	//显示接收成功次数
			OLED_ShowNum(3, 7, ReceiveFailedCount, 3);	//显示接收失败次数
			OLED_ShowNum(3, 11, ReceiveFlag, 1);		//显示最近一次的接收标志位
			
			/*显示接收数据*/
			OLED_ShowHexNum(4, 1, NRF24L01_RxPacket[0], 2);
			OLED_ShowHexNum(4, 4, NRF24L01_RxPacket[1], 2);
			OLED_ShowHexNum(4, 7, NRF24L01_RxPacket[2], 2);
			OLED_ShowHexNum(4, 10, NRF24L01_RxPacket[3], 2);
			
			/*RX字符串闪烁一次，表明接收到了一次数据*/
			OLED_ShowString(3, 15, "RX");
			Delay_ms(100);
			OLED_ShowString(3, 15, "  ");
		}
	}
}
