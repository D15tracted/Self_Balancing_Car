#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructureA;
	GPIO_InitStructureA.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructureA.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructureA.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructureA);
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructureB;
	GPIO_InitStructureB.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructureB.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructureB.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructureB);
	
	//GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
}

void LED1_ON(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
}

void LED1_OFF(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

void LED1_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) == 0)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	}
	else
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
	}
}

void LED2_ON(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_0);
}

void LED2_OFF(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}

void LED2_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0) == 0)
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	}
	else
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_0);
	}
}

void LED3_ON(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
}

void LED3_OFF(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
}

void LED3_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9) == 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
	}
}

void LED4_ON(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
}

void LED4_OFF(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}

void LED4_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8) == 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_8);
	}
}
