#include "stm32f10x.h"                  // Device header

void Timer_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_InternalClockConfig(TIM1);
	
	TIM_TimeBaseInitTypeDef TIM1_TimeBaseInitStructure;
	TIM1_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM1_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM1_TimeBaseInitStructure.TIM_Period = 1000 - 1;
	TIM1_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM1_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure1;
	NVIC_InitStructure1.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure1.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure1);
	
	TIM_Cmd(TIM1, ENABLE);
}
