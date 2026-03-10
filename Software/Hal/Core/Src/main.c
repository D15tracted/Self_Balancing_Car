/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "MPU6050.h"
#include "Serial.h"
#include "Control.h"
#include "LED.h"
#include "KEY.h"
#include "MyCAN.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "PID.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
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

static uint16_t Count0 = 0, Count1 = 0, Count2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	__HAL_AFIO_REMAP_SWJ_NOJTAG();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	MPU6050_Init();
	Serial_Init();
	MyCAN_Init();
	dm_motor_init();
	HAL_TIM_Base_Start_IT(&htim1);
	
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        // 按键扫描（若 Key 模块依赖定时器）
        Key_Tick();   // 需确认 Key 模块是否已移植

        // 串口接收超时检测（Count2 逻辑）
        Count2++;
        if (Serial_RxFlag == 1) {
            Count2 = 0;
        }
        if (Count2 >= 500) {
            Count2 = 0;
            data[0] = 0; data[1] = 0; data[2] = 0; data[3] = 0; data[4] = 0;
            SpeedPID.Target = 0;
            DifVel = 0;
        }

        // 每 10ms 执行一次（假设定时器周期 1ms）
        Count0++;
        if (Count0 >= 10) {
            Count0 = 0;

            // 读取 MPU6050
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

            // 数据校准（根据原代码）
            GY = -GY;
            GY -= 114;
            AX = -AX;
            AZ = -AZ;

            // 角度计算
            AngleAcc = atan2(AX, AZ) / 3.14159f * 180.0f;
            AngleAcc += 3.55f;
            AngleGyro = Angle + (GY / 32768.0f * 2000.0f * 0.01f);
            float Alpha = 0.003f;
            Angle = Alpha * AngleAcc + (1.0f - Alpha) * AngleGyro;

            // 角度超限保护
            if (Angle > 15.0f || Angle < -15.0f) {
                RunFlag = 0;
            }

            // 电机控制输出
            if (RunFlag) {
                AnglePID.Actual = Angle;
                AngleActual2 = AnglePID.Actual - AnglePID.Actual1;
                PID_Update(&AnglePID);
                AveVel = -AnglePID.Out;

                LmVel = AveVel + DifVel / 2.0f;
                RmVel = AveVel - DifVel / 2.0f;

                // 限幅
                if (LmVel > 10.0f) LmVel = 10.0f;
                else if (LmVel < -10.0f) LmVel = -10.0f;
                if (RmVel > 10.0f) RmVel = 10.0f;
                else if (RmVel < -10.0f) RmVel = -10.0f;

                // 发送 MIT 控制命令
                mit_ctrl(m1, 0x01, LmPos, LmVel, LmKp, LmKd, LmTor);
                mit_ctrl(m2, 0x02, RmPos, -RmVel, RmKp, RmKd, RmTor);
            } else {
                // 停机
                mit_ctrl(m1, 0x01, 0, 0, 0, 0, 0);
                mit_ctrl(m2, 0x02, 0, 0, 0, 0, 0);
            }
        }

        // 每 10ms 读取电机反馈
        Count1++;
        if (Count1 >= 10) {
            Count1 = 0;

            // 发送读取命令
            read_motor_ctrl_fbdata(0x01);
						MyCAN_Receive(&RxID, &RxLength, RxData);
						dm_motor_fbdata(m1, RxData);
		
						read_motor_ctrl_fbdata(0x02);
						MyCAN_Receive(&RxID, &RxLength, RxData);
						dm_motor_fbdata(m2, RxData);

            // 速度滤波
            float LeftSpeedRaw = m1->para.vel;
            float RightSpeedRaw = m2->para.vel;
					
            float Beta = 0.1f;
            LeftSpeedFiltered = Beta * LeftSpeedRaw + (1.0f - Beta) * LeftSpeedFiltered;
            RightSpeedFiltered = Beta * RightSpeedRaw + (1.0f - Beta) * RightSpeedFiltered;
            
						LeftSpeed = LeftSpeedFiltered;
            RightSpeed = RightSpeedFiltered;

            AveSpeed = (LeftSpeed - RightSpeed) / 2.0f;

            if (RunFlag) {
                SpeedPID.Actual = AveSpeed;
                SpeedActual2 = SpeedPID.Actual - SpeedPID.Actual1;
                PID_Update(&SpeedPID);
                AnglePID.Target = -SpeedPID.Out;

                if (AveSpeed * AnglePID.Target < 0) {
                    if (AnglePID.Target > 5.0f) AnglePID.Target = 5.0f;
                    if (AnglePID.Target < -5.0f) AnglePID.Target = -5.0f;
                }
            }
        }

        // 可选：记录定时器计数
        TimerCount = __HAL_TIM_GET_COUNTER(&htim1);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 调用电机驱动接收处理函数
    Motor_CAN_RxHandler();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
