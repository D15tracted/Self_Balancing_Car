#include "stm32f1xx_hal.h"
#include "usart.h"          // 包含 huart1 句柄声明
#include <stdio.h>
#include <stdarg.h>
#include <string.h>         // for strlen

char Serial_RxPacket[100];
uint8_t Serial_RxFlag;

static uint8_t rxByte;      // 用于接收单字节的缓冲区

/**
  * @brief 初始化串口接收（仅启动接收中断，不重复初始化硬件）
  */
void Serial_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);   // 使能接收中断，每次接收1字节
}

/**
  * @brief 发送单个字节（阻塞）
  */
void Serial_SendByte(uint8_t Byte)
{
    /* 直接操作寄存器，避免 HAL 阻塞 */
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = Byte;
}

/**
  * @brief 发送数组（阻塞）
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

/**
  * @brief 发送字符串（阻塞）
  */
void Serial_SendString(char *String)
{
    while (*String) {
        Serial_SendByte(*String++);
    }
}

/**
  * @brief 计算 X^Y（内部使用）
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/**
  * @brief 发送数字（指定长度，高位补0）
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief 重定向 printf 到串口（通过 fputc）
  */
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

/**
  * @brief 格式化打印（支持可变参数）
  */
void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

/**
  * @brief UART 接收完成回调（由 HAL 库在中断中调用）
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        static uint8_t RxState = 0;
        static uint8_t pRxPacket = 0;
        uint8_t RxData = rxByte;   // 获取刚接收到的字节

        /* 状态机：原帧协议 [ ... ] */
        if (RxState == 0)
        {
            if (RxData == '[' && Serial_RxFlag == 0)   // 只有在标志未被消费时才接收新帧
            {
                RxState = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)
        {
            if (RxData == ']')
            {
                RxState = 0;
                Serial_RxPacket[pRxPacket] = '\0';    // 字符串结尾
                Serial_RxFlag = 1;                     // 置位数据就绪标志
            }
            else
            {
                if (pRxPacket < sizeof(Serial_RxPacket) - 1)   // 防止溢出
                {
                    Serial_RxPacket[pRxPacket] = RxData;
                    pRxPacket++;
                }
            }
        }

        /* 重新启动接收下一个字节 */
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}
