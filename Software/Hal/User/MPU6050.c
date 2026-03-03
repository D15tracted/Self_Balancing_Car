#include "stm32f1xx_hal.h"          // 根据实际MCU系列调整头文件
#include "MPU6050.h"

// 默认I2C引脚定义（用户可在此修改）
#define MPU6050_I2C_SCL_PORT   	GPIOB
#define MPU6050_I2C_SCL_PIN     GPIO_PIN_10
#define MPU6050_I2C_SDA_PORT    GPIOB
#define MPU6050_I2C_SDA_PIN     GPIO_PIN_11

#define MPU6050_ADDRESS         0xD0    // 器件写地址

// 静态I2C底层函数
static void I2C_W_SCL(uint8_t BitValue);
static void I2C_W_SDA(uint8_t BitValue);
static uint8_t I2C_R_SDA(void);
static void I2C_Start(void);
static void I2C_Stop(void);
static void I2C_SendByte(uint8_t Byte);
static uint8_t I2C_ReceiveByte(void);
static void I2C_SendAck(uint8_t AckBit);
static uint8_t I2C_ReceiveAck(void);

// 保留原MyI2C_Init，供外部调用
void MyI2C_Init(void)
{
    // 使能GPIO时钟（默认PB，若修改引脚请在此添加对应时钟）
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // 初始化SCL引脚
    GPIO_InitStruct.Pin = MPU6050_I2C_SCL_PIN;
    HAL_GPIO_Init(MPU6050_I2C_SCL_PORT, &GPIO_InitStruct);

    // 初始化SDA引脚
    GPIO_InitStruct.Pin = MPU6050_I2C_SDA_PIN;
    HAL_GPIO_Init(MPU6050_I2C_SDA_PORT, &GPIO_InitStruct);

    // 释放总线
    I2C_W_SCL(1);
    I2C_W_SDA(1);
}

// ---------- I2C底层实现 ----------
static void I2C_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(MPU6050_I2C_SCL_PORT, MPU6050_I2C_SCL_PIN,
                      (BitValue) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void I2C_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(MPU6050_I2C_SDA_PORT, MPU6050_I2C_SDA_PIN,
                      (BitValue) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t I2C_R_SDA(void)
{
    return (HAL_GPIO_ReadPin(MPU6050_I2C_SDA_PORT, MPU6050_I2C_SDA_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

static void I2C_Start(void)
{
    I2C_W_SDA(1);
    I2C_W_SCL(1);
    I2C_W_SDA(0);
    I2C_W_SCL(0);
}

static void I2C_Stop(void)
{
    I2C_W_SDA(0);
    I2C_W_SCL(1);
    I2C_W_SDA(1);
}

static void I2C_SendByte(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        I2C_W_SDA((Byte & (0x80 >> i)) ? 1 : 0);
        I2C_W_SCL(1);
        I2C_W_SCL(0);
    }
}

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t Byte = 0;
    I2C_W_SDA(1);               // 释放SDA
    for (uint8_t i = 0; i < 8; i++)
    {
        I2C_W_SCL(1);
        if (I2C_R_SDA())
        {
            Byte |= (0x80 >> i);
        }
        I2C_W_SCL(0);
    }
    return Byte;
}

static void I2C_SendAck(uint8_t AckBit)
{
    I2C_W_SDA(AckBit ? 0 : 1);  // 0: ACK, 1: NACK
    I2C_W_SCL(1);
    I2C_W_SCL(0);
}

static uint8_t I2C_ReceiveAck(void)
{
    uint8_t AckBit;
    I2C_W_SDA(1);               // 释放SDA
    I2C_W_SCL(1);
    AckBit = I2C_R_SDA();
    I2C_W_SCL(0);
    return AckBit;              // 0: ACK, 1: NACK
}

// ---------- MPU6050寄存器操作 ----------
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    I2C_Start();
    I2C_SendByte(MPU6050_ADDRESS);
    I2C_ReceiveAck();
    I2C_SendByte(RegAddress);
    I2C_ReceiveAck();
    I2C_SendByte(Data);
    I2C_ReceiveAck();
    I2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    I2C_Start();
    I2C_SendByte(MPU6050_ADDRESS);      // 发送器件写地址
    I2C_ReceiveAck();
    I2C_SendByte(RegAddress);           // 发送寄存器地址
    I2C_ReceiveAck();

    I2C_Start();
    I2C_SendByte(MPU6050_ADDRESS | 0x01); // 发送器件读地址
    I2C_ReceiveAck();
    Data = I2C_ReceiveByte();            // 接收数据
    I2C_SendAck(0);                      // 发送NACK (0表示NACK，结束读取)
    I2C_Stop();

    return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    I2C_Start();
    I2C_SendByte(MPU6050_ADDRESS);      // 发送器件写地址
    I2C_ReceiveAck();
    I2C_SendByte(RegAddress);           // 发送起始寄存器地址
    I2C_ReceiveAck();

    I2C_Start();
    I2C_SendByte(MPU6050_ADDRESS | 0x01); // 发送器件读地址
    I2C_ReceiveAck();

    for (uint8_t i = 0; i < Count; i++)
    {
        DataArray[i] = I2C_ReceiveByte();  // 接收一个字节
        // 前Count-1个字节发送ACK (1表示ACK)，最后一个字节发送NACK (0表示NACK)
        I2C_SendAck((i < Count - 1) ? 1 : 0);
    }
    I2C_Stop();
}

// ---------- MPU6050初始化 ----------
void MPU6050_Init(void)
{
    MyI2C_Init();                               // 初始化I2C引脚
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01); // 唤醒并选择时钟源
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); // 所有轴使能
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x0F); // 采样率分频
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);     // 数字低通滤波
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); // 陀螺仪±2000dps
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // 加速度计±16g
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                      int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);

    *AccX = (int16_t)((Data[0] << 8) | Data[1]);
    *AccY = (int16_t)((Data[2] << 8) | Data[3]);
    *AccZ = (int16_t)((Data[4] << 8) | Data[5]);
    *GyroX = (int16_t)((Data[8] << 8) | Data[9]);
    *GyroY = (int16_t)((Data[10] << 8) | Data[11]);
    *GyroZ = (int16_t)((Data[12] << 8) | Data[13]);
}
