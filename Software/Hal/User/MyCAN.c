#include "stm32f1xx_hal.h"
#include "can.h"      // 包含 CubeMX 生成的 CAN 句柄 hcan

// 如果需要使用 printf 调试，可以包含 <stdio.h>，但此处不强制

/**
 * @brief  CAN 初始化（滤波器、启动外设）
 * @note   假设 CAN 基本参数已由 MX_CAN_Init() 配置完成
 * @param  无
 * @retval 无
 */
void MyCAN_Init(void)
{
    CAN_FilterTypeDef can_filter = {0};

    // 配置滤波器：允许接收所有 ID（与标准库版本行为一致）
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
    {
        Error_Handler();    // 可根据需要替换为自定义错误处理
    }

    // 启动 CAN 外设
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    // 此处不使能接收中断，采用查询方式接收
}

/**
 * @brief  发送 CAN 数据（阻塞方式，带超时）
 * @param  ID      标准 ID（11 位）
 * @param  Length  数据长度（≤8）
 * @param  Data    数据指针
 * @retval 无
 */
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint32_t timeout = 0;

    txHeader.StdId = ID;
    txHeader.ExtId = 0;             // 不使用扩展 ID
    txHeader.IDE = CAN_ID_STD;       // 标准帧
    txHeader.RTR = CAN_RTR_DATA;     // 数据帧
    txHeader.DLC = Length;
    txHeader.TransmitGlobalTime = DISABLE;

    // 添加消息到发送邮箱
    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, Data, &txMailbox) != HAL_OK)
    {
        // 添加失败，直接返回（可根据需要增加错误处理）
        return;
    }

    // 等待发送完成（轮询 RQCP 位）
    // 根据邮箱号选择对应的完成标志位
    switch (txMailbox)
    {
        case 0:
            while (!(hcan.Instance->TSR & CAN_TSR_RQCP0))
            {
                timeout++;
                if (timeout > 1000000) break;   // 超时退出
            }
            // 清除完成标志（写1清除）
            hcan.Instance->TSR |= CAN_TSR_RQCP0;
            break;
        case 1:
            while (!(hcan.Instance->TSR & CAN_TSR_RQCP1))
            {
                timeout++;
                if (timeout > 1000000) break;
            }
            hcan.Instance->TSR |= CAN_TSR_RQCP1;
            break;
        case 2:
            while (!(hcan.Instance->TSR & CAN_TSR_RQCP2))
            {
                timeout++;
                if (timeout > 1000000) break;
            }
            hcan.Instance->TSR |= CAN_TSR_RQCP2;
            break;
        default:
            break;
    }
}

/**
 * @brief  检查是否接收到数据（查询 FIFO0 中的消息数量）
 * @retval 1 有数据，0 无数据
 */
uint8_t MyCAN_ReceiveFlag(void)
{
    // 读取 FIFO0 的 FMP 位（[1:0]），表示消息数量
    uint32_t fifo_level = (hcan.Instance->RF0R & CAN_RF0R_FMP0) >> CAN_RF0R_FMP0_Pos;
    return (fifo_level > 0) ? 1 : 0;
}

/**
 * @brief  接收 CAN 数据（从 FIFO0 读取一条消息）
 * @param  ID      返回的 ID（标准 ID 或扩展 ID）
 * @param  Length  返回的数据长度
 * @param  Data    数据缓冲区（至少 8 字节）
 * @retval 无
 */
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
    CAN_RxHeaderTypeDef rxHeader;

    // 从 FIFO0 读取消息
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, Data) != HAL_OK)
    {
        // 读取失败，置零返回
        *ID = 0;
        *Length = 0;
        return;
    }

    // 返回 ID（根据帧类型）
    if (rxHeader.IDE == CAN_ID_STD)
    {
        *ID = rxHeader.StdId;
    }
    else
    {
        *ID = rxHeader.ExtId;
    }

    // 返回数据长度（仅数据帧有效，远程帧 DLC 可能不为0但无数据）
    if (rxHeader.RTR == CAN_RTR_DATA)
    {
        *Length = rxHeader.DLC;
    }
    else
    {
        *Length = 0;   // 远程帧不携带数据
    }
}
