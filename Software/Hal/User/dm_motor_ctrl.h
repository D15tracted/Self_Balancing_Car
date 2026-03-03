#ifndef __DM_MOTOR_CTRL_H__
#define __DM_MOTOR_CTRL_H__

#include "dm_motor_drv.h"

// 用于接收数据解析的联合体
typedef union {
    float f_val;
    uint32_t u_val;
    uint8_t b_val[4];
} float_type_u;

// 电机初始化（清空结构体并设置默认参数）
void dm_motor_init(void);

// 轮询读取所有电机内部参数（需配合 receive_motor_data 使用）
void read_all_motor_data(motor_t *motor);

// 处理接收到的参数读取返回数据（在 CAN 接收回调中调用）
void receive_motor_data(motor_t *motor, uint8_t *data);

// CAN 接收回调处理函数（在 HAL_CAN_RxFifo0MsgPendingCallback 中调用）
void Motor_CAN_RxHandler(void);

#endif /* __DM_MOTOR_CTRL_H__ */
