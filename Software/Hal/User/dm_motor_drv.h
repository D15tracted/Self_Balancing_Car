#ifndef __DM_MOTOR_DRV_H__
#define __DM_MOTOR_DRV_H__

#include "stm32f1xx_hal.h"          // HAL 库头文件
#include "MyCAN.h"                  // 使用 HAL 版本的 MyCAN

// 模式基地址
#define MIT_MODE            0x000
#define POS_MODE            0x100
#define SPD_MODE            0x200
#define PSI_MODE            0x300

// MIT 模式下 Kp/Kd 范围
#define KP_MIN              0.0f
#define KP_MAX              500.0f
#define KD_MIN              0.0f
#define KD_MAX              5.0f

// 电机编号枚举
typedef enum {
    Motor1, Motor2, Motor3, Motor4, Motor5,
    Motor6, Motor7, Motor8, Motor9, Motor10,
    num
} motor_num;

// 控制模式枚举（与电机内部模式对应）
typedef enum {
    mit_mode = 1,
    pos_mode = 2,
    spd_mode = 3,
    psi_mode = 4
} mode_e;

// 寄存器地址定义（读取电机参数用）
typedef enum {
    RID_UV_VALUE = 0,   // 低压保护值
    RID_KT_VALUE = 1,   // 扭矩系数
    RID_OT_VALUE = 2,   // 过温保护值
    RID_OC_VALUE = 3,   // 过流保护值
    RID_ACC      = 4,   // 加速度
    RID_DEC      = 5,   // 减速度
    RID_MAX_SPD  = 6,   // 最大速度
    RID_MST_ID   = 7,   // 反馈ID
    RID_ESC_ID   = 8,   // 接收ID
    RID_TIMEOUT  = 9,   // 超时警报时间
    RID_CMODE    = 10,  // 控制模式
    RID_DAMP     = 11,  // 电机粘滞系数
    RID_INERTIA  = 12,  // 电机转动惯量
    RID_HW_VER   = 13,  // 硬件版本
    RID_SW_VER   = 14,  // 软件版本
    RID_SN       = 15,  // 序列号
    RID_NPP      = 16,  // 极对数
    RID_RS       = 17,  // 电阻
    RID_LS       = 18,  // 电感
    RID_FLUX     = 19,  // 磁链
    RID_GR       = 20,  // 减速比
    RID_PMAX     = 21,  // 位置映射范围
    RID_VMAX     = 22,  // 速度映射范围
    RID_TMAX     = 23,  // 扭矩映射范围
    RID_I_BW     = 24,  // 电流环带宽
    RID_KP_ASR   = 25,  // 速度环 Kp
    RID_KI_ASR   = 26,  // 速度环 Ki
    RID_KP_APR   = 27,  // 位置环 Kp
    RID_KI_APR   = 28,  // 位置环 Ki
    RID_OV_VALUE = 29,  // 过压保护值
    RID_GREF     = 30,  // 齿轮效率
    RID_DETA     = 31,  // 速度环阻尼
    RID_V_BW     = 32,  // 速度环带宽
    RID_IQ_CL    = 33,  // 电流环增强系数
    RID_VL_CL    = 34,  // 速度环增强系数
    RID_CAN_BR   = 35,  // CAN 波特率代码
    RID_SUB_VER  = 36,  // 子版本号
    RID_U_OFF    = 50,  // U 相偏置
    RID_V_OFF    = 51,  // V 相偏置
    RID_K1       = 52,  // 补偿因子1
    RID_K2       = 53,  // 补偿因子2
    RID_M_OFF    = 54,  // 角度偏移
    RID_DIR      = 55,  // 方向
    RID_P_M      = 80,  // 电机位置
    RID_X_OUT    = 81   // 输出轴位置
} rid_e;

// 电机内部参数结构体（从寄存器读取）
typedef struct {
    uint8_t read_flag;      // 当前读取步骤
    uint8_t write_flag;
    uint8_t save_flag;

    float UV_Value;
    float KT_Value;
    float OT_Value;
    float OC_Value;
    float ACC;
    float DEC;
    float MAX_SPD;
    uint32_t MST_ID;
    uint32_t ESC_ID;
    uint32_t TIMEOUT;
    uint32_t cmode;
    float Damp;
    float Inertia;
    uint32_t hw_ver;
    uint32_t sw_ver;
    uint32_t SN;
    uint32_t NPP;
    float Rs;
    float Ls;
    float Flux;
    float Gr;
    float PMAX;      // 位置映射范围
    float VMAX;      // 速度映射范围
    float TMAX;      // 扭矩映射范围
    float I_BW;
    float KP_ASR;
    float KI_ASR;
    float KP_APR;
    float KI_APR;
    float OV_Value;
    float GREF;
    float Deta;
    float V_BW;
    float IQ_cl;
    float VL_cl;
    uint32_t can_br;
    uint32_t sub_ver;
    float u_off;
    float v_off;
    float k1;
    float k2;
    float m_off;
    float dir;
    float p_m;
    float x_out;
} esc_inf_t;

// 电机回传数据（运行反馈）
typedef struct {
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float cur;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

// 控制设定值
typedef struct {
    uint8_t mode;       // 当前控制模式
    float pos_set;
    float vel_set;
    float tor_set;
    float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

// 电机综合结构体
typedef struct {
    uint16_t id;        // CAN ID
    uint16_t mst_id;    // 主控ID（未使用）
    motor_fbpara_t para;
    motor_ctrl_t ctrl;
    esc_inf_t tmp;      // 内部参数
} motor_t;

// 外部声明电机数组
extern motor_t motor[num];

// 函数声明 ----------------------------------------------------------------
void dm3519_current_set(uint16_t id, float m1_cur_set, float m2_cur_set,
                        float m3_cur_set, float m4_cur_set);
void dm3519_fbdata(motor_t *motor, uint8_t *rx_data);

// 数值转换
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

// 电机控制接口
void dm_motor_ctrl_send(motor_t *motor);
void dm_motor_enable(motor_t *motor);
void dm_motor_disable(motor_t *motor);
void dm_motor_clear_para(motor_t *motor);
void dm_motor_clear_err(motor_t *motor);
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

// 模式使能/禁用
void enable_motor_mode(uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(uint16_t motor_id, uint16_t mode_id);

// 各模式控制函数
void mit_ctrl(motor_t *motor, uint16_t motor_id, float pos, float vel,
              float kp, float kd, float tor);
void pos_ctrl(uint16_t motor_id, float pos, float vel);
void spd_ctrl(uint16_t motor_id, float vel);
void psi_ctrl(uint16_t motor_id, float pos, float vel, float cur);

// 特殊命令
void save_pos_zero(uint16_t motor_id, uint16_t mode_id);
void clear_err(uint16_t motor_id, uint16_t mode_id);

// 寄存器读写
void read_motor_data(uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(uint16_t id);
void write_motor_data(uint16_t id, uint8_t rid,
                      uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);

#endif /* __DM_MOTOR_DRV_H__ */
