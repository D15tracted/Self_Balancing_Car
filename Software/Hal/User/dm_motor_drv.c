#include "dm_motor_drv.h"
#include <string.h>

// DM3519 四电机电流控制（ID 为 0x280 等，根据实际调整）
void dm3519_current_set(uint16_t id, float m1_cur_set, float m2_cur_set,
                        float m3_cur_set, float m4_cur_set)
{
    uint8_t data[8];
    int16_t m1_cur_tmp = (int16_t)(m1_cur_set * (16384.0f / 20.0f));
    int16_t m2_cur_tmp = (int16_t)(m2_cur_set * (16384.0f / 20.0f));
    int16_t m3_cur_tmp = (int16_t)(m3_cur_set * (16384.0f / 20.0f));
    int16_t m4_cur_tmp = (int16_t)(m4_cur_set * (16384.0f / 20.0f));

    data[0] = m1_cur_tmp >> 8;
    data[1] = m1_cur_tmp & 0xFF;
    data[2] = m2_cur_tmp >> 8;
    data[3] = m2_cur_tmp & 0xFF;
    data[4] = m3_cur_tmp >> 8;
    data[5] = m3_cur_tmp & 0xFF;
    data[6] = m4_cur_tmp >> 8;
    data[7] = m4_cur_tmp & 0xFF;

    MyCAN_Transmit(id, 8, data);
}

// DM3519 反馈数据处理（8字节数据）
void dm3519_fbdata(motor_t *motor, uint8_t *rx_data)
{
    int16_t pos_int16 = (rx_data[0] << 8) | rx_data[1];
    int16_t spd_int16 = (rx_data[2] << 8) | rx_data[3];
    int16_t cur_int16 = (rx_data[4] << 8) | rx_data[5];

    motor->para.pos  = (float)pos_int16 / 8192.0f * 360.0f;   // 圈数 -> 度
    motor->para.vel  = (float)spd_int16;                      // 速度（单位取决于电机）
    motor->para.cur  = (float)cur_int16 / (16384.0f / 20.0f); // 电流（A）
    motor->para.Tcoil = (float)rx_data[6];
    motor->para.Tmos  = (float)rx_data[7];
}

// 浮点数转无符号整数（用于MIT模式打包）
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

// 无符号整数转浮点数（用于MIT模式解包）
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x_int * span / ((float)((1 << bits) - 1)) + offset;
}

// 启用电机模式
void enable_motor_mode(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint16_t id = motor_id + mode_id;
    MyCAN_Transmit(id, 8, data);
}

// 禁用电机模式
void disable_motor_mode(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint16_t id = motor_id + mode_id;
    MyCAN_Transmit(id, 8, data);
}

// 保存当前位置为零点
void save_pos_zero(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint16_t id = motor_id + mode_id;
    MyCAN_Transmit(id, 8, data);
}

// 清除错误
void clear_err(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    uint16_t id = motor_id + mode_id;
    MyCAN_Transmit(id, 8, data);
}

// MIT 模式控制
void mit_ctrl(motor_t *motor, uint16_t motor_id, float pos, float vel,
              float kp, float kd, float tor)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

    data[0] = pos_tmp >> 8;
    data[1] = pos_tmp & 0xFF;
    data[2] = vel_tmp >> 4;
    data[3] = ((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp & 0xFF;
    data[5] = kd_tmp >> 4;
    data[6] = ((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp & 0xFF;

    MyCAN_Transmit(id, 8, data);
}

// 位置模式控制（发送 float 位置和速度）
void pos_ctrl(uint16_t motor_id, float pos, float vel)
{
    uint16_t id = motor_id + POS_MODE;
    uint8_t *pbuf = (uint8_t*)&pos;
    uint8_t *vbuf = (uint8_t*)&vel;
    uint8_t data[8];

    data[0] = pbuf[0];
    data[1] = pbuf[1];
    data[2] = pbuf[2];
    data[3] = pbuf[3];
    data[4] = vbuf[0];
    data[5] = vbuf[1];
    data[6] = vbuf[2];
    data[7] = vbuf[3];

    MyCAN_Transmit(id, 8, data);
}

// 速度模式控制（发送 float 速度）
void spd_ctrl(uint16_t motor_id, float vel)
{
    uint16_t id = motor_id + SPD_MODE;
    uint8_t *vbuf = (uint8_t*)&vel;
    uint8_t data[4];

    data[0] = vbuf[0];
    data[1] = vbuf[1];
    data[2] = vbuf[2];
    data[3] = vbuf[3];

    MyCAN_Transmit(id, 4, data);
}

// 混控模式（位置 + 速度 + 电流）
void psi_ctrl(uint16_t motor_id, float pos, float vel, float cur)
{
    uint16_t id = motor_id + PSI_MODE;
    uint8_t *pbuf = (uint8_t*)&pos;
    uint16_t u16_vel = (uint16_t)(vel * 100);
    uint16_t u16_cur = (uint16_t)(cur * 10000);
    uint8_t *vbuf = (uint8_t*)&u16_vel;
    uint8_t *ibuf = (uint8_t*)&u16_cur;
    uint8_t data[8];

    data[0] = pbuf[0];
    data[1] = pbuf[1];
    data[2] = pbuf[2];
    data[3] = pbuf[3];
    data[4] = vbuf[0];
    data[5] = vbuf[1];
    data[6] = ibuf[0];
    data[7] = ibuf[1];

    MyCAN_Transmit(id, 8, data);
}

// 电机使能（根据当前设定的模式）
void dm_motor_enable(motor_t *motor)
{
    switch (motor->ctrl.mode) {
        case mit_mode:
            enable_motor_mode(motor->id, MIT_MODE);
            break;
        case pos_mode:
            enable_motor_mode(motor->id, POS_MODE);
            break;
        case spd_mode:
            enable_motor_mode(motor->id, SPD_MODE);
            break;
        case psi_mode:
            enable_motor_mode(motor->id, PSI_MODE);
            break;
        default:
            break;
    }
}

// 电机关闭（禁用模式，并清零设定值）
void dm_motor_disable(motor_t *motor)
{
    switch (motor->ctrl.mode) {
        case mit_mode:
            disable_motor_mode(motor->id, MIT_MODE);
            break;
        case pos_mode:
            disable_motor_mode(motor->id, POS_MODE);
            break;
        case spd_mode:
            disable_motor_mode(motor->id, SPD_MODE);
            break;
        case psi_mode:
            disable_motor_mode(motor->id, PSI_MODE);
            break;
        default:
            break;
    }
    dm_motor_clear_para(motor);
}

// 清除控制设定值
void dm_motor_clear_para(motor_t *motor)
{
    motor->ctrl.kd_set  = 0;
    motor->ctrl.kp_set  = 0;
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
    motor->ctrl.cur_set = 0;
}

// 清除电机错误（根据模式发送清除命令）
void dm_motor_clear_err(motor_t *motor)
{
    switch (motor->ctrl.mode) {
        case mit_mode:
            clear_err(motor->id, MIT_MODE);
            break;
        case pos_mode:
            clear_err(motor->id, POS_MODE);
            break;
        case spd_mode:
            clear_err(motor->id, SPD_MODE);
            break;
        case psi_mode:
            clear_err(motor->id, PSI_MODE);
            break;
        default:
            break;
    }
}

// 发送当前控制命令（根据模式）
void dm_motor_ctrl_send(motor_t *motor)
{
    switch (motor->ctrl.mode) {
        case mit_mode:
            mit_ctrl(motor, motor->id,
                     motor->ctrl.pos_set, motor->ctrl.vel_set,
                     motor->ctrl.kp_set, motor->ctrl.kd_set,
                     motor->ctrl.tor_set);
            break;
        case pos_mode:
            pos_ctrl(motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
            break;
        case spd_mode:
            spd_ctrl(motor->id, motor->ctrl.vel_set);
            break;
        case psi_mode:
            psi_ctrl(motor->id,
                     motor->ctrl.pos_set, motor->ctrl.vel_set,
                     motor->ctrl.cur_set);
            break;
        default:
            break;
    }
}

// 处理反馈数据（标准 DM 电机反馈格式）
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id    = rx_data[0] & 0x0F;
    motor->para.state = rx_data[0] >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    motor->para.pos = uint_to_float(motor->para.p_int,
                                    -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    motor->para.vel = uint_to_float(motor->para.v_int,
                                    -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    motor->para.tor = uint_to_float(motor->para.t_int,
                                    -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    motor->para.Tmos   = (float)rx_data[6];
    motor->para.Tcoil  = (float)rx_data[7];
}

// 读取电机内部寄存器（发送读命令）
void read_motor_data(uint16_t id, uint8_t rid)
{
    uint8_t can_id_l = id & 0xFF;
    uint8_t can_id_h = (id >> 8) & 0x07;
    uint8_t data[8] = {can_id_l, can_id_h, 0x33, rid, 0, 0, 0, 0};
    MyCAN_Transmit(0x7FF, 8, data);
}

// 读取电机控制反馈（标准 DM 协议）
void read_motor_ctrl_fbdata(uint16_t id)
{
    uint8_t can_id_l = id & 0xFF;
    uint8_t can_id_h = (id >> 8) & 0x07;
    uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
    MyCAN_Transmit(0x7FF, 4, data);
}

// 写电机寄存器
void write_motor_data(uint16_t id, uint8_t rid,
                      uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint8_t can_id_l = id & 0xFF;
    uint8_t can_id_h = (id >> 8) & 0x07;
    uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
    MyCAN_Transmit(0x7FF, 8, data);
}

// 保存写入的参数
void save_motor_data(uint16_t id, uint8_t rid)
{
    uint8_t can_id_l = id & 0xFF;
    uint8_t can_id_h = (id >> 8) & 0x07;
    uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
    MyCAN_Transmit(0x7FF, 4, data);
}
