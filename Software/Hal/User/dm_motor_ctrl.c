#include "dm_motor_ctrl.h"
#include <string.h>

motor_t motor[num];   // 电机数组定义

// 电机初始化
void dm_motor_init(void)
{
    // 清空所有电机结构体
    for (int i = 0; i < num; i++) {
        memset(&motor[i], 0, sizeof(motor_t));
    }

    // 示例：配置电机1（可根据实际修改）
    motor[Motor1].id = 0x01;
    motor[Motor1].mst_id = 0x00;
    motor[Motor1].tmp.read_flag = 1;          // 开始读取参数
    motor[Motor1].ctrl.mode = mit_mode;
    motor[Motor1].tmp.PMAX = 12.5f;            // 位置范围
    motor[Motor1].tmp.VMAX = 30.0f;            // 速度范围
    motor[Motor1].tmp.TMAX = 10.0f;            // 扭矩范围

    // 电机2 配置
    motor[Motor2].id = 0x02;
    motor[Motor2].mst_id = 0x00;
    motor[Motor2].tmp.read_flag = 1;
    motor[Motor2].ctrl.mode = mit_mode;
    motor[Motor2].tmp.PMAX = 12.5f;
    motor[Motor2].tmp.VMAX = 30.0f;
    motor[Motor2].tmp.TMAX = 10.0f;

    // 可根据需要继续添加其他电机
}

// 轮询读取电机所有参数（每次调用发送一条读命令，read_flag 自动递增）
void read_all_motor_data(motor_t *motor)
{
    switch (motor->tmp.read_flag) {
        case 1:  read_motor_data(motor->id, RID_UV_VALUE); break;
        case 2:  read_motor_data(motor->id, RID_KT_VALUE); break;
        case 3:  read_motor_data(motor->id, RID_OT_VALUE); break;
        case 4:  read_motor_data(motor->id, RID_OC_VALUE); break;
        case 5:  read_motor_data(motor->id, RID_ACC);      break;
        case 6:  read_motor_data(motor->id, RID_DEC);      break;
        case 7:  read_motor_data(motor->id, RID_MAX_SPD);  break;
        case 8:  read_motor_data(motor->id, RID_MST_ID);   break;
        case 9:  read_motor_data(motor->id, RID_ESC_ID);   break;
        case 10: read_motor_data(motor->id, RID_TIMEOUT);  break;
        case 11: read_motor_data(motor->id, RID_CMODE);    break;
        case 12: read_motor_data(motor->id, RID_DAMP);     break;
        case 13: read_motor_data(motor->id, RID_INERTIA);  break;
        case 14: read_motor_data(motor->id, RID_HW_VER);   break;
        case 15: read_motor_data(motor->id, RID_SW_VER);   break;
        case 16: read_motor_data(motor->id, RID_SN);       break;
        case 17: read_motor_data(motor->id, RID_NPP);      break;
        case 18: read_motor_data(motor->id, RID_RS);       break;
        case 19: read_motor_data(motor->id, RID_LS);       break;
        case 20: read_motor_data(motor->id, RID_FLUX);     break;
        case 21: read_motor_data(motor->id, RID_GR);       break;
        case 22: read_motor_data(motor->id, RID_PMAX);     break;
        case 23: read_motor_data(motor->id, RID_VMAX);     break;
        case 24: read_motor_data(motor->id, RID_TMAX);     break;
        case 25: read_motor_data(motor->id, RID_I_BW);     break;
        case 26: read_motor_data(motor->id, RID_KP_ASR);   break;
        case 27: read_motor_data(motor->id, RID_KI_ASR);   break;
        case 28: read_motor_data(motor->id, RID_KP_APR);   break;
        case 29: read_motor_data(motor->id, RID_KI_APR);   break;
        case 30: read_motor_data(motor->id, RID_OV_VALUE); break;
        case 31: read_motor_data(motor->id, RID_GREF);     break;
        case 32: read_motor_data(motor->id, RID_DETA);     break;
        case 33: read_motor_data(motor->id, RID_V_BW);     break;
        case 34: read_motor_data(motor->id, RID_IQ_CL);    break;
        case 35: read_motor_data(motor->id, RID_VL_CL);    break;
        case 36: read_motor_data(motor->id, RID_CAN_BR);   break;
        case 37: read_motor_data(motor->id, RID_SUB_VER);  break;
        case 38: read_motor_data(motor->id, RID_U_OFF);    break;
        case 39: read_motor_data(motor->id, RID_V_OFF);    break;
        case 40: read_motor_data(motor->id, RID_K1);       break;
        case 41: read_motor_data(motor->id, RID_K2);       break;
        case 42: read_motor_data(motor->id, RID_M_OFF);    break;
        case 43: read_motor_data(motor->id, RID_DIR);      break;
        case 44: read_motor_data(motor->id, RID_P_M);      break;
        case 45: read_motor_data(motor->id, RID_X_OUT);    break;
        default: break;
    }
}

// 处理参数读取返回的数据（在 CAN 接收回调中调用）
void receive_motor_data(motor_t *motor, uint8_t *data)
{
    if (motor->tmp.read_flag == 0) return;

    float_type_u y;
    if (data[2] == 0x33) {                     // 参数读取返回标识
        uint16_t rid_value = data[3];
        y.b_val[0] = data[4];
        y.b_val[1] = data[5];
        y.b_val[2] = data[6];
        y.b_val[3] = data[7];

        switch (rid_value) {
            case RID_UV_VALUE: motor->tmp.UV_Value = y.f_val; motor->tmp.read_flag =  2; break;
            case RID_KT_VALUE: motor->tmp.KT_Value = y.f_val; motor->tmp.read_flag =  3; break;
            case RID_OT_VALUE: motor->tmp.OT_Value = y.f_val; motor->tmp.read_flag =  4; break;
            case RID_OC_VALUE: motor->tmp.OC_Value = y.f_val; motor->tmp.read_flag =  5; break;
            case RID_ACC:      motor->tmp.ACC      = y.f_val; motor->tmp.read_flag =  6; break;
            case RID_DEC:      motor->tmp.DEC      = y.f_val; motor->tmp.read_flag =  7; break;
            case RID_MAX_SPD:  motor->tmp.MAX_SPD  = y.f_val; motor->tmp.read_flag =  8; break;
            case RID_MST_ID:   motor->tmp.MST_ID   = y.u_val; motor->tmp.read_flag =  9; break;
            case RID_ESC_ID:   motor->tmp.ESC_ID   = y.u_val; motor->tmp.read_flag = 10; break;
            case RID_TIMEOUT:  motor->tmp.TIMEOUT  = y.u_val; motor->tmp.read_flag = 11; break;
            case RID_CMODE:    motor->tmp.cmode    = y.u_val; motor->tmp.read_flag = 12; break;
            case RID_DAMP:     motor->tmp.Damp     = y.f_val; motor->tmp.read_flag = 13; break;
            case RID_INERTIA:  motor->tmp.Inertia  = y.f_val; motor->tmp.read_flag = 14; break;
            case RID_HW_VER:   motor->tmp.hw_ver   = y.u_val; motor->tmp.read_flag = 15; break;
            case RID_SW_VER:   motor->tmp.sw_ver   = y.u_val; motor->tmp.read_flag = 16; break;
            case RID_SN:       motor->tmp.SN       = y.u_val; motor->tmp.read_flag = 17; break;
            case RID_NPP:      motor->tmp.NPP      = y.u_val; motor->tmp.read_flag = 18; break;
            case RID_RS:       motor->tmp.Rs       = y.f_val; motor->tmp.read_flag = 19; break;
            case RID_LS:       motor->tmp.Ls       = y.f_val; motor->tmp.read_flag = 20; break;
            case RID_FLUX:     motor->tmp.Flux     = y.f_val; motor->tmp.read_flag = 21; break;
            case RID_GR:       motor->tmp.Gr       = y.f_val; motor->tmp.read_flag = 22; break;
            case RID_PMAX:     motor->tmp.PMAX     = y.f_val; motor->tmp.read_flag = 23; break;
            case RID_VMAX:     motor->tmp.VMAX     = y.f_val; motor->tmp.read_flag = 24; break;
            case RID_TMAX:     motor->tmp.TMAX     = y.f_val; motor->tmp.read_flag = 25; break;
            case RID_I_BW:     motor->tmp.I_BW     = y.f_val; motor->tmp.read_flag = 26; break;
            case RID_KP_ASR:   motor->tmp.KP_ASR   = y.f_val; motor->tmp.read_flag = 27; break;
            case RID_KI_ASR:   motor->tmp.KI_ASR   = y.f_val; motor->tmp.read_flag = 28; break;
            case RID_KP_APR:   motor->tmp.KP_APR   = y.f_val; motor->tmp.read_flag = 29; break;
            case RID_KI_APR:   motor->tmp.KI_APR   = y.f_val; motor->tmp.read_flag = 30; break;
            case RID_OV_VALUE: motor->tmp.OV_Value = y.f_val; motor->tmp.read_flag = 31; break;
            case RID_GREF:     motor->tmp.GREF     = y.f_val; motor->tmp.read_flag = 32; break;
            case RID_DETA:     motor->tmp.Deta     = y.f_val; motor->tmp.read_flag = 33; break;
            case RID_V_BW:     motor->tmp.V_BW     = y.f_val; motor->tmp.read_flag = 34; break;
            case RID_IQ_CL:    motor->tmp.IQ_cl    = y.f_val; motor->tmp.read_flag = 35; break;
            case RID_VL_CL:    motor->tmp.VL_cl    = y.f_val; motor->tmp.read_flag = 36; break;
            case RID_CAN_BR:   motor->tmp.can_br   = y.u_val; motor->tmp.read_flag = 37; break;
            case RID_SUB_VER:  motor->tmp.sub_ver  = y.u_val; motor->tmp.read_flag = 38; break;
            case RID_U_OFF:    motor->tmp.u_off    = y.f_val; motor->tmp.read_flag = 39; break;
            case RID_V_OFF:    motor->tmp.v_off    = y.f_val; motor->tmp.read_flag = 40; break;
            case RID_K1:       motor->tmp.k1       = y.f_val; motor->tmp.read_flag = 41; break;
            case RID_K2:       motor->tmp.k2       = y.f_val; motor->tmp.read_flag = 42; break;
            case RID_M_OFF:    motor->tmp.m_off    = y.f_val; motor->tmp.read_flag = 43; break;
            case RID_DIR:      motor->tmp.dir      = y.f_val; motor->tmp.read_flag = 44; break;
            case RID_P_M:      motor->tmp.p_m      = y.f_val; motor->tmp.read_flag = 45; break;
            case RID_X_OUT:    motor->tmp.x_out    = y.f_val; motor->tmp.read_flag =  0; break;
            default: break;
        }
    }
}

// CAN 接收回调处理（需在 HAL_CAN_RxFifo0MsgPendingCallback 中调用）
void Motor_CAN_RxHandler(void)
{
    uint32_t rec_id;
    uint8_t rx_data[8];
    uint8_t len;

    // 使用 MyCAN_Receive 读取数据
    MyCAN_Receive(&rec_id, &len, rx_data);

    // 根据 ID 分发到对应的电机反馈处理函数
    switch (rec_id) {
        case 0x201:                 // DM3519 反馈 ID 示例
            dm3519_fbdata(&motor[Motor1], rx_data);
            break;
        // 可添加更多 ID 处理
        default:
            // 如果是参数读取返回（ID 一般为 0x7FF? 实际需确认）
            // 此处简单轮询所有电机，根据实际协议可优化
            for (int i = 0; i < num; i++) {
                if (motor[i].id != 0 && motor[i].tmp.read_flag != 0) {
                    receive_motor_data(&motor[i], rx_data);
                }
            }
            break;
    }
}
