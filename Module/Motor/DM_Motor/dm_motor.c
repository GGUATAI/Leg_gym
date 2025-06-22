#include "dm_motor.h"

float Hex_To_Float(uint32_t *Byte, int num) //十六进制到浮点数
{
	return *((float*) Byte);
}

uint32_t Float_To_Hex(float HEX) //浮点数到十六进制转换
{
	return *(uint32_t*) &HEX;
}

/**
 ************************************************************************
 * @brief:      	float_to_uint: 浮点数转换为无符号整数函数
 * @param[in]:   x_float:	待转换的浮点数
 * @param[in]:   x_min:		范围最小值
 * @param[in]:   x_max:		范围最大值
 * @param[in]:   bits: 		目标无符号整数的位数
 * @retval:     	无符号整数结果
 * @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
 ************************************************************************
 **/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float - offset) * ((float) ((1 << bits) - 1)) / span);
}

/**
 ************************************************************************
 * @brief:      	uint_to_float: 无符号整数转换为浮点数函数
 * @param[in]:   x_int: 待转换的无符号整数
 * @param[in]:   x_min: 范围最小值
 * @param[in]:   x_max: 范围最大值
 * @param[in]:   bits:  无符号整数的位数
 * @retval:     	浮点数结果
 * @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
 ************************************************************************
 **/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode)
{
    motor->mode = mode;
    motor->para.id = id;
}

void wheel_motor_init(Wheel_Motor_t *motor, uint16_t id, uint16_t mode)
{
    motor->mode = mode;
    motor->para.id = id;
}

/**
************************************************************************
* @brief:      	DM8009p_Decode: ��ȡDM4310����������ݺ���
* @param[in]:   motor:    ָ��motor_t�ṹ��ָ�룬������������Ϣ�ͷ�������
* @param[in]:   rx_data:  ָ������������ݵ�����ָ��
* @param[in]:   data_len: ���ݳ���
* @retval:     	void
* @details:    	�ӽ��յ�����������ȡDM4310����ķ�����Ϣ���������ID��
*               ״̬��λ�á��ٶȡ�Ť������¶Ȳ������Ĵ������ݵ�
************************************************************************
**/
void DM8009p_Decode(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if (data_len == FDCAN_DLC_BYTES_8)
    { // ���ص�������8���ֽ�
        motor->para.id = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
        motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
        motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
        motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); // (-10.0,10.0)
        motor->para.Tmos = (float)(rx_data[6]);
        motor->para.Tcoil = (float)(rx_data[7]);
    }
}

void DM4310_Decode(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{
	if (data_len == FDCAN_DLC_BYTES_8)
	{ // ���ص�������8���ֽ�
		motor->para.id = (rx_data[0]) & 0x0F;
		motor->para.state = (rx_data[0]) >> 4;
		motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
		motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
		motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
		motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
		motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
		motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); // (-10.0,10.0)
		motor->para.tor = motor->para.tor * 0.8f + motor->para.last_tor * 0.2f;
		motor->para.Tmos = (float)(rx_data[6]);
		motor->para.Tcoil = (float)(rx_data[7]);
		motor->para.last_tor = motor->para.tor;
	}
}

void DJI3508_Decode(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if (data_len == FDCAN_DLC_BYTES_8)
    { // ���ص�������8���ֽ�
        motor->para.pos = (uint16_t)(rx_data[0] << 8) + rx_data[1];
        motor->para.vel = (float)((int16_t)(rx_data[2] << 8) + rx_data[3]); // rpm
        motor->para.vel /= 60;                                              // rps
        motor->para.vel /= REDUCTION_RATIO;                                 // ��ȥ���ٱ�
        motor->para.tor = (float)((int16_t)(rx_data[4] << 8) + rx_data[5]);
		motor->para.tor = motor->para.tor * 0.8f + motor->para.last_tor * 0.2f;
        motor->para.Tmos = (float)(uint8_t)rx_data[6];
        // 268/17
    }
}

void DM_Motor_ENABLE(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	DM_Motor_DISABLE: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ���õ�ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͽ����ض�ģʽ������
************************************************************************
**/
void DM_Motor_DISABLE(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    canx_send_data(hcan, id, data, 8);
}

void DM_Motor_Set_Zeropoint(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;

    canx_send_data(hcan, id, data, 8);
}

//MIT 模式是为了兼容原版 MIT 模式所设计，可以在实现无缝切换的同时，能够灵活设
//定控制范围（P_MAX,V_MAX，T_MAX），电调将接收到的 CAN 数据转化成控制变量进行
//运算得到扭矩值作为电流环的电流给定，电流环根据其调节规律最终达到给定的扭矩电流。
//根据 MIT 模式可以衍生出多种控制模式，如 kp=0,kd 不为 0 时，给定 v_des 即可实现匀
//速转动;kp=0,kd=0，给定 t_ff 即可实现给定扭矩输出。
//注意：对位置进行控制时，kd 不能赋 0，否则会造成电机震荡，甚至失控。
//1. 当 kp=0，kd≠0 时，给定 v_des 即可实现匀速转动。匀速转动过程中存在静差，另外 kd 不宜过大， kd 过大时会引起震荡。
//2. 当 kp=0，kd=0 时，给定 t_ff 即可实现给定扭矩输出。在该情况下，电机会持续输出一个恒定力矩。但是当电机空转或负载较小时，如果给定 t_ff 较大，电机会持续加速，直到加速到最大速度，这时也仍然达不到目标力矩 t_ff。
//3. 当 kp≠0，kd=0 时，会引起震荡。即对位置进行控制时，kd 不能赋0，否则会造成电机震荡，甚至失控。
//4. 当 kp≠0，kd≠0 时，有多种情况，这里下面简单介绍两种情况。
//
//- 当期望位置 p_des 为常量，期望速度 v_des 为0时，可实现定点控制，在这个过程中，实际位置趋近于p_des，实际速度 dθm 趋近于0。
//- 当p_des是随时间变化的连续可导函数时，同时 v_des 是 p_des 的导数，可实现位置跟踪和速度跟踪，即按照期望速度旋转期望角度。
/**
 ************************************************************************
 * @brief:      	mit_ctrl: MIT模式下的电机控制函数
 * @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id:	电机ID，指定目标电机
 * @param[in]:   pos:			位置给定值
 * @param[in]:   vel:			速度给定值
 * @param[in]:   kp:				位置比例系数
 * @param[in]:   kd:				位置微分系数
 * @param[in]:   torq:			转矩给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
 ************************************************************************
 **/
void DM_MIT_Control(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    canx_send_data(hcan, id, data, 8);
}

//位置串级模式是采用三环串联控制的模式，位置环作为最外环，其输出作为速度环的给
//定，而速度环的输出作为内环电流环的给定，用以控制实际的电流输出
//p_des 为控制的目标位置，v_des 是用来限定运动过程中的最大绝对速度值。
//位置串级模式如使用调试助手推荐的控制参数控制，可以达到较好的控制精度，控制过
//程相对柔顺，但响应时间相对较长。可配置的相关参数除 v_des 外，另有加/减速度进行设
//定，如控制过程中产生额外的震荡可提高加/减速度。
//注意：p_des，v_des 单位分别为 rad 和 rad/s，数据类型为 float，阻尼因子必须设置为
//非 0 的正数，可参考速度模式的注意事项。
/**
 ************************************************************************
 * @brief:      	DM_Position_Control: 位置速度控制函数
 * @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id:	电机ID，指定目标电机
 * @param[in]:   vel:			速度给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送位置速度控制命令
 ************************************************************************
 **/
void DM_Position_Control(hcan_t *hcan, uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id = motor_id + POS_MODE;
    pbuf = (uint8_t *)&pos;
    vbuf = (uint8_t *)&vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    canx_send_data(hcan, id, data, 8);
}

//速度模式能让电机稳定运行在设定的速度
//注意：v_des 单位为 rad/s，数据类型为 float，如需使用调试助手自动计算参数，则需要
//设置阻尼因子为非 0 正数，通常情况下取值在 2.0~10.0，过小的阻尼因子会带来速度的震荡
//以及较大的过冲，过大的阻尼因子则会带来较长的上升时间，推荐的设定值为 4.0。
/**
 ************************************************************************
 * @brief:      	DM_Speed_Control: 速度控制函数
 * @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id: 电机ID，指定目标电机
 * @param[in]:   vel: 			速度给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送速度控制命令
 ************************************************************************
 **/
void DM_Speed_Control(hcan_t *hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t data[4];

    id = motor_id + SPEED_MODE;
    vbuf = (uint8_t *)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    canx_send_data(hcan, id, data, 4);
}

void DJI3508_Control(hcan_t *hcan, float torq1, float torq2)
{
    static int16_t i = 0;
    static uint8_t data[8] = {0};

    torq1 = torq1 / 0.206f;           // �ļ����������س���  0.3 *��268/17��/��3591/157�� �ֲ��ϵ����س��� * �ּ�������ٱ� / ԭ��������ٱ�
    torq1 = torq1 / 20.0f * 16384.0f; // ���ص���ת��
    if (torq1 > 16384.0f)
    {
        torq1 = 16384.0f;
    }
    else if (torq1 < -16384.0f)
    {
        torq1 = -16384.0f;
    }
    i = (int16_t)torq1;

    data[0] = (i >> 8);
    data[1] = i;

    torq2 = torq2 / 0.206f;
    torq2 = torq2 / 20.0f * 16384.0f;

    if (torq2 > 16384.0f)
    {
        torq2 = 16384.0f;
    }
    else if (torq2 < -16384.0f)
    {
        torq2 = -16384.0f;
    }
    i = (int16_t)torq2;
    data[2] = (i >> 8);
    data[3] = i;

    canx_send_data(hcan, 0x200, data, 8);
}
