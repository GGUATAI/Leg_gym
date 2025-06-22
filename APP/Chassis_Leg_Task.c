#include "Chassis_Leg_Task.h"

uint8_t right_off_ground_flag = 0;
uint8_t left_off_ground_flag = 0;

const uint8_t ps_flag = 0;

const float MG = 14.43f;// 质量*重力加速度*高度
float roll_f0_forward = 0.0f;

float offset_pitch_set = -0.06f;
float offset_theta_set = 0.0f;
float offset_v_set = 0.0f;
float offset_x_filter = 0.5f;

float Poly_Coefficient_Speed[12][4] = {
	
{-201.4532,253.7486,-140.9255,0.1224},
{4.6417,-2.7518,-9.3417,0.3022},
{-34.4840,37.6960,-14.9411,-0.2637},
{-54.0011,59.6126,-24.7513,-0.4970},
{-22.1627,40.1278,-27.7903,9.2411},
{-2.0437,5.0834,-4.2171,1.6894},
{229.5085,-209.5229,51.3689,9.0693},
{22.7869,-25.3141,10.0099,0.2495},
{-13.9107,23.7908,-15.7232,4.8805},
{-21.8553,37.3141,-24.7803,7.9257},
{168.1183,-184.4427,73.6655,0.3219},
{29.4190,-32.7661,13.4186,-0.3060}
	
//{-205.2145,254.9769,-143.7492,-1.0680},
//{3.9353,-2.5600,-10.6433,0.2577},
//{-30.3472,32.7514,-12.7706,-0.8291},
//{-51.7666,56.5124,-23.3142,-1.5576},
//{-42.4655,60.7958,-35.1480,10.2959},
//{-5.8128,8.9622,-5.6193,1.9050},
//{162.7712,-147.6195,35.3632,8.0901},
//{18.0406,-19.9859,7.9830,0.2995},
//{-18.9462,26.7268,-15.1171,4.1834},
//{-33.0322,46.3861,-26.2458,7.4481},
//{139.5574,-151.0851,59.3405,2.7558},
//{24.9025,-27.3194,10.9809,0.1325}

};

float Poly_Coefficient_Position[12][4] = {

		{ -333.5526, 392.3060,  -208.4730, -2.9882 },
		{ -0.6832,   1.0135,    -15.8599,  0.3691 },
		{ -96.2153,  99.1720,   -36.0979,  -4.6801 },
		{ -73.7459,  77.4364,   -31.8566,  -3.8835 },
		{ -135.4099, 164.5452,  -78.1525,  17.8723 },
		{ -10.5871,  13.6609,   -7.0274,   1.9369 },
		{ 133.0067,  -105.3362, 13.6453,   12.2323 },
		{ 19.9475,   -20.6067,  7.2021,    0.5390 },
		{ -113.3571, 136.1625,  -63.4090,  13.6812 },
		{ -87.9016,  105.4369,  -49.2619,  11.0095 },
		{ 234.1354,  -242.5060, 89.1033,   9.4299 },
		{ 24.4997,   -25.8426,  9.7980,    0.4786 }

};

vmc_leg_t chassis_right_leg;
vmc_leg_t chassis_left_leg;
chassis_leg_t chassis_move;

digital_PID_t leg_l_length_pid; // 左腿的腿长pd
digital_PID_t leg_r_length_pid; // 右腿的腿长pd

digital_PID_t leg_roll_position_pid; // 横滚角补偿pd
digital_PID_t leg_roll_speed_pid; // 横滚角补偿pd

digital_PID_t leg_Tp_pid;   // 防劈叉补偿pd
digital_PID_t leg_turn_pid; // 转向pd

float leg_init_set = 0.162f;
float leg_max_set = 0.333f;
float leg_min_set = 0.139f;

// 保持腿长
float leg_length_pid_kp = 100.0f;
float leg_length_pid_ki = 0.0f; // 不积分
float leg_length_pid_kd = 18.0f;
float leg_length_pid_max_out = 12.0f; // 90牛

void Leg_Pensation_Init( pid_t *Tp, pid_t *turn )
{ // 补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿

	// 保持机体水平
	float leg_roll_position_kp = 100.0f;
	float leg_roll_position_ki = 0.0f; // 不用积分项
	float leg_roll_position_kd = 5.0f;
	float leg_roll_position_max_out = 12.0f;

	// 保持机体水平
	float leg_roll_speed_kp = 1.0f;
	float leg_roll_speed_ki = 0.005f; // 不用积分项
	float leg_roll_speed_kd = 0.0f;
	float leg_roll_speed_max_out = 5.0f;

	leg_roll_position_pid.Kp = leg_roll_position_kp;
	leg_roll_position_pid.Ki = leg_roll_position_ki;
	leg_roll_position_pid.Kd = leg_roll_position_kd;
	leg_roll_position_pid.Output_Max = leg_roll_position_max_out;
	leg_roll_position_pid.dead_band = 0.001f;
	leg_roll_position_pid.improve = PID_IMPROVE_NONE_MOTION;

	leg_roll_speed_pid.Kp = leg_roll_speed_kp;
	leg_roll_speed_pid.Ki = leg_roll_speed_ki;
	leg_roll_speed_pid.Kd = leg_roll_speed_kd;
	leg_roll_speed_pid.Output_Max = leg_roll_speed_max_out;
	leg_roll_speed_pid.improve = PID_IMPROVE_NONE_MOTION;

	// 防劈腿
	float leg_Tp_pid_kp = 0.0f;
	float leg_Tp_pid_ki = 0.0f; // 不用积分项
	float leg_Tp_pid_kd = 0.0f;
	float leg_Tp_pid_max_out = 0.0f;

	leg_Tp_pid.Kp = leg_Tp_pid_kp;
	leg_Tp_pid.Ki = leg_Tp_pid_ki;
	leg_Tp_pid.Kd = leg_Tp_pid_kd;
	leg_Tp_pid.Output_Max = leg_Tp_pid_max_out;
	leg_Tp_pid.dead_band = 0.001f;
	leg_Tp_pid.improve = PID_IMPROVE_NONE_MOTION;

// 转向环yaw
	float leg_turn_pid_kp = 15.0f;
	float leg_turn_pid_ki = 0.0f; // 不用积分项
	float leg_turn_pid_kd = 1.0f;
	float leg_turn_pid_max_out = 4.2f; // 轮毂电机的额定扭矩

	leg_turn_pid.Kp = leg_turn_pid_kp;
	leg_turn_pid.Ki = leg_turn_pid_ki;
	leg_turn_pid.Kd = leg_turn_pid_kd;
	leg_turn_pid.Output_Max = leg_turn_pid_max_out;
	leg_turn_pid.dead_band = 0.001f;
	leg_turn_pid.improve = PID_IMPROVE_NONE_MOTION;

}

void Leg_Saturate( float *in, float min, float max )
{
	if( *in < min )
	{
		*in = min;
	}
	else if( *in > max )
	{
		*in = max;
	}
}

void Chassis_Roll_Force_Calc( float roll_target, float roll_value )
{
	;
}

//{-316.7252,383.2687,-202.0971,-1.7453},
//{1.9488,0.3395,-13.9077,0.3617},
//{-21.7250,23.0883,-8.8087,-0.6620},
//{-62.8151,67.1208,-26.4302,-1.9771},
//{-35.3014,48.3816,-26.8564,7.7085},
//{-4.2514,6.2239,-3.7054,1.2390},
//{205.5243,-181.1215,39.7316,11.3074},
//{22.2532,-24.1459,9.2703,0.3689},
//{-16.4702,21.7686,-11.4827,2.9483},
//{-47.2340,62.5341,-33.1101,8.6401},
//{105.5088,-113.0716,43.8539,1.8506},
//{16.6085,-18.0150,7.1389,0.1005}

//{-320.4402,388.4106,-209.3709,-2.0873},
//{2.1228,-0.0285,-15.5652,0.3910},
//{-20.4726,21.6563,-8.2108,-0.7589},
//{-68.8016,73.1768,-28.6239,-2.6216},
//{-51.0298,67.2982,-35.7646,9.7992},
//{-5.4818,7.6810,-4.3785,1.4139},
//{197.5511,-174.1444,38.1478,11.8032},
//{23.9243,-25.9328,10.0469,0.4240},
//{-17.5551,22.5777,-11.5449,2.8806},
//{-58.7970,75.6643,-38.7851,9.8189},
//{130.0814,-138.5719,53.2835,3.1434},
//{18.2655,-19.7283,7.7741,0.1689}

//{-338.9380,396.0585,-194.0748,-1.1227},
//{-2.0368,4.1244,-12.5852,0.3289},
//{-72.0808,74.4939,-27.0454,-1.3722},
//{-70.3781,73.3667,-28.2284,-1.5967},
//{-64.2031,85.5107,-44.3260,10.8181},
//{-3.7353,5.5905,-3.2534,0.9909},
//{241.5912,-165.9339,1.8549,24.1291},
//{29.8223,-28.5318,8.1248,1.3078},
//{-104.6354,134.8189,-66.7094,14.6896},
//{-116.2439,146.3851,-70.9682,15.5813},
//{323.5959,-336.6639,123.7975,2.2072},
//{28.1422,-29.9306,11.4037,-0.1818}

//	{-242.9620, 299.3549, -162.1772, -2.3617},
//	{0.4096, 0.8223, -10.6855, 0.0679},
//	{-30.0173, 32.5363, -12.5449, -3.1905},
//	{-24.8376, 27.7830, -12.3033, -3.6670},
//	{-75.2240, 103.2555, -57.4417, 16.9364},
//	{-2.4809, 3.9657, -2.5241, 1.2559},
//	{269.8904, -233.2223, 51.4253, 17.4715},
//	{20.6657, -20.2606, 5.6966, 1.1458},
//	{-56.8991, 80.9259, -44.2387, 10.6338},
//	{-69.6440, 93.8583, -49.0809, 11.5144},
//	{337.7607, -367.9950, 145.8889, 4.6029},
//	{22.7427, -25.9250, 11.0196, -0.2817}

// TODO:跳跃：分阶段，缩腿，给支持力
//	chassis_left_leg.YD = chassis_left_leg.l4 * arm_sin_f32( chassis_left_leg.phi4 );
//	chassis_left_leg.YB = chassis_left_leg.l1 * arm_sin_f32( chassis_left_leg.phi1 );
//	chassis_left_leg.XD = chassis_left_leg.l5 + chassis_left_leg.l4 * arm_cos_f32( chassis_left_leg.phi4 );
//	chassis_left_leg.XB = chassis_left_leg.l1 * arm_cos_f32( chassis_left_leg.phi1 );

//	chassis_left_leg.l_BD = sqrt(
//			( chassis_left_leg.XD - chassis_left_leg.XB ) * ( chassis_left_leg.XD - chassis_left_leg.XB ) +
//			( chassis_left_leg.YD - chassis_left_leg.YB ) * ( chassis_left_leg.YD - chassis_left_leg.YB ) );

//	chassis_left_leg.A0 = 2 * chassis_left_leg.l2 * ( chassis_left_leg.XD - chassis_left_leg.XB );
//	chassis_left_leg.B0 = 2 * chassis_left_leg.l2 * ( chassis_left_leg.YD - chassis_left_leg.YB );
//	chassis_left_leg.C0 = chassis_left_leg.l2 * chassis_left_leg.l2 + chassis_left_leg.l_BD * chassis_left_leg.l_BD -
//						  chassis_left_leg.l3 * chassis_left_leg.l3;
//	chassis_left_leg.phi2 = 2 * atan2f( ( chassis_left_leg.B0 + sqrt( chassis_left_leg.A0 * chassis_left_leg.A0 +
//																	  chassis_left_leg.B0 * chassis_left_leg.B0 -
//																	  chassis_left_leg.C0 * chassis_left_leg.C0 ) ),
//										chassis_left_leg.A0 + chassis_left_leg.C0 );
//	chassis_left_leg.phi3 = atan2f(
//			chassis_left_leg.YB - chassis_left_leg.YD + chassis_left_leg.l2 * arm_sin_f32( chassis_left_leg.phi2 ),
//			chassis_left_leg.XB - chassis_left_leg.XD + chassis_left_leg.l2 * arm_cos_f32( chassis_left_leg.phi2 ) );

//	chassis_left_leg.XC = chassis_left_leg.l1 * arm_cos_f32( chassis_left_leg.phi1 ) +
//						  chassis_left_leg.l2 * arm_cos_f32( chassis_left_leg.phi2 );
//	chassis_left_leg.YC = chassis_left_leg.l1 * arm_sin_f32( chassis_left_leg.phi1 ) +
//						  chassis_left_leg.l2 * arm_sin_f32( chassis_left_leg.phi2 );

///////////////////////////////////////更改重心///////////////////////////////////////
//	chassis_left_leg.L0 = sqrt( ( chassis_left_leg.XC - chassis_left_leg.l5 / 2.0f ) *
//								( chassis_left_leg.XC - chassis_left_leg.l5 / 2.0f ) +
//								chassis_left_leg.YC * chassis_left_leg.YC );

//	chassis_left_leg.phi0 = atan2f( chassis_left_leg.YC, ( chassis_left_leg.XC - chassis_left_leg.l5 / 2.0f ) );
//	/////////////////////////////////////////////////////////////////////////////

//	chassis_left_leg.alpha = PI / 2.0f - chassis_left_leg.phi0;

//	if( dt == 0 )
//	{
//		chassis_left_leg.last_phi0 = chassis_left_leg.phi0;
//	}

//	dt = DWT_GetDeltaT( &left_dwt );

//	chassis_left_leg.d_phi0 = ( chassis_left_leg.phi0 - chassis_left_leg.last_phi0 ) / dt;

//	chassis_left_leg.d_alpha = 0.0f - chassis_left_leg.d_phi0;

//	chassis_left_leg.theta = PI / 2.0f - chassis_left_leg.pitch - chassis_left_leg.phi0;
//	chassis_left_leg.d_theta = -chassis_left_leg.d_pitch - chassis_left_leg.d_phi0;

//	chassis_left_leg.last_phi0 = chassis_left_leg.phi0;

//	chassis_left_leg.d_L0 = ( chassis_left_leg.L0 - chassis_left_leg.last_L0 ) / dt;
//	chassis_left_leg.dd_L0 = ( chassis_left_leg.d_L0 - chassis_left_leg.last_d_L0 ) / dt;

//	chassis_left_leg.last_L0 = chassis_left_leg.L0;
//	chassis_left_leg.last_d_L0 = chassis_left_leg.d_L0;

//	chassis_left_leg.dd_theta = ( chassis_left_leg.d_theta - chassis_left_leg.last_d_theta ) / dt;
//	chassis_left_leg.last_d_theta = chassis_left_leg.d_theta;
