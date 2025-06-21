/**
 ******************************************************************************
 * @file	controller.h
 * @author  Wang Hongxi
 * @author  Zhang Hongyu (fuzzy pid)
 * @author  GUATAI (smc)
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "stm32h7xx.h"
#include "main.h"
#include "cmsis_os.h"

#ifndef user_malloc
#ifdef CMSIS_OS_H_
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/******************************** FUZZY PID **********************************/
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3

typedef struct
{
  float KpFuzzy;
  float KiFuzzy;
  float KdFuzzy;

  float (*FuzzyRuleKp)[7];
  float (*FuzzyRuleKi)[7];
  float (*FuzzyRuleKd)[7];

  float KpRatio;
  float KiRatio;
  float KdRatio;

  float eStep;
  float ecStep;

  float e;
  float ec;
  float eLast;

  uint32_t DWT_CNT;
  float dt;
} __attribute__((__packed__)) FuzzyRule_t;

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule,
		     float (*fuzzyRuleKp)[7],
		     float (*fuzzyRuleKi)[7],
		     float (*fuzzyRuleKd)[7],
		     float kpRatio,
		     float kiRatio,
		     float kdRatio,
		     float eStep,
		     float ecStep);
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);

/******************************* PID CONTROL *********************************/
// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum
{
  NONE = 0X00,                        //0000 0000
  Integral_Limit = 0x01,              //0000 0001
  Derivative_On_Measurement = 0x02,   //0000 0010
  Trapezoid_Intergral = 0x04,         //0000 0100
  Proportional_On_Measurement = 0x08, //0000 1000
  OutputFilter = 0x10,                //0001 0000
  ChangingIntegrationRate = 0x20,     //0010 0000
  DerivativeFilter = 0x40,            //0100 0000
  ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum
{
  PID_ERROR_NONE = 0x00U,
  PID_MOTOR_BLOCKED_ERROR = 0x01U
} analog_pid_error_e;

typedef struct
{
  uint64_t error_count;
  analog_pid_error_e error_type;
} analog_pid_errorhandler_t;

/* PID结构体 */
typedef struct analog_pid
{
  //---------------------------------- init config block
  float Ref;
  // config parameter
  float Kp;
  float Ki;
  float Kd;

  // improve parameter
  PID_Improvement_e Improve;
  float MaxOut;
  float DeadBand;
  float IntegralLimit;     // 积分限幅
  float CoefA;             // 变速积分 For Changing Integral
  float CoefB;     // 变速积分 ITerm = Err*((A-user_abs(err)+B)/A)  when B<|err|<A+B
  float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
  float Derivative_LPF_RC; // 微分滤波器系数
  float ControlPeriod;
  //-----------------------------------
  // for calculating
  float Measure;
  float Last_Measure;
  float Err;
  float Last_Err;
  float Last_ITerm;

  float Pout;
  float Iout;
  float Dout;
  float ITerm;

  float Output;
  float Last_Output;
  float Last_Dout;

  uint32_t DWT_CNT;
  float dt;

  analog_pid_errorhandler_t ERRORHandler;
  uint16_t OLS_Order;
  Ordinary_Least_Squares_t OLS;

  FuzzyRule_t *FuzzyRule;

  void (*User_Func1_f)(struct analog_pid *pid);
  void (*User_Func2_f)(struct analog_pid *pid);
} analog_pid_t;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
  // basic parameter
  float Kp;
  float Ki;
  float Kd;
  float MaxOut;   // 输出限幅
  float DeadBand; // 死区

  // improve parameter
  PID_Improvement_e Improve;
  float IntegralLimit; // 积分限幅
  float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
  float CoefB;         // ITerm = Err*((A-user_abs(err)+B)/A)  when B<|err|<A+B
  float Output_LPF_RC; // RC = 1/omegac
  float Derivative_LPF_RC;
} analog_pid_init_config_t;

/**
 * @brief 初始化PID实例
 * @todo 待修改为统一的PIDRegister风格
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void Analog_PID_Init(analog_pid_t *pid, analog_pid_init_config_t *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float Analog_PID_Calculate(analog_pid_t *pid, float measure, float ref);
/******************************** SMC PARAMS **********************************/
/* 用于SMC初始化的结构体*/
typedef struct
{
  float lambda;		// 滑模面系数 (λ > 0)
  float eta;        	// 切换增益 (η > 扰动上界)
  float phi;        	// 边界层厚度 (φ > 0)

  float output_limit;
} SMC_init_config_t;

// 滑模控制器参数
typedef struct
{
  float lambda;		// 滑模面系数 (λ > 0)
  float eta;        	// 切换增益 (η > 扰动上界)
  float phi;        	// 边界层厚度 (φ > 0)

  float s;		// 滑模面: s = λ*e + e_dot
  float sat_s;	    	// 饱和函数代替符号函数

  float err;
  float last_err;
  float derivative_err;

  uint32_t DWT_CNT;
  float dt;

  float output;
  float output_limit;
} SMC_t;

// 复合控制器参数
typedef struct
{
  SMC_t smc;
  analog_pid_t pid;
  float k_smc;       // 滑模权重 (0.0~1.0)
  float k_pid;       // PID权重 (0.0~1.0)
} SMC_PID_params_t;

/*************************** FEEDFORWARD CONTROL *****************************/
typedef struct
{
  float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

  float Ref;
  float Last_Ref;

  float DeadBand;

  uint32_t DWT_CNT;
  float dt;

  float LPF_RC; // RC = 1/omegac

  float Ref_dot;
  float Ref_ddot;
  float Last_Ref_dot;

  uint16_t Ref_dot_OLS_Order;
  Ordinary_Least_Squares_t Ref_dot_OLS;
  uint16_t Ref_ddot_OLS_Order;
  Ordinary_Least_Squares_t Ref_ddot_OLS;

  float Output;
  float MaxOut;

} __attribute__((__packed__)) Feedforward_t;

void Feedforward_Init(Feedforward_t *ffc,
		      float max_out,
		      float *c,
		      float lpf_rc,
		      uint16_t ref_dot_ols_order,
		      uint16_t ref_ddot_ols_order);

float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/************************* LINEAR DISTURBANCE OBSERVER *************************/
typedef struct
{
  float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

  float Measure;
  float Last_Measure;

  float u; // system input

  float DeadBand;

  uint32_t DWT_CNT;
  float dt;

  float LPF_RC; // RC = 1/omegac

  float Measure_dot;
  float Measure_ddot;
  float Last_Measure_dot;

  uint16_t Measure_dot_OLS_Order;
  Ordinary_Least_Squares_t Measure_dot_OLS;
  uint16_t Measure_ddot_OLS_Order;
  Ordinary_Least_Squares_t Measure_ddot_OLS;

  float Disturbance;
  float Output;
  float Last_Disturbance;
  float Max_Disturbance;
} __attribute__((__packed__)) LDOB_t;

void LDOB_Init(LDOB_t *ldob,
	       float max_d,
	       float deadband,
	       float *c,
	       float lpf_rc,
	       uint16_t measure_dot_ols_order,
	       uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);

/*************************** Tracking Differentiator ***************************/
typedef struct
{
  float Input;

  float h0;
  float r;

  float x;
  float dx;
  float ddx;

  float last_dx;
  float last_ddx;

  uint32_t DWT_CNT;
  float dt;
} __attribute__((__packed__)) TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);

#endif
