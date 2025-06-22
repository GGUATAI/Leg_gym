/**
 ******************************************************************************
 * @file    controller.c
 * @author  Wang Hongxi
 * @author  Zhang Hongyu (fuzzy pid)
 * @author  GUATAI (smc)
 * @version V1.1.3
 * @date    2021/7/3
 * @brief   DWT定时器用于计算控制周期，OLS用于提取信号微分
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "controller.h"
/******************************** SMC PARAMS **********************************/
//#### **1. 阶跃响应法**
//- **操作**：快速改变目标值（如从0到100），录制实际位置曲线。
//- **分析**：
//  - **上升时间**：增大λ可缩短。
//  - **超调量**：增大φ或减小λ可抑制。
//  - **恢复抖动**：减小η或增大φ。
//
//#### **2. 扰动实验法**
//- **操作**：系统稳定时施加脉冲干扰（如用手拨动云台）。
//- **分析**：
//  - **恢复时间**：增大η可加快恢复。
//  - **恢复过程振荡**：减小η或增大λ。
//
//#### **3. 扫频测试法**
//- **操作**：让目标值按正弦波变化（频率由低到高）。
//- **分析**：
//  - **低频跟踪差**：增大λ或η。
//  - **高频抖动**：增大φ或降低η。
void SMC_Init(SMC_t *smc, SMC_init_config_t *config)
{
//  λ，lambda：决定收敛速度，通常取1~5。平衡响应速度与稳定性
//  η，eta：需大于系统扰动幅值，通过实验观察响应调整。抑制扰动，需避免过大抖振
//  φ，phi：边界层厚度，越小跟踪精度越高，但抖振风险越大。抑制高频抖振
//  - **若响应过慢**：增大λ（每次+1）。
//  - **若震荡发散**：减小λ（每次-0.5）。
//**目标**：使系统能快速接近目标，且无持续振荡（允许轻微超调）。（阶跃突变输入）
//  - **若恢复太慢**：增大η（每次+2）。
//  - **若抖振明显**（电机高频振动）：减小η或增大φ。
//**目标**：在扰动后1~3个控制周期内恢复稳定。（手动轻推云台）
//  - **抖振严重**（波形锯齿明显）：增大φ（每次+0.1）。
//  - **稳态误差大**：减小φ（每次-0.05）。
//**目标**：抖振幅度控制在执行器容忍范围内。（稳态输出波形）
//  | 响应慢，收敛时间长    	| λ太小         	| 增大λ             	|
//  | 超调大，振荡发散	| λ太大          | 减小λ             	|
//  | 受扰动后无法恢复     	| η太小          | 增大η             	|
//  | 稳态时电机高频振动    	| η太大或φ太小    	| 减小η 或 增大φ      	|
//  | 稳态误差持续存在      | φ太大或η太小    	| 减小φ 或 增大η      	|
//  | 控制输出剧烈跳变      | 未启用饱和函数   	| 检查代码中的饱和函数实现	|
  smc->lambda = config->lambda;
  smc->eta = config->eta;
  smc->phi = config->phi;
  smc->output_limit = config->output_limit;
}

float SMC_Calculate(SMC_t *smt, float target, float measure)
{
  smt->err = target - measure;
  //误差微分
  smt->derivative_err = (smt->err - smt->last_err) / smt->dt;
  smt->last_err = smt->err;

  // 滑模面: s = λ*e + e_dot
  smt->s = smt->lambda * smt->err + smt->derivative_err;
  //		放大误差			误差微分
  // 饱和函数代替符号函数
  if (fabsf(smt->s) > smt->phi)
  {
    smt->sat_s = (smt->s > 0) ?
	1.0f :
	-1.0f;
  }
  else
  {
    smt->sat_s = smt->s / smt->phi;
  }

  // 控制律: u_smc = λ*e_dot + η*sat(s/φ)
  smt->output = smt->lambda * smt->derivative_err + smt->eta * smt->sat_s;

  if (fabsf(smt->output) > smt->output_limit)
  {
    if (smt->output > 0)
    {
      smt->output = smt->output_limit;
    }
    else if (smt->output <= 0)
    {
      smt->output = -smt->output_limit;
    }
  }

  return smt->output;
}
/******************************** FUZZY PID **********************************/
static float FuzzyRuleKpRAW[7][7] = {
PB , PB , PM , PM , PS , ZE , ZE ,
PB , PB , PM , PS , PS , ZE , PS ,
PM , PM , PM , PS , ZE , PS , PS ,
PM , PM , PS , ZE , PS , PM , PM ,
PS , PS , ZE , PS , PS , PM , PM ,
PS , ZE , PS , PM , PM , PM , PB ,
ZE , ZE , PM , PM , PM , PB , PB };

static float FuzzyRuleKiRAW[7][7] = {
PB , PB , PM , PM , PS , ZE , ZE ,
PB , PB , PM , PS , PS , ZE , ZE ,
PB , PM , PM , PS , ZE , PS , PS ,
PM , PM , PS , ZE , PS , PM , PM ,
PS , PS , ZE , PS , PS , PM , PB ,
ZE , ZE , PS , PS , PM , PB , PB ,
ZE , ZE , PS , PM , PM , PB , PB };

static float FuzzyRuleKdRAW[7][7] = {
PS , PS , PB , PB , PB , PM , PS ,
PS , PS , PB , PM , PM , PS , ZE ,
ZE , PS , PM , PM , PS , PS , ZE ,
ZE , PS , PS , PS , PS , PS , ZE ,
ZE , ZE , ZE , ZE , ZE , ZE , ZE ,
PB , PS , PS , PS , PS , PS , PB ,
PB , PM , PM , PM , PS , PS , PB };

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule,
		     float (*fuzzyRuleKp)[7],
		     float (*fuzzyRuleKi)[7],
		     float (*fuzzyRuleKd)[7],
		     float kpRatio,
		     float kiRatio,
		     float kdRatio,
		     float eStep,
		     float ecStep)
{
  if (fuzzyRuleKp == NULL)
    fuzzyRule->FuzzyRuleKp = FuzzyRuleKpRAW;
  else
    fuzzyRule->FuzzyRuleKp = fuzzyRuleKp;
  if (fuzzyRuleKi == NULL)
    fuzzyRule->FuzzyRuleKi = FuzzyRuleKiRAW;
  else
    fuzzyRule->FuzzyRuleKi = fuzzyRuleKi;
  if (fuzzyRuleKd == NULL)
    fuzzyRule->FuzzyRuleKd = FuzzyRuleKdRAW;
  else
    fuzzyRule->FuzzyRuleKd = fuzzyRuleKd;

  fuzzyRule->KpRatio = kpRatio;
  fuzzyRule->KiRatio = kiRatio;
  fuzzyRule->KdRatio = kdRatio;

  if (eStep < 0.00001f)
    eStep = 1;
  if (ecStep < 0.00001f)
    ecStep = 1;
  fuzzyRule->eStep = eStep;
  fuzzyRule->ecStep = ecStep;
}

void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref)
{
  float eLeftTemp, ecLeftTemp;
  float eRightTemp, ecRightTemp;
  int eLeftIndex, ecLeftIndex;
  int eRightIndex, ecRightIndex;

  fuzzyRule->dt = DWT_GetDeltaT((void*) fuzzyRule->DWT_CNT);

  fuzzyRule->e = ref - measure;
  fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast) / fuzzyRule->dt;
  fuzzyRule->eLast = fuzzyRule->e;

  //隶属区间
  eLeftIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ?
      6 :
      (fuzzyRule->e <= -3 * fuzzyRule->eStep ?
	  0 :
	  (fuzzyRule->e >= 0 ?
	      ((int) (fuzzyRule->e / fuzzyRule->eStep) + 3) :
	      ((int) (fuzzyRule->e / fuzzyRule->eStep) + 2)));
  eRightIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ?
      6 :
      (fuzzyRule->e <= -3 * fuzzyRule->eStep ?
	  0 :
	  (fuzzyRule->e >= 0 ?
	      ((int) (fuzzyRule->e / fuzzyRule->eStep) + 4) :
	      ((int) (fuzzyRule->e / fuzzyRule->eStep) + 3)));
  ecLeftIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ?
      6 :
      (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ?
	  0 :
	  (fuzzyRule->ec >= 0 ?
	      ((int) (fuzzyRule->ec / fuzzyRule->ecStep) + 3) :
	      ((int) (fuzzyRule->ec / fuzzyRule->ecStep) + 2)));
  ecRightIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ?
      6 :
      (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ?
	  0 :
	  (fuzzyRule->ec >= 0 ?
	      ((int) (fuzzyRule->ec / fuzzyRule->ecStep) + 4) :
	      ((int) (fuzzyRule->ec / fuzzyRule->ecStep) + 3)));

  //隶属度
  eLeftTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ?
      0 :
      (fuzzyRule->e <= -3 * fuzzyRule->eStep ?
	  1 :
	  (eRightIndex - fuzzyRule->e / fuzzyRule->eStep - 3));
  eRightTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ?
      1 :
      (fuzzyRule->e <= -3 * fuzzyRule->eStep ?
	  0 :
	  (fuzzyRule->e / fuzzyRule->eStep - eLeftIndex + 3));
  ecLeftTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ?
      0 :
      (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ?
	  1 :
	  (ecRightIndex - fuzzyRule->ec / fuzzyRule->ecStep - 3));
  ecRightTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ?
      1 :
      (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ?
	  0 :
	  (fuzzyRule->ec / fuzzyRule->ecStep - ecLeftIndex + 3));

  fuzzyRule->KpFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecLeftIndex] + eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecLeftIndex] + eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecRightIndex] + eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecRightIndex];

  fuzzyRule->KiFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecLeftIndex] + eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecLeftIndex] + eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecRightIndex] + eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecRightIndex];

  fuzzyRule->KdFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecLeftIndex] + eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecLeftIndex] + eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecRightIndex] + eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecRightIndex];
}

/******************************* PID CONTROL *********************************/
// PID优化环节函数声明
static void f_Trapezoid_Intergral(analog_pid_t *pid);
static void f_Integral_Limit(analog_pid_t *pid);
static void f_Derivative_On_Measurement(analog_pid_t *pid);
static void f_Changing_Integration_Rate(analog_pid_t *pid);
static void f_Output_Filter(analog_pid_t *pid);
static void f_Derivative_Filter(analog_pid_t *pid);
static void f_Output_Limit(analog_pid_t *pid);
static void f_Proportion_Limit(analog_pid_t *pid);
static void f_PID_ErrorHandle(analog_pid_t *pid);

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void Analog_PID_Init(analog_pid_t *pid, analog_pid_init_config_t *config)
{
  pid->MaxOut = config->MaxOut;
  pid->IntegralLimit = config->IntegralLimit;
  pid->DeadBand = config->DeadBand;
  pid->Kp = config->Kp;
  pid->Ki = config->Ki;
  pid->Kd = config->Kd;
  pid->Improve = config->Improve;
  DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float Analog_PID_Calculate(analog_pid_t *pid, float measure, float ref)
{
  if (pid->Improve & ErrorHandle)
    f_PID_ErrorHandle(pid);

  pid->dt = DWT_GetDeltaT((void*) &pid->DWT_CNT);

  pid->Measure = measure;
  pid->Ref = ref;
  pid->Err = pid->Ref - pid->Measure;

  if (pid->User_Func1_f != NULL)
    pid->User_Func1_f(pid);

  if (user_abs(pid->Err) > pid->DeadBand)
  {
    if (pid->FuzzyRule == NULL)
    {
      pid->Pout = pid->Kp * pid->Err;
      pid->ITerm = pid->Ki * pid->Err * pid->dt;
      if (pid->OLS_Order > 2)
	pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
      else
	pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
    }
    else
    {
      pid->Pout = (pid->Kp + pid->FuzzyRule->KpFuzzy) * pid->Err;
      pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * pid->Err * pid->dt;
      if (pid->OLS_Order > 2)
	pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS,
									 pid->dt,
									 pid->Err);
      else
	pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Err - pid->Last_Err) / pid->dt;
    }

    if (pid->User_Func2_f != NULL)
      pid->User_Func2_f(pid);

    // 梯形积分
    if (pid->Improve & Trapezoid_Intergral)
      f_Trapezoid_Intergral(pid);
    // 变速积分
    if (pid->Improve & ChangingIntegrationRate)
      f_Changing_Integration_Rate(pid);
    // 微分先行
    if (pid->Improve & Derivative_On_Measurement)
      f_Derivative_On_Measurement(pid);
    // 微分滤波器
    if (pid->Improve & DerivativeFilter)
      f_Derivative_Filter(pid);
    // 积分限幅
    if (pid->Improve & Integral_Limit)
      f_Integral_Limit(pid);

    pid->Iout += pid->ITerm;

    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    // 输出滤波
    if (pid->Improve & OutputFilter)
      f_Output_Filter(pid);

    // 输出限幅
    f_Output_Limit(pid);

    // 无关紧要
    f_Proportion_Limit(pid);
  }

  pid->Last_Measure = pid->Measure;
  pid->Last_Output = pid->Output;
  pid->Last_Dout = pid->Dout;
  pid->Last_Err = pid->Err;
  pid->Last_ITerm = pid->ITerm;

  return pid->Output;
}

//梯形积分
static void f_Trapezoid_Intergral(analog_pid_t *pid)
{
  // 计算梯形的面积,(上底+下底)*高/2
  if (pid->FuzzyRule == NULL)
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
  else
    pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(analog_pid_t *pid)
{
  if (pid->Err * pid->Iout > 0)
  {
    // 积分呈累积趋势
    // Integral still increasing
    if (user_abs(pid->Err) <= pid->CoefB)
      return; // Full integral
    if (user_abs(pid->Err) <= (pid->CoefA + pid->CoefB))
      pid->ITerm *= (pid->CoefA - user_abs(pid->Err) + pid->CoefB) / pid->CoefA;
    else
      //最大阈值，不使用积分
      pid->ITerm = 0;
  }
}

//积分限幅
static void f_Integral_Limit(analog_pid_t *pid)
{
  static float temp_Output, temp_Iout;
  temp_Iout = pid->Iout + pid->ITerm;
  temp_Output = pid->Pout + pid->Iout + pid->Dout;
  if (user_abs(temp_Output) > pid->MaxOut)
  {
    if (pid->Err * pid->Iout > 0)
    {
      // 积分呈累积趋势
      // Integral still increasing
      pid->ITerm = 0;
    }
  }

  if (temp_Iout > pid->IntegralLimit)
  {
    pid->ITerm = 0;
    pid->Iout = pid->IntegralLimit;
  }
  if (temp_Iout < -pid->IntegralLimit)
  {
    pid->ITerm = 0;
    pid->Iout = -pid->IntegralLimit;
  }
}

//微分先行（仅使用反馈值而不计参考输入的微分）
static void f_Derivative_On_Measurement(analog_pid_t *pid)
{
  if (pid->FuzzyRule == NULL)
  {
    if (pid->OLS_Order > 2)
      pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
    else
      pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
  }
  else
  {
    if (pid->OLS_Order > 2)
      pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS,
								       pid->dt,
								       -pid->Measure);
    else
      pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Last_Measure - pid->Measure) / pid->dt;
  }
}

//微分滤波（采集微分时，滤除高频噪声）
static void f_Derivative_Filter(analog_pid_t *pid)
{
  pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) + pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

//输出滤波
static void f_Output_Filter(analog_pid_t *pid)
{
  pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) + pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

//输出限幅
static void f_Output_Limit(analog_pid_t *pid)
{
  if (pid->Output > pid->MaxOut)
  {
    pid->Output = pid->MaxOut;
  }
  if (pid->Output < -(pid->MaxOut))
  {
    pid->Output = -(pid->MaxOut);
  }
}

//比例限幅
static void f_Proportion_Limit(analog_pid_t *pid)
{
  if (pid->Pout > pid->MaxOut)
  {
    pid->Pout = pid->MaxOut;
  }
  if (pid->Pout < -(pid->MaxOut))
  {
    pid->Pout = -(pid->MaxOut);
  }
}

//电机堵转检测
// PID ERRORHandle Function
static void f_PID_ErrorHandle(analog_pid_t *pid)
{
  /*Motor Blocked Handle*/
  if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
    return;

  if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
  {
    // Motor blocked counting
    pid->ERRORHandler.error_count++;
  }
  else
  {
    pid->ERRORHandler.error_count = 0;
  }

  if (pid->ERRORHandler.error_count > 500)
  {
    // Motor blocked over 1000times
    pid->ERRORHandler.error_type = PID_MOTOR_BLOCKED_ERROR;
  }
}

/*************************** FEEDFORWARD CONTROL *****************************/
/**
 * @brief          前馈控制初始化
 * @param[in]      前馈控制结构体
 * @param[in]      略
 * @retval         返回空
 */
void Feedforward_Init(Feedforward_t *ffc,
		      float max_out,
		      float *c,
		      float lpf_rc,
		      uint16_t ref_dot_ols_order,
		      uint16_t ref_ddot_ols_order)
{
  ffc->MaxOut = max_out;

  // 设置前馈控制器参数 详见前馈控制结构体定义
  // set parameters of feed-forward controller (see struct definition)
  if (c != NULL && ffc != NULL)
  {
    ffc->c[0] = c[0];
    ffc->c[1] = c[1];
    ffc->c[2] = c[2];
  }
  else
  {
    ffc->c[0] = 0;
    ffc->c[1] = 0;
    ffc->c[2] = 0;
    ffc->MaxOut = 0;
  }

  ffc->LPF_RC = lpf_rc;

  // 最小二乘提取信号微分初始化
  // differential signal is distilled by OLS
  ffc->Ref_dot_OLS_Order = ref_dot_ols_order;
  ffc->Ref_ddot_OLS_Order = ref_ddot_ols_order;
  if (ref_dot_ols_order > 2)
    OLS_Init(&ffc->Ref_dot_OLS, ref_dot_ols_order);
  if (ref_ddot_ols_order > 2)
    OLS_Init(&ffc->Ref_ddot_OLS, ref_ddot_ols_order);

  ffc->DWT_CNT = 0;

  ffc->Output = 0;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
  ffc->dt = DWT_GetDeltaT((void*) &ffc->DWT_CNT);

  ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) + ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);

  // 计算一阶导数
  // calculate first derivative
  if (ffc->Ref_dot_OLS_Order > 2)
    ffc->Ref_dot = OLS_Derivative(&ffc->Ref_dot_OLS, ffc->dt, ffc->Ref);
  else
    ffc->Ref_dot = (ffc->Ref - ffc->Last_Ref) / ffc->dt;

  // 计算二阶导数
  // calculate second derivative
  if (ffc->Ref_ddot_OLS_Order > 2)
    ffc->Ref_ddot = OLS_Derivative(&ffc->Ref_ddot_OLS, ffc->dt, ffc->Ref_dot);
  else
    ffc->Ref_ddot = (ffc->Ref_dot - ffc->Last_Ref_dot) / ffc->dt;

  // 计算前馈控制输出
  // calculate feed-forward controller output
  ffc->Output = ffc->c[0] * ffc->Ref + ffc->c[1] * ffc->Ref_dot + ffc->c[2] * ffc->Ref_ddot;

  ffc->Output = float_constrain(ffc->Output, -ffc->MaxOut, ffc->MaxOut);

  ffc->Last_Ref = ffc->Ref;
  ffc->Last_Ref_dot = ffc->Ref_dot;

  return ffc->Output;
}

/*************************LINEAR DISTURBANCE OBSERVER *************************/
void LDOB_Init(LDOB_t *ldob,
	       float max_d,
	       float deadband,
	       float *c,
	       float lpf_rc,
	       uint16_t measure_dot_ols_order,
	       uint16_t measure_ddot_ols_order)
{
  ldob->Max_Disturbance = max_d;

  ldob->DeadBand = deadband;

  // 设置线性扰动观测器参数 详见LDOB结构体定义
  // set parameters of linear disturbance observer (see struct definition)
  if (c != NULL && ldob != NULL)
  {
    ldob->c[0] = c[0];
    ldob->c[1] = c[1];
    ldob->c[2] = c[2];
  }
  else
  {
    ldob->c[0] = 0;
    ldob->c[1] = 0;
    ldob->c[2] = 0;
    ldob->Max_Disturbance = 0;
  }

  // 设置Q(s)带宽  Q(s)选用一阶惯性环节
  // set bandwidth of Q(s)    Q(s) is chosen as a first-order low-pass form
  ldob->LPF_RC = lpf_rc;

  // 最小二乘提取信号微分初始化
  // differential signal is distilled by OLS
  ldob->Measure_dot_OLS_Order = measure_dot_ols_order;
  ldob->Measure_ddot_OLS_Order = measure_ddot_ols_order;
  if (measure_dot_ols_order > 2)
    OLS_Init(&ldob->Measure_dot_OLS, measure_dot_ols_order);
  if (measure_ddot_ols_order > 2)
    OLS_Init(&ldob->Measure_ddot_OLS, measure_ddot_ols_order);

  ldob->DWT_CNT = 0;

  ldob->Disturbance = 0;
}

float LDOB_Calculate(LDOB_t *ldob, float measure, float u)
{
  ldob->dt = DWT_GetDeltaT((void*) &ldob->DWT_CNT);

  ldob->Measure = measure;

  ldob->u = u;

  // 计算一阶导数
  // calculate first derivative
  if (ldob->Measure_dot_OLS_Order > 2)
    ldob->Measure_dot = OLS_Derivative(&ldob->Measure_dot_OLS,
				       ldob->dt,
				       ldob->Measure);
  else
    ldob->Measure_dot = (ldob->Measure - ldob->Last_Measure) / ldob->dt;

  // 计算二阶导数
  // calculate second derivative
  if (ldob->Measure_ddot_OLS_Order > 2)
    ldob->Measure_ddot = OLS_Derivative(&ldob->Measure_ddot_OLS,
					ldob->dt,
					ldob->Measure_dot);
  else
    ldob->Measure_ddot = (ldob->Measure_dot - ldob->Last_Measure_dot) / ldob->dt;

  // 估计总扰动
  // estimate external disturbances and internal disturbances caused by model uncertainties
  ldob->Disturbance = ldob->c[0] * ldob->Measure + ldob->c[1] * ldob->Measure_dot + ldob->c[2] * ldob->Measure_ddot - ldob->u;
  ldob->Disturbance = ldob->Disturbance * ldob->dt / (ldob->LPF_RC + ldob->dt) + ldob->Last_Disturbance * ldob->LPF_RC / (ldob->LPF_RC + ldob->dt);

  ldob->Disturbance = float_constrain(ldob->Disturbance,
				      -ldob->Max_Disturbance,
				      ldob->Max_Disturbance);

  // 扰动输出死区
  // deadband of disturbance output
  if (user_abs(ldob->Disturbance) > ldob->DeadBand * ldob->Max_Disturbance)
    ldob->Output = ldob->Disturbance;
  else
    ldob->Output = 0;

  ldob->Last_Measure = ldob->Measure;
  ldob->Last_Measure_dot = ldob->Measure_dot;
  ldob->Last_Disturbance = ldob->Disturbance;

  return ldob->Output;
}

/*************************** Tracking Differentiator ***************************/
void TD_Init(TD_t *td, float r, float h0)
{
  td->r = r;
  td->h0 = h0;

  td->x = 0;
  td->dx = 0;
  td->ddx = 0;
  td->last_dx = 0;
  td->last_ddx = 0;
}
float TD_Calculate(TD_t *td, float input)
{
  static float d, a0, y, a1, a2, a, fhan;

  td->dt = DWT_GetDeltaT((void*) &td->DWT_CNT);

  if (td->dt > 0.5f)
    return 0;

  td->Input = input;

  d = td->r * td->h0 * td->h0;
  a0 = td->dx * td->h0;
  y = td->x - td->Input + a0;
  a1 = sqrt(d * (d + 8 * user_abs(y)));
  a2 = a0 + sign(y) * (a1 - d) / 2;
  a = (a0 + y) * (sign(y + d) - sign(y - d)) / 2 + a2 * (1 - (sign(y + d) - sign(y - d)) / 2);
  fhan = -td->r * a / d * (sign(a + d) - sign(a - d)) / 2 - td->r * sign(a) * (1 - (sign(a + d) - sign(a - d)) / 2);

  td->ddx = fhan;
  td->dx += (td->ddx + td->last_ddx) * td->dt / 2;
  td->x += (td->dx + td->last_dx) * td->dt / 2;

  td->last_ddx = td->ddx;
  td->last_dx = td->dx;

  return td->x;
}
