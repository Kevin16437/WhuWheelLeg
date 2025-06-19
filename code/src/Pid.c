#include "Pid.h"
#include "MathLib.h"

/********************* 模糊PID *********************** */
#if USEFUZZY
float EFF[7];	 // 误差e的隶属度值
float DFF[7];	 // 误差de/dt的隶属度值

const int rule_p[7][7] = {
	{NB, NB, NM, NM, NS, ZO, ZO}, //  kp规则表
	{NB, NB, NM, NS, NS, ZO, NS},
	{NM, NM, NM, NS, ZO, NS, NS},
	{NM, NM, NS, ZO, NS, NM, NM},
	{NS, NS, ZO, NS, NS, NM, NM},
	{NS, ZO, NS, NM, NM, NM, NB},
	{ZO, ZO, NM, NM, NM, NB, NB}
};
const int rule_d[7][7] = {
	{PS, NS, NB, NB, NB, NM, PS}, //  kd规则表
	{PS, NS, NB, NM, NM, NS, ZO},
	{ZO, NS, NM, NM, NS, NS, ZO},
	{ZO, NS, NS, NS, NS, NS, ZO},
	{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
	{PB, NS, PS, PS, PS, PS, PB},
	{PB, PM, PM, PM, PS, PS, PB}
};
DMFType		  DMF;
FuzzyPidType* Turn_FuzzyPD;
#endif

/**
 * @brief  全局PID结构体初始化
 *
 * @param pid
 * @param param
 * @param outputThreshold 0 为不限幅
 * @param mode
 */
void PIDTypeInit(PIDType* pid, float kp, float ki, float kd, PIDMode mode, float outputThreshold)
{
	if (pid == NULL)
		return;

	pid->kp				 = kp;
	pid->ki				 = ki;
	pid->kd				 = kd;

	pid->mode			 = mode;
	pid->outputThreshold = outputThreshold;	   // 0表示不限幅，具体见 PIDOperation

	pid->pOut			 = 0;
	pid->iOut			 = 0;
	pid->dOut			 = 0;
	pid->output			 = 0;

	pid->err[0]			 = 0;
	pid->err[1]			 = 0;
	pid->err[2]			 = 0;

	pid->kiScale		 = 1;
}

/// @brief  增量式PID和位置PID的运算函数
/// @param pid
/// @param real
/// @param target
/// @return
float PIDOperation(PIDType* pid, float real, float target)
{
	if (pid == NULL)
		return 0.f;

	pid->err[0] = target - real;

	switch (pid->mode) {
			// 位置式pid
		case PIDPOS:
			pid->err[2] = 0.5f * pid->err[0] + 0.5f * pid->err[2];	  // 滤波后累计Ki项

			pid->pOut	= pid->kp * pid->err[0];
			pid->iOut	= pid->ki * pid->err[2];
			pid->dOut	= pid->kd * (pid->err[0] - pid->err[1]);
			pid->output = pid->pOut + pid->kiScale * pid->iOut + pid->dOut;

			pid->err[1] = pid->err[0];
			break;

			// 增量式pid
		case PIDINC:
			pid->pOut	= pid->kp * (pid->err[0] - pid->err[1]);
			pid->iOut	= pid->ki * pid->err[0];
			pid->dOut	= pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
			pid->output = pid->pOut + pid->iOut + pid->dOut;

			pid->err[2] = pid->err[1];
			pid->err[1] = pid->err[0];
			break;

		case PIDFuzzy:
			// TODO: 实现模糊PID
			break;

		default:
			break;
	}

	// 输出限幅
	if (pid->outputThreshold != 0) {
		Limit(pid->output, -pid->outputThreshold, pid->outputThreshold);
	}

	return pid->output;
}

/**
 * @brief PID参数重置
 *
 * @param pid
 */
void PIDReset(PIDType* pid)
{
	if (pid == NULL)
		return;

	pid->pOut = 0;
	pid->iOut = 0;
	pid->dOut = 0;
	pid->output = 0;

	pid->err[0] = 0;
	pid->err[1] = 0;
	pid->err[2] = 0;
}

/**
 * @brief 设置PID参数
 *
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 */
void PIDSetParam(PIDType* pid, float kp, float ki, float kd)
{
	if (pid == NULL)
		return;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}
