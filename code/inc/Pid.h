#ifndef _PID_H
#define _PID_H

#include "zf_common_typedef.h"

/*********************** 模糊PID **************** */
#define USEFUZZY 1

#define PB		  3		// 正大
#define PM		  2		// 正中
#define PS		  1		// 正小
#define ZO		  0		// 零
#define NS		  -1	// 负小
#define NM		  -2	// 负中
#define NB		  -3	// 负大
#define EC_FACTOR 1		// 误差变化率的量化

typedef enum _PIDMode {
	PIDPOS,
	PIDINC,
	PIDFuzzy	// 模糊PID
} PIDMode;

// 全局PID
typedef struct _PID {
	PIDMode mode;

	volatile float kp;
	volatile float ki;
	volatile float kd;

	volatile float pOut;
	volatile float iOut;
	volatile float dOut;

	volatile float err[3];	  // 误差err：now、last、lastlast

	float outputThreshold;	  // 输出限幅

	volatile float output;

	float kiScale;	  // Ki积分分离或积分系数
} PIDType;

/****** 模糊PID使用结构体 */
typedef struct _DMFType {
	int EF[2];
	int En[2];
	int DF[2];
	int Dn[2];
} DMFType;

void PIDInit(PIDType* pid, PIDMode mode, float kp, float ki, float kd, float outputThreshold);
float PIDCalculate(PIDType* pid, float target, float current);
void PIDReset(PIDType* pid);

#endif
