#ifndef _SERVOMOTOR_H_
#define _SERVOMOTOR_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

// 舵机初始化
#define FL_CHANNEL ATOM0_CH4_P21_6	   // 前左腿pwm通道
#define FR_CHANNEL ATOM0_CH3_P21_5	   // 前右腿pwm通道
#define BL_CHANNEL ATOM0_CH1_P21_3	   // 后左腿pwm通道1
#define BR_CHANNEL ATOM0_CH2_P21_4	   // 后右腿pwm通道

// 关节编号
typedef enum { Fl, Fr, Bl, Br } JointNum;

// 舵机结构体
typedef struct _Servo {
	pwm_channel_enum pin;	 // PWM 通道
	JointNum		 num;

	float angleSet;	   // 设定的实际转角 解算出来的角度值 弧度 （0-180度）
	float angleAdj;	   // 调试的调节值
	float angleLeg;	   // 在Leg系的机械角度，见Leg.h的θ1和θ4
	int8  sign;		   // 正负号，正负向

	float deltaAngle;	 // 舵机安装偏移角度 弧度 （0-180度）

	int32_t PWMSet;
} ServoType;

extern ServoType Servo[4];

void ServoInit(void);
void ServoFunc(void);

#endif
