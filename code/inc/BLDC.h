#ifndef _BRUSHLESS_H_
#define _BRUSHLESS_H_

#include "Pid.h"
#include "zf_common_typedef.h"

#define NEWMSG 1	// 新的消息处理

// 无刷电机串口初始化
#define BLDC_DRIVER_UART	 (UART_3)
#define BLDC_DRIVER_BAUDRATE (460800)
#define BLDC_DRIVER_RX		 (UART3_TX_P15_7)
#define BLDC_DRIVER_TX		 (UART3_TX_P15_6)

//  P10-5 6 编码器
//  P21-6 7 Uart RT
// encod1编码器

// PWM>0时
// 逆时针转 右腿向前转

// 无刷电机相关
#define OUTPUT_DUTY_MAX ((uint16) (4000))	 // 占空比输出最大值
#define PULSEPERROUND	(32767)				 // 编码器一圈脉冲值  32767
#define PULSETIME		(0.01)				 // 两次读取编码器之间时间，10ms

extern double Stretch_Left, Stretch_Right, Stretch;

typedef enum { Left, Right } LegNum;

// 控制模式
typedef enum _BldcMode {
	CURRENT,	// 电流模式
	RPM,		// 速度模式
	POSITION	// 位置模式
} BldcMode;

// 数值
typedef struct _Value {
	volatile float speed;	   // TODO rpm
	volatile float angle;	   // 弧度 累计位置，-∞ ~ +∞
	volatile int16 current;	   // 电流PWM Set得到
} ValueType;

// 脉冲 不知道能不能用得到
typedef struct _PulseType {
	volatile uint16_t pulseRead;	 // 当前读取编码器数
	volatile uint16_t pulseLast;	 // 上次读取编码器数
	volatile uint16_t Distanse;		 // 当前编码器与上次编码器之差
	volatile int32_t  pulseTotal;	 // 累计编码器数
} PulseType;

// 无刷电机结构体
typedef struct _Bldc {
	LegNum num;		// 腿部编号
	BldcMode mode;	// 控制模式

	ValueType value;	// 当前值
	ValueType target;	// 目标值

	PulseType pulse;	// 脉冲相关

	PIDType speedPID;	// 速度PID
	PIDType anglePID;	// 角度PID

	float outputCurrent;	// 输出电流
	float outputPWM;		// 输出PWM

	bool isEnable;	// 是否使能
} BldcType;

extern BldcType Motor[2];

void BldcInit(void);
void BldcSetMode(BldcType* motor, BldcMode mode);
void BldcSetTarget(BldcType* motor, float target);
void BldcControl(void);
void BldcEnable(BldcType* motor, bool enable);

#endif
