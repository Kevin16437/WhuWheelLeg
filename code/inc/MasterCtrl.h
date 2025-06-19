#ifndef _MASTERCTRL_H_
#define _MASTERCTRL_H_

#include "zf_common_typedef.h"
#include "vector.h"

// 运动类型
typedef enum _MotionType {
	MOTION_STOP,		// 停止
	MOTION_FORWARD,		// 前进
	MOTION_BACKWARD,	// 后退
	MOTION_TURN_LEFT,	// 左转
	MOTION_TURN_RIGHT,	// 右转
	MOTION_JUMP,		// 跳跃
	MOTION_BALANCE		// 平衡
} MotionType;

// 运动控制结构体
typedef struct _Motion {
	MotionType type;		// 运动类型
	uint8 degree;			// 贝塞尔曲线阶数
	Vector2f CtrlPoint[10];	// 控制点数组
	float duration;			// 运动持续时间
	bool isActive;			// 是否激活
} MotionType;

// 参数结构体
typedef struct _Param {
	uint64_t leftTime;		// 左腿运行时间
	uint64_t rightTime;		// 右腿运行时间
	uint64_t systemTime;	// 系统运行时间
	float balanceAngle;		// 平衡角度
	float targetSpeed;		// 目标速度
} ParamType;

// IMU数据结构体
typedef struct _IMU {
	float roll;		// 横滚角
	float pitch;	// 俯仰角
	float yaw;		// 偏航角
	float gyroX;	// X轴角速度
	float gyroY;	// Y轴角速度
	float gyroZ;	// Z轴角速度
	float accelX;	// X轴加速度
	float accelY;	// Y轴加速度
	float accelZ;	// Z轴加速度
} IMUType;

// 状态机函数
void Prepared(PipelineType* pipeline);
void Processed(PipelineType* pipeline);
void MotionInit(MotionType* motion);

#endif
