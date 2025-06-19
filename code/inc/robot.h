#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "Pid.h"
#include "zf_common_typedef.h"

// 流水线状态
enum StateEnum {
	StatePreparing,		// 准备中
	StateProcessing,	// 处理中
	StateEnd,			// 结束
};

// 流水线状态机水线，用于多线程调用
typedef struct _Pipeline {
	bool		   isRun;	 // 是否运行
	enum StateEnum state;
} PipelineType;

typedef struct _JumpLineType {
	Vector2f Pos[5];
	float	 reachTime[5];
	bool	 Lerp;	  // 是否进行插值
} JumpLineType;

typedef struct _robot {
	PipelineType pipeline;

	LegType* left;
	LegType* right;

	IMUType* posture;

	ParamType	 param;
	JumpLineType jumpLine;

	PIDType rollPID, yawPID, yawSpeedPID;				  // 平衡PID
	PIDType pitchSpeedPID, pitchAnglePID, pitchVecPID;	  // 速度环、角度环、速度环 俯仰平衡PID

	float left_Torque, right_Torque;
	float LeftSum, RightSum;
	float speedNow, speedSet;	 // 当前速度，设定速度绝对值
	float speedRollNow, speedRollSet;

	// TODO: CameraType
	Vector2f ZeroPoint;

} RobotType;

extern RobotType robot;

#endif
