#ifndef _LEG_H_
#define _LEG_H_
#include "MasterCtrl.h"
#include "MathLib.h"
#include "Servo.h"
#include "bldc.h"
#include "vector.h"
#include "zf_common_typedef.h"

// 腿长 单位为mm
#define L1	   60
#define L2	   88
#define L3	   88
#define L4	   60
#define L5	   37.5
#define WHEELR 67	 // 直径

typedef struct _leg {
	LegNum num;

	ServoType* front;
	ServoType* behind;

	BldcType* wheel;

	// 腿部坐标系下的坐标，原点在机器人的重心处
	float angle1Set, angle4Set;	   // 角度设定值，PosSet直接解算得到的结果

	Vector2f PosNow, PosSet;		 // 实际坐标点 和 设定坐标点。单位mm，插值运算Set是控制的
	Vector2f PosStart, PosTarget;	 // 直线运动

	float reachTime;	// 设定移动到目标位置的移动时间(ms)

	uint64_t* RunTime;	  // 运行计时器时间

	MotionType motion;

} LegType;

extern LegType legLeft;
extern LegType legRight;

void LegInit(void);

bool	 PointLimit(Vector2f* point);
Vector2f InverseKinematics(Vector2f point);				   // 逆解 输出C1 C4
Vector2f ForwardKinematics(float angle1, float angle4);	   // 正解，输出x和y坐标

void LegReset(void);												// 重置PosZero，移动到Poszero
bool LegDrawLine(LegType* leg, Vector2f PosTarget, float reachTime);	// 腿画直线，插值运算
bool LegDrawCurve(LegType* leg, float reachTime);						// 腿画直线，贝塞尔插值

void AngleCalculate(LegType* leg, Vector2f pos);	// 由Pos更新腿角度 Servo.angleSet
void AngleLeg2Servo(LegType* leg);

#endif
