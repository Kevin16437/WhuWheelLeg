#include "leg.h"
#include "robot.h"
#include "MathLib.h"
#include "string.h"

LegType legLeft;
LegType legRight;

void LegInit(void)
{
	memset(&legLeft, 0, sizeof(LegType));
	memset(&legRight, 0, sizeof(LegType));

	legLeft.num		   = Left;
	legRight.num	   = Right;

	legLeft.front	   = &(Servo[Fl]);
	legLeft.behind	   = &(Servo[Bl]);
	legLeft.wheel	   = &(Motor[0]);
	legLeft.angle1Set  = PI;	// TODO：
	legLeft.angle4Set  = 0;		// TODO：

	legRight.front	   = &(Servo[Fr]);
	legRight.behind	   = &(Servo[Br]);
	legRight.wheel	   = &(Motor[1]);
	legRight.angle1Set = PI;	// TODO：
	legRight.angle4Set = 0;		// TODO：

	MotionInit(&(legLeft.motion));
	MotionInit(&(legRight.motion));

	legLeft.RunTime	 = &(robot.param.leftTime);
	legRight.RunTime = &(robot.param.rightTime);
}

/**
 * @brief 腿画曲线 默认三次
 *
 * @param leg
 */
bool LegDrawCurve(LegType* leg, float reachTime)
{
	switch (robot.pipeline.state) {
		case StatePreparing:
			leg->reachTime = reachTime;

			/* code */

			Prepared(&(robot.pipeline));
			break;

		case StateProcessing:;
			uint64_t runTime = *(leg->RunTime);
			if (runTime < leg->reachTime) {
				leg->PosSet = BezierCalculate(leg->motion.degree, leg->motion.CtrlPoint, runTime / leg->reachTime);
			} else {
				Processed(&(robot.pipeline));
			}

			AngleCalculate(leg, leg->PosSet);	 // 更新角度值
			break;

		case StateEnd:

			/* code */

			return true;
	}
	return false;
};

/**
 * @brief 腿画直线，线性插值
 *
 * @param leg
 * @param PosTarget
 * @param reachTime
 */
bool LegDrawLine(LegType* leg, Vector2f PosTarget, float reachTime)
{
	switch (robot.pipeline.state) {
		case StatePreparing:
			leg->reachTime	= reachTime;
			leg->PosTarget	= PosTarget;
			leg->PosStart	= ForwardKinematics(leg->angle1Set, leg->angle4Set);	// 当前的坐标

			*(leg->RunTime) = 0;	// 时间清零

			Prepared(&(robot.pipeline));

			break;

		case StateProcessing:;
			uint64_t runTime = *(leg->RunTime);
			if (runTime < leg->reachTime) {
				// Start + (target - start)*(runtime/reachtime)
				leg->PosSet.x = Lerp(leg->PosStart.x, leg->PosTarget.x, runTime / leg->reachTime);
				leg->PosSet.y = Lerp(leg->PosStart.y, leg->PosTarget.y, runTime / leg->reachTime);

				AngleCalculate(leg, leg->PosSet);	 // 更新角度值

			} else {	// 时间到，进行结束
				Processed(&(robot.pipeline));
			}
			break;

		case StateEnd:
			return true;
	}
	return false;
}

/**
 * @brief 逆运动学解算
 *
 * @param point 目标点坐标
 * @return Vector2f 返回角度1和角度4
 */
Vector2f InverseKinematics(Vector2f point)
{
	Vector2f result;
	float x = point.x;
	float y = point.y;
	
	// 逆运动学计算
	float L = sqrt(x*x + y*y);
	float alpha = atan2(y, x);
	
	// 使用余弦定理计算关节角度
	float cos_beta = (L1*L1 + L*L - (L2+L3)*(L2+L3)) / (2*L1*L);
	float beta = acos(Constrain(cos_beta, -1.0f, 1.0f));
	
	result.x = alpha + beta;  // angle1
	result.y = alpha - beta;  // angle4
	
	return result;
}

/**
 * @brief 正运动学解算
 *
 * @param angle1 关节1角度
 * @param angle4 关节4角度
 * @return Vector2f 返回末端位置坐标
 */
Vector2f ForwardKinematics(float angle1, float angle4)
{
	Vector2f result;
	
	// 正运动学计算
	float x1 = L1 * cos(angle1);
	float y1 = L1 * sin(angle1);
	
	float x4 = L4 * cos(angle4);
	float y4 = L4 * sin(angle4);
	
	// 计算末端位置
	result.x = (x1 + x4) / 2;
	result.y = (y1 + y4) / 2;
	
	return result;
}

/**
 * @brief 根据位置计算角度
 *
 * @param leg 腿部结构体
 * @param pos 目标位置
 */
void AngleCalculate(LegType* leg, Vector2f pos)
{
	Vector2f angles = InverseKinematics(pos);
	leg->angle1Set = angles.x;
	leg->angle4Set = angles.y;
	
	// 更新舵机角度
	AngleLeg2Servo(leg);
}

/**
 * @brief 将腿部角度转换为舵机角度
 *
 * @param leg 腿部结构体
 */
void AngleLeg2Servo(LegType* leg)
{
	if (leg->num == Left) {
		leg->front->angleSet = leg->angle1Set;
		leg->behind->angleSet = leg->angle4Set;
	} else {
		leg->front->angleSet = leg->angle1Set;
		leg->behind->angleSet = leg->angle4Set;
	}
}

/**
 * @brief 检查点是否在工作空间内
 *
 * @param point 目标点
 * @return bool 是否在工作空间内
 */
bool PointLimit(Vector2f* point)
{
	float distance = sqrt(point->x * point->x + point->y * point->y);
	float minReach = abs(L1 - L2 - L3);
	float maxReach = L1 + L2 + L3;
	
	if (distance < minReach || distance > maxReach) {
		return false;
	}
	
	return true;
}

/**
 * @brief 腿部复位到零点
 */
void LegReset(void)
{
	legLeft.PosSet = robot.param.PosZero;
	legRight.PosSet = robot.param.PosZero;
	
	AngleCalculate(&legLeft, legLeft.PosSet);
	AngleCalculate(&legRight, legRight.PosSet);
}
