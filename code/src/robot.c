#include "robot.h"
#include "leg.h"
#include "MathLib.h"
#include "imu.h"
#include "string.h"

RobotType robot;
float Kp_1;
float Ki_1;
float Kd_1;

float Kp_2;
float Ki_2;
float Kd_2;

float Kp_3;
float Ki_3;
float Kd_3;
float Roll_Kp   = 0.00005f;	 // 转向系数0.00009f
float Kspeed= 0.0085f;
float Kp_visual=3.07f;
float Kd_visual=2.4f;//28.4
float Kp_nonlinear_visual=0.01f;
float K_imu=0.3f;

/**
 * @brief robot初始化
 *
 * @param robot
 */
void robotInit(RobotType* robot)
{
	memset(robot, 0, sizeof(RobotType));

	robot->left	 = &(legLeft);
	robot->right = &(legRight);

	Start(&(robot->pipeline));
	RobotJumpLineInit();

	robot->posture		   = &(IMUdata);
	// TODO jumpLine

	robot->param.leftTime  = 0;
	robot->param.rightTime = 0;
	robot->param.PosZero.x = 0;
	robot->param.PosZero.y = 22.6838;	 // 39  24.2739
	robot->left->PosSet	   = robot->param.PosZero;
	robot->right->PosSet   = robot->param.PosZero;

	LegInit();
	BalanceInit();

	// AngleCalculate(robot->left, robot->param.PosZero);	   // 更新角度值
	// AngleCalculate(robot->right, robot->param.PosZero);	   // TODO：直接给PosSet

	legLeft.PosSet	= robot->param.PosZero;
	legRight.PosSet = robot->param.PosZero;

	// LegReset();	   // 腿部到初始位置
};

void RobotJumpLineInit(void)
{
	// TODO
	robot.jumpLine.Pos[0].x = 0;
	robot.jumpLine.Pos[0].y = 0;

	robot.jumpLine.Pos[1].x = 0;
	robot.jumpLine.Pos[1].y = 0;

	robot.jumpLine.Pos[2].x = 0;
	robot.jumpLine.Pos[2].y = 0;

	robot.jumpLine.Pos[3].x = 0;
	robot.jumpLine.Pos[3].y = 0;

	robot.jumpLine.Pos[4].x = 0;
	robot.jumpLine.Pos[4].y = 0;
	;
};

bool RobotDrawLine(Vector2f PosTarget, float reachTime)
{
	switch (robot.pipeline.state) {
		case StatePreparing:

			robot.param.reachTime  = reachTime;
			robot.param.runTime	   = 0;

			robot.left->PosTarget  = PosTarget;
			robot.right->PosTarget = PosTarget;

			robot.left->PosSet	   = PosTarget;
			robot.right->PosSet	   = PosTarget;

			robot.left->PosStart   = ForwardKinematics(robot.left->angle1Set, robot.left->angle4Set);	 // 当前的坐标
			robot.right->PosStart  = ForwardKinematics(robot.right->angle1Set, robot.right->angle4Set);

			Prepared(&(robot.pipeline));
			break;

		case StateProcessing:
			if (robot.param.runTime < robot.param.reachTime) {
				// Start + (target - start)*(runtime/reachtime)
				robot.left->PosSet.x = Lerp(robot.left->PosStart.x, robot.left->PosTarget.x, robot.param.runTime / robot.param.reachTime);
				robot.left->PosSet.y = Lerp(robot.left->PosStart.y, robot.left->PosTarget.y, robot.param.runTime / robot.param.reachTime);

				robot.right->PosSet.x = Lerp(robot.right->PosStart.x, robot.right->PosTarget.x, robot.param.runTime / robot.param.reachTime);
				robot.right->PosSet.y = Lerp(robot.right->PosStart.y, robot.right->PosTarget.y, robot.param.runTime / robot.param.reachTime);

				AngleCalculate(robot.left, robot.left->PosSet);	 // 更新角度值
				AngleCalculate(robot.right, robot.right->PosSet);

			} else {	// 时间到，进行结束
				Processed(&(robot.pipeline));
			}
			break;

		case StateEnd:
			return true;
	}
	return false;
}

// 机器人平衡控制相关函数
void BalanceInit(void)
{
	// 初始化平衡PID控制器
	PIDTypeInit(&robot.rollPID, 0.5f, 0.0f, 0.1f, PIDPOS, 100.0f);
	PIDTypeInit(&robot.pitchAnglePID, 1.0f, 0.0f, 0.2f, PIDPOS, 100.0f);
	PIDTypeInit(&robot.pitchSpeedPID, 0.8f, 0.0f, 0.15f, PIDPOS, 100.0f);
	PIDTypeInit(&robot.yawPID, 0.3f, 0.0f, 0.05f, PIDPOS, 50.0f);
	PIDTypeInit(&robot.yawSpeedPID, 0.4f, 0.0f, 0.08f, PIDPOS, 50.0f);
}

void RobotBalance(void)
{
	// 平衡控制算法实现
	float rollOutput = PIDOperation(&robot.rollPID, robot.posture->roll, 0.0f);
	float pitchOutput = PIDOperation(&robot.pitchAnglePID, robot.posture->pitch, 0.0f);
	
	// 应用平衡控制输出到电机
	robot.left_Torque += rollOutput;
	robot.right_Torque -= rollOutput;
	
	robot.left_Torque += pitchOutput;
	robot.right_Torque += pitchOutput;
}
