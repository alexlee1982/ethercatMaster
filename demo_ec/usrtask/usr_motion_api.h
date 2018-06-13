#ifndef USR_MOTION_API_H
#define USR_MOTION_API_H
#include "motionCommandType.h"
#include "motionInterfaceBuffer.h"

#include "motionParameterType.h"
#include "motionConfig.h"
#include "motionFeedback.h"
#include "posemath.h"



#ifdef __cplusplus
extern "C" {
#endif

extern MotionConfig *emcmotConfig ;
extern MotionCommandBuffer *commandShmem ;
extern VRobotMotionCommand  *motCmd ;
extern MotionCommandQueue *interpList ;
extern VRobotMotionCommand *emcmotCommand;
extern MotionFeedback *motionFb;

extern int CTRL_ServoReset(void);			//伺服复位，用于伺服清报警
extern int CTRL_ServoEnable(int enable);	//伺服使能，enbale 为0：下使能 为1：上使能


extern int CTRL_USR_Init(void);
extern void CTRL_USR_Exit(void);



extern int CTRL_AddSingleAxisMove(SingleAxisMove *singleMove); //单轴运动
extern int CTRL_AddJointMove(JointMoveInformation *jointMove,u32 lineNumber,u32 fileNumber);		//关节插补movj
extern int CTRL_AddLinearMove(LinearMoveInformation *movl,u32 lineNumber,u32 fileNumber);			//直线插补，终点位置用关节表示
extern int CTRL_AddCircularMove(CircularMoveInformation *movC,u32 lineNumber,u32 fileNumber);
extern void CTRL_MovementStop();										//运动停止，并且后面未执行的命令也不会执行
extern int CTRL_AddDescartesLinearMove(LinearDescartesMoveInformation *movDL,u32 lineNumber,u32 fileNumber);//直线插补，终点位置用x/y/z表示。
extern int CTRL_AddDescartesCircularMove(CircularDescartesMoveInformation *movDC,u32 lineNumber,u32 fileNumber);//圆弧插补，目标位置使用笛卡尔位置值
extern void CTRL_GetJointValue(JointPoint * joint);
extern double CTRL_GetJointValueFor3DTest(int jointNumber);
extern double CTRL_GetPositionValue(int axis);


extern int CTRL_GetTCPInUserSpace(PointPose *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool);//输入关节位置，计算工具尖端点在用户坐标系下的位姿。
 extern int CTRL_GetTCPInUserSpaceInMatrix(PmHomogeneous *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool); // 软件内部使用

extern int CTRL_GetTCPInJoint(JointPoint *joint,PointPose *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool);//已知工具位姿，反解关节位置结果放到joint中。返回值为0时表示结果有效。
extern int CTRL_GetTCPInJointInMatrix(JointPoint *joint,JointPoint *preJoint,PmHomogeneous *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool);

extern int CTRL_GetMotionStatus(MotionFeedback * fb); 
//extern	 void CTRL_SetSocket(char * IP, int port, int cycleTime);
//jp , 用户输入的三个点的关节值信息，out 计算出的用户坐标系x/y/z/a/bc的值。
extern int CTRL_UserCalibration(ThreeJointPoints *jp, ToolCoordianteInformation *tool,UserCoordianteInformation *out);
//设置伺服电机的原点偏移值。其中offset 表示电机的编码器反馈脉冲值，axis表示轴号:1-6。
extern int CTRL_SetOriginOffset(s64 offset,u32 axis);
//获取原点偏移值，axis为轴号(1-6)，offset返回偏移值。
extern int CTRL_GetOriginOffset(s64 * offset,u32 axis);
//设置机器人连杆参数, length : 连杆参数长度，num: 连杆参数号码
extern int CTRL_SetPUMA(f64 length,u32 num);

//获得机器人连杆参数, length : 连杆参数长度，num: 连杆参数号码
extern int CTRL_GetPUMA(f64 *length,u32 num);

//设置示教时的线速度，velocity: 设置的速度值
extern int CTRL_SetTeachLinearVelocity(f64 velocity);

//获得示教时的线速度，velocity: 设置的速度值
extern int CTRL_GetTeachLinearVelocity(f64 *velocity);
//设置直线运动(MOVL) 的最大速度，velocity: 设置的速度值
extern int CTRL_SetLinearMaxVelocity(f64 velocity);
//获得直线运动(MOVL) 的最大速度，velocity: 设置的速度值
extern int CTRL_GetLinearMaxVelocity(f64 *velocity);
//设置直线运动(MOVL) 的加速度，acc: 设置的加速度值
extern int CTRL_SetLinearMoveAcc(f64 acc);
//获得直线运动(MOVL) 的加速度，acc: 设置的加速度值
extern int CTRL_GetLinearMoveAcc(f64 *acc);
//设置旋转运动的速度，velocity: 设置的速度值
extern int CTRL_SetRotateMoveVelocity(f64 velocity);
//获得旋转运动的速度，velocity: 设置的速度值
extern int CTRL_GetRotateMoveVelocity(f64 *velocity);
//设置旋转运动的加速度，acc:设置的加速度值。
extern int CTRL_SetRotateMoveAcc(f64 acc);
//获得旋转运动的加速度，acc:设置的加速度值。
extern int CTRL_GetRotateMoveAcc(f64 *acc);
//设置线性运动正软限，limit:软限数值，axis:关节号1-6  依次代表:X/Y/Z/A/B/C
extern int CTRL_SetLinearPositiveLimit(f64 limit,u32 axis);
//获取线性运动正软限，limit:软限数值，axis:关节号1-6  依次代表:X/Y/Z/A/B/C
extern int CTRL_GetLinearPositiveLimit(f64 *limit,u32 axis);
//设置线性运动负软限，limit:软限数值，axis:关节号1-6  依次代表:X/Y/Z/A/B/C
extern int CTRL_SetLinearNegativeLimit(f64 limit,u32 axis);
//获取线性运动负软限，limit:软限数值，axis:关节号1-6  依次代表:X/Y/Z/A/B/C
extern int CTRL_GetLinearNegativeLimit(f64 *limit,u32 axis);
//设置关节运动正软限，limit:软限数值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_SetJointPositiveLimit(f64 limit,u32 axis);
//获取关节运动正软限，limit:软限数值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_GetJointPositiveLimit(f64 *limit,u32 axis);
//设置关节运动负软限，limit:软限数值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_SetJointNegativeLimit(f64 limit,u32 axis);
//获取关节运动负软限，limit:软限数值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_GetJointNegativeLimit(f64 *limit,u32 axis);
//设置关节运动最大速度，velocity:速度值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_SetJointMaxVelocity(f64 velocity,u32 axis);
//设置关节运动最大速度，velocity:速度值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_GetJointMaxVelocity(f64 *velocity,u32 axis);

//设置关节运动最大加速度，acc:加速度值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_SetJointMaxAcc(f64 acc,u32 axis);
//获取关节运动最大加速度，acc:加速度值，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_GetJointMaxAcc(f64 *acc,u32 axis);
//设置各关节的减速比，demoninator:减速比分母，numerator:减速比分子，axis:关节号1-6  依次代表:J1-J6
extern int CTRL_SetReductionRate(s64 demoninator, s64 numerator, u32 axis);
//获取各关节的减速比，rate:减速比axis:关节号1-6  依次代表:J1-J6
extern int CTRL_GetReductionRate(s64 *demoninator, s64 *numerator,u32 axis);

//设置5/6关节的耦合比
extern int CTRL_SetCoupleRate(s64 rate);

//读取5/6关节的耦合比
extern int CTRL_GetCoupleRate(s64 *rate);
 

extern int CTRL_MotionPause();
extern int CTRL_MotionResume();




#ifdef __cplusplus
}
#endif	
	

#endif

