#ifndef MOTION_FEEDBACK_H
#define MOTION_FEEDBACK_H
#include "Common.h"
#include "motionInterfaceBuffer.h"



typedef struct
{

u32 motionState; 				//运动命令执行状态: 
							//COMMAND_DONE 完成，COMMAND_EXEC ，正在执行，COMMAND_ERROR :执行出错
u16 commandStatus;   //宏
f64 distance_was_gone;		//本条指令已经走完的距离
f64 distance_to_go;   			//本条指令还没走完的距离
u16 onSoftLimit;				//当前处于软限位上	          
u16 queueFull;				//运动队列是否为满
JointPoint jointCmd;			//关节命令值
JointPoint jointFeedback;		//关节反馈值
PointPose positionCmd;
PointPose usrCmd;
s64 servoEncoderValue[6];
s64 servoEncoderDestination[6];
u32 enable[6];

u32 linkError;					//总线连接错误

u64 errCode;
u32 lineNumber;

UserCoordianteInformation user;		//当前使用的用户坐标系的信息
ToolCoordianteInformation tool;		//当前使用的工具坐标系的信息
u16 socketConnectError;
}MotionFeedback;



#endif
