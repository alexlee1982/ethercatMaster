#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "motionCommandType.h"
#include "motionInterfaceBuffer.h"

#include "motionParameterType.h"
#include "motionConfig.h"
#include "motionFeedback.h"
#include "posemath.h"
#include "motionIner.h"






#ifdef __cplusplus
extern "C" {
#endif

extern int CTRL_Init(long us);				//初始化，us 毫秒为单位的调用周期
extern int CTRL_Callback(long us);			//us暂时没处理
extern void CTRL_Exit(void);				//运动模块卸载函数
extern int CTRL_GetTCPInUserSpace(PointPose *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool);
extern int CTRL_GetTCPInUserSpaceInMatrix(PmHomogeneous *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool);
extern int CTRL_GetTCPInJoint(JointPoint *joint,PointPose *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool);

extern int CTRL_GetTCPInJointInMatrix(JointPoint *joint,JointPoint *preJoint,PmHomogeneous *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool);



#ifdef __cplusplus
}
#endif	
	

#endif

