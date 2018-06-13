#include <stdio.h>
#include <string.h>
#include <math.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <signal.h>

#include <stdlib.h>
#include <unistd.h>
#include <error.h>

//#include "contrller.h"
#include "motionInterfaceBuffer.h"
#include "motionParameterType.h"
#include "motionParameter.h"
#include "motionIner.h"
#include "motionConfig.h"
#include "motionCommandType.h"
#include "Common.h"
#include "posemath.h"
#include "kinematics.h"
#include "motionFeedback.h"
#include "rtos.h"
//#include "MotionApi.h"
#include "tp.h"
#include "motionErrorType.h"
#define AROUND_ZERO 0.00001

#ifndef MOTION_COMMAND_KEY
#define MOTION_COMMAND_KEY 	2001
#endif
#ifndef MOTION_FEEDBACK_KEY
#define MOTION_FEEDBACK_KEY	2020
#endif
#ifndef MOTION_CONFIG_KEY
#define MOTION_CONFIG_KEY	2030
#endif
#ifndef MOTION_STATUS_KEY
#define MOTION_STATUS_KEY	2040
#endif
#ifndef MOTION_JOINTS_KEY
#define MOTION_JOINTS_KEY	2050
#endif
#ifndef MOTION_DEBUG_KEY
#define MOTION_DEBUG_KEY	2060
#endif

MotionConfig *emcmotConfig ;
MotionCommandBuffer *commandShmem ;
VRobotMotionCommand  *motCmd ;
MotionCommandQueue *interpList ;
VRobotMotionCommand *emcmotCommand;
MotionFeedback *motionFb;
MotionJointParameter *joints;
MotionStatus *emcmotStatus;
MotionDebug *emcmotDebug;

t_lock Mutex;


int commandShmID,statusShmID,configShmID,feedbackShmID,jointShmID,debugShmID;

 double PUMA_A1;
 double PUMA_A2;
 double PUMA_A3;
 double PUMA_D3;
 double PUMA_D4;
 double PUMA_D6;

void matrixToRpy(PmRotationMatrix rot, PmRpy *rpy)
{
	double  T[3][3];
	double Pitch;
	double Roll;
	double Yaw;
	T[0][0] = rot.x.x;
	T[0][1] = rot.x.y;
	T[0][2] = rot.x.z;

	T[1][0] = rot.y.x;
	T[1][1] = rot.y.y;
	T[1][2] = rot.y.z;

	T[2][0] = rot.z.x;
	T[2][1] = rot.z.y;
	T[2][2] = rot.z.z;

	Pitch = asin(-T[2][0]);
	Roll = atan(T[2][1] / T[2][2]);
	Yaw = atan(T[1][0] / T[0][0]);

	if (T[2][2]<0)
	{
		if (Roll < 0)
		{
			Roll = Roll + 3.1416;
		}
		else
		{
			Roll = Roll - 3.1416;
		}
	}

	if (T[0][0]<0)
	{
		if (T[1][0]>0)
		{
			Yaw = Yaw + 3.1416;
		}
		else
		{
			Yaw = Yaw - 3.1416;
		}
	}

	rpy->p = Pitch;
	rpy->r = Roll;
	rpy->y = Yaw;


	return;
}

void rpyToMatrix(PmRpy rpy,PmRotationMatrix *rot)
{
	double theta_z, theta_y, theta_x;

	double cos_z_2;
	double cos_y_2;
	double cos_x_2;

	double sin_z_2;
	double sin_y_2;
	double sin_x_2;

	double Q[4];
	double T[3][3];

	theta_z = rpy.r;
	theta_y = rpy.y;
	theta_x = rpy.p;


	theta_z = theta_z*PM_PI / 180;
	theta_y = theta_y*PM_PI / 180;
	theta_x = theta_x*PM_PI / 180;

	cos_z_2 = cos(0.5*theta_z);
	cos_y_2 = cos(0.5*theta_y);
	cos_x_2 = cos(0.5*theta_x);

	sin_z_2 = sin(0.5*theta_z);
	sin_y_2 = sin(0.5*theta_y);
	sin_x_2 = sin(0.5*theta_x);

	Q[0] = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
	Q[1] = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
	Q[2] = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
	Q[3] = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;

	T[0][0] = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
	T[0][1] = 2 * (Q[1] * Q[2] - Q[0] * Q[3]);
	T[0][2] = 2 * (Q[1] * Q[3] + Q[0] * Q[2]);

	T[1][0] = 2 * (Q[1] * Q[2] + Q[0] * Q[3]);
	T[1][1] = Q[0] * Q[0] - Q[1] * Q[1] + Q[2] * Q[2] - Q[3] * Q[3];
	T[1][2] = 2 * (Q[2] * Q[3] - Q[0] * Q[1]);

	T[2][0] = 2 * (Q[1] * Q[3] - Q[0] * Q[2]);
	T[2][1] = 2 * (Q[2] * Q[3] + Q[0] * Q[1]);
	T[2][2] = Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];

	rot->x.x = T[0][0];
	rot->x.y = T[0][1];
	rot->x.z = T[0][2];

	rot->y.x = T[1][0];
	rot->y.y = T[1][1];
	rot->y.z = T[1][2];

	rot->z.x = T[2][0];
	rot->z.y = T[2][1];
	rot->z.z = T[2][2];

	return;
}

int CTRL_SetOriginOffset(s64 offset,u32 axis)
{
	
	if(1==emcmotStatus->enble)
	{
		printf("can not change the origin offset while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_ORGOFFSET_WHILE_ENABLED_TYPE;
	}
	else
	{
		if(axis>6||axis<1)
		{
			printf("input axis number error axis=%d \n",axis);		
			return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
		}
		else
		{
			emcmotConfig->encodeOriginOffset[axis-1]=offset;
			printf("emcmotConfig->encodeOriginOffset[%d]=%lld \r\n",axis-1,emcmotConfig->encodeOriginOffset[axis-1]);
		}
	}
	return 0;
}

int CTRL_GetOriginOffset(s64 * offset,u32 axis)
{
	
	if(axis>6||axis<1)
	{
		printf("input axis number error axis=%d \n",axis);		
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	else
	{
		*offset=emcmotConfig->encodeOriginOffset[axis-1];
	}
	return 0;
}

int CTRL_SetPUMA(f64 length,u32 num)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the puma while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_PUMA_WHILE_ENABLED_TYPE;
	}
	else
	{
		switch(num)
		{
			case 1:
	 			emcmotConfig->PUMA_A[0] = length ;
				break;
			case 2:
				emcmotConfig->PUMA_A[1] = length ;
				break;
			case 3:
				emcmotConfig->PUMA_A[2] = length;
				break;
			case 4:
				emcmotConfig->PUMA_A[3] = length;
				break;
			case 5:
 				emcmotConfig->PUMA_A[4] = length;	
				break;
			default:
				printf("can not edit puma that is not exist \r\n");
				return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
				break;
		}
	}
	return 0;
}

int CTRL_GetPUMA(f64 *length,u32 num)
{
	switch(num)
	{
		case 1:
	 		*length=emcmotConfig->PUMA_A[0] ;
			break;
		case 2:
			*length=emcmotConfig->PUMA_A[1]  ;
			break;
		case 3:
			*length=emcmotConfig->PUMA_A[2] ;
			break;
		case 4:
			*length=emcmotConfig->PUMA_A[3] ;
			break;
		case 5:
 			*length=emcmotConfig->PUMA_A[4] ;	
			break;
		default:
			printf("can not get puma that is not exist \r\n");
			return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
			break;
	}	
	return 0;
}

int CTRL_SetTeachLinearVelocity(f64 velocity)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the Teach Linear Velocity while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_TEACHV_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->teachLinearV=velocity;
	return 0;
}

int CTRL_GetTeachLinearVelocity(f64 *velocity)
{
	*velocity=emcmotConfig->teachLinearV;
	return 0;
}

int CTRL_SetLinearMaxVelocity(f64 velocity)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum Linear Velocity while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_MAX_LINEARV_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->linearMaxV=velocity;
	return 0;

}
 int CTRL_GetLinearMaxVelocity(f64 *velocity)
{
	*velocity=emcmotConfig->linearMaxV;
	return 0;
}

int CTRL_SetLinearMoveAcc(f64 acc)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum Linear acceleration while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_MAX_LINEARACC_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->linearMoveAcc=acc;
	return 0;
}

 int CTRL_GetLinearMoveAcc(f64 *acc)
{
	*acc=emcmotConfig->linearMoveAcc;
	return 0;
}

int CTRL_SetRotateMoveVelocity(f64 velocity)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum rotation velocity while servo is enabled \n");
		return MOTION_ERROR_CANNOT_MODIFY_MAX_ROTATEV_WHILE_ENABLED_TYPE;
	}
	emcmotConfig->rotateMoveV=velocity;
	return 0;
}

 int CTRL_GetRotateMoveVelocity(f64 *velocity)
{
	*velocity=emcmotConfig->rotateMoveV;
	return 0;
}
int CTRL_SetRotateMoveAcc(f64 acc)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the  Maximum rotation move acceleration while servo is enabled \n");
		return -1;
	}
	emcmotConfig->rotateMoveAcc=acc;
	return 0;
}

int CTRL_GetRotateMoveAcc(f64 *acc)
{
	*acc=emcmotConfig->rotateMoveAcc;
	return 0;
}
int CTRL_SetLinearPositiveLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->linearPosLimit[axis-1]=limit;
		joints[axis-1].max_pos_limit=limit;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetLinearPositiveLimit(f64 *limit,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->linearPosLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetLinearNegativeLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->linearNegaLimit[axis-1]=limit;
		joints[axis-1].min_pos_limit=limit;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetLinearNegativeLimit(f64 *limit,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->linearNegaLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}



int CTRL_SetJointPositiveLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointPosLimit[axis-1]=limit;
		joints[axis-1].max_jog_limit=limit;

	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointPositiveLimit(f64 *limit,u32 axis)
{

	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->jointPosLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetJointNegativeLimit(f64 limit,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change the soft limit while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_SOFT_LIMIT_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointNegaLimit[axis-1]=limit;
		joints[axis-1].min_jog_limit=limit;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointNegativeLimit(f64 *limit,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*limit=emcmotConfig->jointNegaLimit[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}


int CTRL_SetJointMaxVelocity(f64 velocity,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change set joint velocity while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_JOINT_VELOCITY_WHILE_ENABLED_TYPE;
	}

	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointMoveVel[axis-1]=velocity;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointMaxVelocity(f64 *velocity,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*velocity=emcmotConfig->jointMoveVel[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetJointMaxAcc(f64 acc,u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change set joint ACC while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_JOINT_ACC_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->jointMoveAcc[axis-1]=acc;
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetJointMaxAcc(f64 *acc,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*acc=emcmotConfig->jointMoveAcc[axis-1];
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}



int CTRL_SetReductionRate(s64 demoninator, s64 numerator, u32 axis)
{
	if(1==emcmotStatus->enble)
	{
		printf("can not change reduction rate while servo is enabled \n");		
		return MOTION_ERROR_CANNOT_MODIFY_RATIO_WHILE_ENABLED_TYPE;
	}
	if(axis>=1&&axis<=6)
	{
		emcmotConfig->mechTransRatio[axis-1]=(f64)numerator/(f64)demoninator;
		
		emcmotConfig->mechTransDenominator[axis-1]=demoninator;
		emcmotConfig->mechTransNumerator[axis-1]=numerator;
		printf("aixs:%d: Denominator=%lld\r\n，Numerator=%lld ",axis,demoninator,numerator);
		
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}
int CTRL_GetReductionRate(s64 *demoninator, s64 *numerator,u32 axis)
{
	if(axis>=1&&axis<=6)
	{
		*demoninator=emcmotConfig->mechTransDenominator[axis-1];
		*numerator=emcmotConfig->mechTransNumerator[axis-1];	
	}
	else
	{
		printf("int put axis error axis=%d \r\n",axis);
		return MOTION_ERROR_INPUT_AXIS_NUMBER_ERROR_TYPE;
	}
	return 0;
}

int CTRL_SetCoupleRate(s64 rate)
{
	emcmotConfig->coupleRate=rate;
	return 0;
}

int CTRL_GetCoupleRate(s64 *rate)
{
	*rate=emcmotConfig->coupleRate;
	return 0;
}



int CTRL_UserCalibration(ThreeJointPoints *jp, ToolCoordianteInformation *tool,UserCoordianteInformation *out)
{
	double tempJoints[6];
	PmCartesian origin,ox,oy;
	PmCartesian VecX,VecY,VecZ;
	RobotPose world;
	PmRotationMatrix  Ehom;
	PmQuaternion tempQuaternion;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	int i=0;
	double angle;
	PmRpy tempRpy;
	PmCartesian tcf;
	PmPose  orgT,EXT,EYT,TTool,TORG,TXT,TYT;
	PmEulerZyx tempZyx;
	PmRotationMatrix rotT;
	tempZyx.x=tool->rx*PM_PI/180.0;
	tempZyx.y=tool->ry*PM_PI/180.0;
	tempZyx.z=tool->rz*PM_PI/180.0;
	pmZyxQuatConvert(tempZyx,&tempQuaternion);
	TTool.rot=tempQuaternion;
	TTool.tran.x=tool->x;
	TTool.tran.y=tool->y;
	TTool.tran.z=tool->z;
	
	tempJoints[0]=jp->org.j1;
	tempJoints[1]=jp->org.j2;
	tempJoints[2]=jp->org.j3;
	tempJoints[3]=jp->org.j4;
	tempJoints[4]=jp->org.j5;
	tempJoints[5]=jp->org.j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	origin=world.tran;
	orgT.tran=world.tran;
	pmMatQuatConvert(Ehom,&tempQuaternion);
	orgT.rot=tempQuaternion;
	pmPosePoseMult(orgT,TTool,&TORG);

	tempJoints[0]=jp->ox.j1;
	tempJoints[1]=jp->ox.j2;
	tempJoints[2]=jp->ox.j3;
	tempJoints[3]=jp->ox.j4;
	tempJoints[4]=jp->ox.j5;
	tempJoints[5]=jp->ox.j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	ox=world.tran;
	EXT.tran=world.tran;
	pmMatQuatConvert(Ehom,&tempQuaternion);
	EXT.rot=tempQuaternion;
	pmPosePoseMult(EXT,TTool,&EXT);
	
	tempJoints[0]=jp->oy.j1;
	tempJoints[1]=jp->oy.j2;
	tempJoints[2]=jp->oy.j3;
	tempJoints[3]=jp->oy.j4;
	tempJoints[4]=jp->oy.j5;
	tempJoints[5]=jp->oy.j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	oy=world.tran;

	EYT.tran=world.tran;
	pmMatQuatConvert(Ehom,&tempQuaternion);
	EYT.rot=tempQuaternion;
	pmPosePoseMult(EYT,TTool,&TYT);
	

/*
	for(i=0;i<3;i++)
	{
		tempJoints[0]=jp[i].j1;
		tempJoints[1]=jp[i].j2;
		tempJoints[2]=jp[i].j3;
		tempJoints[3]=jp[i].j4;
		tempJoints[4]=jp[i].j5;
		tempJoints[5]=jp[i].j6;
		kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
		switch(i)
		{
		case 0:
			origin=world.tran;
			break;
		case 1:
			ox=world.tran;
			break;
		case 2:
			oy=world.tran;
			break;
		default:
			break;
		}
	}
	*/
	pmCartCartSub(ox,origin,&VecX);
	pmCartUnit(VecX,&VecX);
	pmCartCartSub(oy,origin,&oy);
	pmCartUnit(oy,&oy);
	pmCartCartDot(VecX,oy,&angle);
	if((1-angle)<AROUND_ZERO)
	{
		printf("userCalibaration failed \n");	
		return MOTION_ERROR_USR_CALIBARATION_FAILED_TYPE;
	}
	pmCartCartCross(VecX,oy,&VecZ);
	pmCartCartCross(VecZ, VecX,&VecY);


	PmHomogeneous hom;
	PmPose worldPose;



	Ehom.x=VecX;
	Ehom.y=VecY;
	Ehom.z=VecZ;

	

	printf("x.x=%f , x.y=%f ,x.z =%f \n", Ehom.x.x, Ehom.x.y, Ehom.x.z);
	printf("y.x=%f , y.y=%f ,y.z =%f \n", Ehom.y.x, Ehom.y.y, Ehom.y.z);
	printf("z.x=%f , z.y=%f ,z.z =%f \n", Ehom.z.x, Ehom.z.y, Ehom.z.z);

	//matrixToRpy(Ehom,&tempRpy);
	//PmQuaternion tempVec;
	////pmMatQuatConvert(Ehom,&tempVec);
	
	pmMatZyxConvert(Ehom, &tempZyx);
	//pmMatRpyConvert(Ehom,&tempRpy);
	out->x=TORG.tran.x;
	out->y=TORG.tran.y;
	out->z=TORG.tran.z;
	//out->a= tempRpy.p*180.0/PM_PI;
	//out->b= tempRpy.y*180.0/PM_PI;
	//out->c= tempRpy.r*180.0/PM_PI;
	out->a= tempZyx.x*180.0/PM_PI;
	out->b= tempZyx.y*180.0/PM_PI;
	out->c= tempZyx.z*180.0/PM_PI;
	printf("user calibration x=%f,y=%f,z=%f,a=%f, b=%f ,c=%f \n",out->x,out->y,out->z,out->a,out->b,out->c);

	//pmRpyQuatConvert(tempRpy, &worldPose.rot);
	//pmPoseHomConvert(worldPose, &hom);

	//Ehom = hom.rot;

	//pmQuatMatConvert(tempVec, &Ehom);
	//pmRpyMatConvert(tempRpy, &Ehom);


	//rpyToMatrix(tempRpy,&Ehom);
	pmZyxMatConvert(tempZyx,&Ehom);
	printf("x.x=%f , x.y=%f ,x.z =%f \n", Ehom.x.x, Ehom.x.y, Ehom.x.z);
	printf("y.x=%f , y.y=%f ,y.z =%f \n", Ehom.y.x, Ehom.y.y, Ehom.y.z);
	printf("z.x=%f , z.y=%f ,z.z =%f \n", Ehom.z.x, Ehom.z.y, Ehom.z.z);

	//matrixToRpy(Ehom, &tempRpy);

	//printf("user calibration x=%f,y=%f,z=%f,a=%f, b=%f ,c=%f \n", out->x, out->y, out->z, out->a, out->b, out->c);

	return 0;
}


/*
 this function shoud calculate the Tool coordinate information based on the position that was indicated by 
 the pointer jp.
 the jp shoud address at least 6 JointPoint variables, otherwise the outcome would be nonsense.
 the pointNumber shoud be 6 right now. In the futuer it should be able to be any integer that is greater than 4.
 And the value of the pointNumber should math the number of the JointPoint type array 
 that is input in this function.
At last, the coordinate information calculated would be put in the pointer out.
*/
 int CTRL_ToolCalibration(JointPoint *jp,int pointNumber,ToolCoordianteInformation *out)
{
	double tempJoints[6];
	PmCartesian descartesPoints[6],center,temp,PT;
	PmCartesian VecX,VecY,VecZ;
	RobotPose world;
	PmRotationMatrix  Ehom,Inv;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	double a11,a12,a13,a21,a22,a23,a31,a32,a33,b1,b2,b3,d,d1,d2,d3;
	double xt,yt,zt;
	int i=0;
	double angle;
	PmRpy tempRpy;
	PmEulerZyx tempZyx;
	PmRotationMatrix AE,AEInv,AT;
	if(pointNumber<6)
	{
		return -1;
	}
	
//get descarte coordinate positions
	for(i=0;i<6;i++)
	{
		tempJoints[0]=jp[i].j1;
		tempJoints[1]=jp[i].j2;
		tempJoints[2]=jp[i].j3;
		tempJoints[3]=jp[i].j4;
		tempJoints[4]=jp[i].j5;
		tempJoints[5]=jp[i].j6;	
		kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
		if(i==3)
		{
			AE=Ehom;
		}
		descartesPoints[i]=world.tran;
	}
	a11=2*(descartesPoints[1].x-descartesPoints[0].x); 
	a12=2*(descartesPoints[1].y-descartesPoints[0].y);
	a13=2*(descartesPoints[1].z-descartesPoints[0].z);  
	a21=2*(descartesPoints[2].x-descartesPoints[1].x);
	a22=2*(descartesPoints[2].y-descartesPoints[1].y);
	a23=2*(descartesPoints[2].z-descartesPoints[1].z);  
	a31=2*(descartesPoints[3].x-descartesPoints[2].x);
	a32=2*(descartesPoints[3].y-descartesPoints[2].y);
	a33=2*(descartesPoints[3].z-descartesPoints[2].z);  
	b1=descartesPoints[1].x*descartesPoints[1].x-descartesPoints[0].x*descartesPoints[0].x+descartesPoints[1].y*descartesPoints[1].y-descartesPoints[0].y*descartesPoints[0].y+descartesPoints[1].z*descartesPoints[1].z-descartesPoints[0].z*descartesPoints[0].z;  
	b2=descartesPoints[2].x*descartesPoints[2].x-descartesPoints[1].x*descartesPoints[1].x+descartesPoints[2].y*descartesPoints[2].y-descartesPoints[1].y*descartesPoints[1].y+descartesPoints[2].z*descartesPoints[2].z-descartesPoints[1].z*descartesPoints[1].z;   
	b3=descartesPoints[3].x*descartesPoints[3].x-descartesPoints[2].x*descartesPoints[2].x+descartesPoints[3].y*descartesPoints[3].y-descartesPoints[2].y*descartesPoints[2].y+descartesPoints[3].z*descartesPoints[3].z-descartesPoints[2].z*descartesPoints[2].z;     
	d=a11*a22*a33+a12*a23*a31+a13*a21*a32-a11*a23*a32-a12*a21*a33-a13*a22*a31;  
	d1=b1*a22*a33+a12*a23*b3+a13*b2*a32-b1*a23*a32-a12*b2*a33-a13*a22*b3;  
	d2=a11*b2*a33+b1*a23*a31+a13*a21*b3-a11*a23*b3-b1*a21*a33-a13*b2*a31;  
	d3=a11*a22*b3+a12*b2*a31+b1*a21*a32-a11*b2*a32-a12*a21*b3-b1*a22*a31; 
	if(fabs(d)>AROUND_ZERO)
	{
		xt=d1/d;  
		yt=d2/d;  
		zt=d3/d;  
	d=Ehom.x.x*Ehom.y.y*Ehom.z.z+Ehom.x.y*Ehom.y.z*Ehom.z.x+Ehom.x.z*Ehom.y.x*Ehom.z.y
		-Ehom.x.x*Ehom.y.z*Ehom.z.y-Ehom.x.y*Ehom.y.x*Ehom.z.z-Ehom.x.z*Ehom.y.y*Ehom.z.x;
		center.x=xt/d;
		center.y=yt/d;
		center.z=zt/d;
		pmCartCartSub(center,descartesPoints[3],&temp);
		pmMatInv(AE, &Inv);
		pmMatCartMult(Inv,temp,&PT);
		out->x=PT.x;
		out->y=PT.y;
		out->z=PT.z;
	}
//	emcmotDebug->robot_tcp.x = out->x;
//	emcmotDebug->robot_tcp.y = out->y;
//	emcmotDebug->robot_tcp.z = out->z;
	



//get the vectors for three axises
//	descartesPoints pmCartCartCross
	pmCartCartSub(descartesPoints[4],descartesPoints[3],&VecX);
	pmCartUnit(VecX,&VecX);

	pmCartCartSub(descartesPoints[5],descartesPoints[3],&VecY);
	pmCartUnit(VecY,&VecY);

	pmCartCartDot(VecX, VecY, &angle);
	if((1-angle)<AROUND_ZERO)
	{		
		return MOTION_ERROR_TOOL_CALIBARATION_FAILED_TYPE;
	}
	pmCartCartCross(VecX, VecY,&VecZ);
	pmCartUnit(VecZ,&VecZ);

//	pmCartCartCross(VecX, VecZ,&VecY);
//	pmCartUnit(VecY,&VecY);

	Ehom.x=VecX;
	Ehom.y=VecY;
	Ehom.z=VecZ;
	pmMatInv(AE,&AEInv);
	pmMatMatMult(AEInv, Ehom, &AT);
	pmMatZyxConvert(AT,&tempZyx);
	out->rx= tempZyx.x*180.0/PM_PI;
	out->ry= tempZyx.y*180.0/PM_PI;
	out->rz= tempZyx.z*180.0/PM_PI;
	
	return 0;

}

/*
 void CTRL_SetSocket(char * IP,int port ,int cycleTime)
{
	
	motConfigBuf.cycleTime = cycleTime;
	motConfigBuf.socketPortNumber = port;
	strcpy(motConfigBuf.socketIP, IP);

}
*/
 int CTRL_GetMotionStatus(MotionFeedback * fb)
{
	long int ret = 0;
	//ret = LOCK(Mutex);
	LOCK(Mutex);
	if (1)	// (0==ret)
	{	
		//printf("B");
		*fb = *motionFb;
		if(motionFb->motionState==COMMAND_ERROR||motionFb->motionState==COMMAND_EXEC)
		{
			fb->motionState=motionFb->motionState;
		}
		else if(interpList->start!=interpList->end)
		{
			fb->motionState=COMMAND_EXEC;
			printf("temporay display \r\n");
		}
		else
		{
			fb->motionState=COMMAND_DONE;
		}
		UNLOCK(Mutex); 
	}
	else
	{
		printf("D");
		return ret;
	}
	return 0;
}



 int CTRL_ServoEnable(int enable)
{
	int i=0;
	int ret=0;
	if(enable!=0&&enable!=1)
	{
		//reportErrorType(MOTION_ERROR_INABLE_INPUT_ERROR_TYPE);
		printf("input enable error %d \n", enable);
		return MOTION_ERROR_INPUT_ENABLE_VALUE_ERROR_TYPE;
	}
	//printf("enter the enable function \n");
	for(i=0;i<6;i++)
	{
		//if(g_servo[i]!=NULL)
		if(1)
		{
			if(enable!=emcmotStatus->enble)
			{
				//ret=Servo_Enable(g_servo[i],enable);
				//printf("the return value of servo enable is %d ret\n",ret);
				if(0==ret)
				{
					//continue;
				}
				else
				{
					//printf("failed to enable the servo ret=%d\n",ret);
					return MOTION_ERROR_MOTION_ENABLE_FAILED_TYPE;
					//return ret;
				}		
			}
		}
		else
		{
			printf("can not get access to all servo\n");
			return MOTION_ERROR_CANNOT_ACCESS_SERVO_FAILED_TYPE;
		}
	}

	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_SERVO_ENABLE_TYPE;
	if (enable != 0 && enable != 1)
	{
		printf("input enable error %d \n", enable);
		return -1;
	}
	motCmd->motionCmdParameter.svEable.enable = enable;

	motCmd->tail = motCmd->head;
	return 0;

}

 int CTRL_ServoReset(void)
{
	int i=0;
	int ret=0;
	for(i=0;i<6;i++)
	{
		//if(g_servo[i]!=NULL)
		if(1)
		{
			//ret=Servo_Reset(g_servo[i]);
			//printf("the return value of servo enable is %d ret\n",ret);
			if(0==ret)
			{
				continue;
			}
			else
			{
				printf("failed to reset the servo ret=%d\n",ret);
				return MOTION_ERROR_MOTION_RESET_FAILED_TYPE;
			}		
		}
		else
		{
			printf("can not get access to all servo\n");
			return MOTION_ERROR_CANNOT_ACCESS_SERVO_FAILED_TYPE;	//-1;
		}
	}
	emcmotStatus->linkErr=0;

	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_SERVO_RESET_TYPE;
	
	motCmd->tail = motCmd->head;
	return 0;	// 0;

	
}



 void CTRL_MovementStop()
{
	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_MOVEMENT_STOP_TYPE;
	motCmd->tail = motCmd->head;
	return ;
}


 int CTRL_AddSingleAxisMove(SingleAxisMove *singleMove)
{
	
	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_SINGLEAXISMOVE_TYPE;
	if (singleMove->vel>1.0 || singleMove->vel<0.001)
	{
		//reportErrorType(MOTION_ERROR_SINGLE_AXIS_MOVE_VELOCITY_OFR_TYPE);
		printf("input velocity out of range %f \n", singleMove->vel);
		return MOTION_ERROR_SINGLE_AXIS_MOVE_VELOCITY_OFR_TYPE;
	}
	motCmd->motionCmdParameter.singleMove.vel = singleMove->vel;
	if (singleMove->axis>6 || singleMove->axis<1)
	{
		//reportErrorType(MOTION_ERROR_SINGLE_AXIS_MOVE_AXIS_OFR_TYPE);
		printf("axis number not in the rang axis=%d\n", singleMove->axis);
		return MOTION_ERROR_SINGLE_AXIS_MOVE_AXIS_OFR_TYPE;
	}
	motCmd->motionCmdParameter.singleMove.axis = singleMove->axis;
	if (MOVE_DIR_POSITIVE != singleMove->direction&&MOVE_DIR_NEGATIVE != singleMove->direction)
	{
		//reportErrorType(MOTION_ERROR_SINGLE_AXIS_MOVE_DIR_ERROR_TYPE);
		printf("direction error %d \n", singleMove->direction);
		return MOTION_ERROR_SINGLE_AXIS_MOVE_DIR_ERROR_TYPE;
	}
	//printf("add single axis move\n");
	motCmd->motionCmdParameter.singleMove = *singleMove;
	//add coordinate information later
	motCmd->tail = motCmd->head;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}
 int CTRL_AddJointMove(JointMoveInformation *jointMove,u32 lineNumber,u32 fileNumber)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE );
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	//interpList->end++;
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_JOINTMOVE_TYPE;
	if (jointMove->vJ<0.001 || jointMove->vJ>1.0)
	{
		//reportErrorType(MOTION_ERROR_JOINT_MOVE_VELOCITY_ERROR_TYPE);
		printf("joint move velocity scale out of range %f", jointMove->vJ);
		return MOTION_ERROR_JOINT_MOVE_VELOCITY_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movJ.vJ = jointMove->vJ;
	}
	if (jointMove->acc<0.0001 || jointMove->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_JOINT_MOVE_ACC_ERROR_TYPE);
		printf("joint move acceleration scale out of range %f", jointMove->acc);
		return MOTION_ERROR_JOINT_MOVE_ACC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movJ.acc = jointMove->acc;
	}
	if (jointMove->dec<0.0001 || jointMove->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_JOINT_MOVE_DEC_ERROR_TYPE);
		printf("joint move acceleration scale out of range %f", jointMove->acc);
		return MOTION_ERROR_JOINT_MOVE_DEC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movJ.dec = jointMove->dec;
	}
	command->motionCmdParameter.movJ.endPoint = jointMove->endPoint;

	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}



 int CTRL_AddLinearMove(LinearMoveInformation *movl,u32 lineNumber,u32 fileNumber)
		
 {
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	//interpList->end++;
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_LINEARMOVE_TYPE;
	if(movl->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE);
		printf("linear move velocity out of range %f", movl->acc);
		return MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE;
	}
	if(movl->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE);
		printf("linear move rotate velocity out of range %f", movl->acc);
		return MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movl->acc<0.0001 || movl->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movl->acc);
		return MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movL.acc = movl->acc;
	}
	if (movl->dec<0.0001 || movl->dec>1.0)
	{
		
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movl->acc);
		return MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movL.dec = movl->dec;
	}


	command->motionCmdParameter.movL.vL = movl->vL;
	command->motionCmdParameter.movL.endPoint = movl->endPoint;

	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}

 int CTRL_AddCircularMove(CircularMoveInformation *movC,u32 lineNumber,u32 fileNumber)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	//interpList->end++;
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_CIRCULARMOVE_TYPE;

	if(movC->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE);
		printf("circular move velocity out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE;
	}
	if(movC->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE);
		printf("circular move rotate velocity out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movC->acc<0.0001 || movC->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movC.acc = movC->acc;
	}
	if (movC->dec<0.0001 || movC->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE;
	}
	else
	{
		command->motionCmdParameter.movC.dec = movC->dec;
	}


	command->motionCmdParameter.movC.vL = movC->vL;
	command->motionCmdParameter.movC.endPoint[0] = movC->endPoint[0];
	command->motionCmdParameter.movC.endPoint[1] = movC->endPoint[1];
	command->motionCmdParameter.movC.endPoint[2] = movC->endPoint[2];
	command->motionCmdParameter.movC = *movC;
	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}
#ifdef TEST
 int CTRL_AddDescartesLinearMove(double x,double y,double z,int type)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		printf("command buffer is full \n");
		return -1;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->type = MOTION_COMMAND_DESCARTES_LINEARMOVE_TYPE;
	switch(type)
	{
		case 0:
			command->motionCmdParameter.movDL.free=1;
			break;
		case 1:
			command->motionCmdParameter.movDL.rot.x.x=0;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=1;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=1;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		case 2:
			command->motionCmdParameter.movDL.rot.x.x=0;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=-1;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=-1;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		case 3:
			command->motionCmdParameter.movDL.rot.x.x=-1;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=1;
			break;
		case 4:
			command->motionCmdParameter.movDL.rot.x.x=1;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=-1;
			command->motionCmdParameter.movDL.rot.y.z=0;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=0;
			command->motionCmdParameter.movDL.rot.z.z=-1;
			break;
		case 5:
			command->motionCmdParameter.movDL.rot.x.x=1;
			command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=0;
			command->motionCmdParameter.movDL.rot.y.z=-1;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=1;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		case 6:
			command->motionCmdParameter.movDL.rot.x.x=1;
				command->motionCmdParameter.movDL.rot.x.y=0;
			command->motionCmdParameter.movDL.rot.x.z=0;

			command->motionCmdParameter.movDL.rot.y.x=0;
			command->motionCmdParameter.movDL.rot.y.y=0;
			command->motionCmdParameter.movDL.rot.y.z=1;
			
			command->motionCmdParameter.movDL.rot.z.x=0;
			command->motionCmdParameter.movDL.rot.z.y=-1;
			command->motionCmdParameter.movDL.rot.z.z=0;
			break;
		default:
			printf("can not deal with this kind of tcf type %d \n",type);
			break;
	}
	command->motionCmdParameter.movDL.vL=100;
	command->motionCmdParameter.movDL.vR=50;
	command->motionCmdParameter.movDL.endPoint.x=x;
	command->motionCmdParameter.movDL.endPoint.y=y;
	command->motionCmdParameter.movDL.endPoint.z=z;

	command->tail = command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}
 int CTRL_AddDescartesCircularMove(double x1,double y1,double z1,double x2,double y2,double z2,double x3,double y3,double z3)
{
	int i=0;
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		printf("command buffer is full \n");
		return -1;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->type = MOTION_COMMAND_DESCARTES_CIRCULARMOVE_TYPE;
	//command->motionCmdParameter.movDC.endPoint[1]=movDC.endPoint[1];
	//command->motionCmdParameter.movDC.endPoint[2]=movDC.endPoint[2];
	command->motionCmdParameter.movDC.free=1;
	command->motionCmdParameter.movDC.endPoint[0].x=x1;
	command->motionCmdParameter.movDC.endPoint[0].y=y1;
	command->motionCmdParameter.movDC.endPoint[0].z=z1;

	command->motionCmdParameter.movDC.endPoint[1].x=x2;
	command->motionCmdParameter.movDC.endPoint[1].y=y2;
	command->motionCmdParameter.movDC.endPoint[1].z=z2;

	command->motionCmdParameter.movDC.endPoint[2].x=x3;
	command->motionCmdParameter.movDC.endPoint[2].y=y3;
	command->motionCmdParameter.movDC.endPoint[2].z=z3;

	for(i=0;i<3;i++)
		{
		//command->motionCmdParameter.movDC.rot[i].x=0.0;
		//command->motionCmdParameter.movDC.rot[i].y=0.0;
		//command->motionCmdParameter.movDC.rot[i].z=0.0;
		}	
	command->motionCmdParameter.movDC.vL=100;
	command->motionCmdParameter.movDC.vR=100;
	command->motionCmdParameter.movDC.acc=1.0;
	command->motionCmdParameter.movDC.dec=1.0;
	command->tail=command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}
#else
 int CTRL_AddDescartesLinearMove(LinearDescartesMoveInformation *movDL, u32 lineNumber, u32 fileNumber)
{
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	if(movDL->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE);
		printf("linear move velocity out of range %f", movDL->acc);
		return MOTION_ERROR_LINEAR_VELOCITY_OFR_TYPE;
	}
	if(movDL->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE);
		printf("linear move rotate velocity out of range %f", movDL->acc);
		return MOTION_ERROR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movDL->acc<0.0001 || movDL->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movDL->acc);
		return MOTION_ERROR_LINEAR_MOVE_ACC_ERROR_TYPE;
	}	
	if (movDL->dec<0.0001 || movDL->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE);
		printf("linear move acceleration scale out of range %f", movDL->acc);
		return MOTION_ERROR_LINEAR_MOVE_DEC_ERROR_TYPE;
	}
	
	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	command->type = MOTION_COMMAND_DESCARTES_LINEARMOVE_TYPE;
	command->motionCmdParameter.movDL=*movDL;

	command->tail=command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}


 int CTRL_AddDescartesCircularMove(CircularDescartesMoveInformation *movDC,u32 lineNumber,u32 fileNumber)
{
	
	int i=0;
	Command_PTR command;
	if ((interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE == interpList->start)
	{
		//reportErrorType(MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE);
		printf("command buffer is full \n");
		return MOTION_ERROR_COMMAND_BUFFER_FULL_TYPE;
	}
	if (interpList->end>MOTION_COMMAD_QUEUE_SIZE)
	{
		interpList->end = interpList->end%MOTION_COMMAD_QUEUE_SIZE;
	}
	else if (interpList->end<0)
	{
		interpList->end = 0;
	}
	if(movDC->vL<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE);
		printf("circular move velocity out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_VELOCITY_OFR_TYPE;
	}
	if(movDC->vR<0.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE);
		printf("circular move rotate velocity out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_ROTATION_VELOCITY_OFR_TYPE;
	}
	if (movDC->acc<0.0001 || movDC->acc>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_ACC_ERROR_TYPE;
	}	
	if (movDC->dec<0.0001 || movDC->dec>1.0)
	{
		//reportErrorType(MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE);
		printf("circular move acceleration scale out of range %f", movDC->acc);
		return MOTION_ERROR_CIRCULAR_MOVE_DEC_ERROR_TYPE;
	}

	command = &(interpList->motionCmd[interpList->end]);
	command->head++;
	command->type = MOTION_COMMAND_DESCARTES_CIRCULARMOVE_TYPE;
	command->serialNumber++;
	command->currentLineNumber=lineNumber;
	command->currentFileNumber=fileNumber;
	//command->motionCmdParameter.movDC.endPoint[1]=movDC.endPoint[1];
	//command->motionCmdParameter.movDC.endPoint[2]=movDC.endPoint[2];
	command->motionCmdParameter.movDC=*movDC;
	//command->motionCmdParameter.movDC.free=movDC->free;
	//for(i=0;i<3;i++)
	//{
	//	command->motionCmdParameter.movDC.endPoint[i]=movDC->endPoint[i];
	//	command->motionCmdParameter.movDC.rot[i]=movDC->rot[i];
	//}
	//command->motionCmdParameter.movDC.vL=movDC->vL;
	//command->motionCmdParameter.movDC.vR=movDC->vR;
	
	//command->motionCmdParameter.movDC.acc=movDC->acc;
	//command->motionCmdParameter.movDC.dec=movDC->dec;
	
	command->tail=command->head;
	interpList->end = (interpList->end + 1) % MOTION_COMMAD_QUEUE_SIZE;
	motionFb->motionState=COMMAND_EXEC;
	return 0;
}

#endif
int CTRL_MotionPause()
{
	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_TRAJ_PAUSE_TYPE;
	motCmd->tail = motCmd->head;
	return ;

}
int CTRL_MotionResume()
{
	motCmd->head++;
	motCmd->serialNumber++;
	motCmd->type = MOTION_COMMAND_TRAJ_RESUME_TYPE;
	motCmd->tail = motCmd->head;
	return ;
}

int CTRL_USR_Init(void)
{
	/*     命令队列       */

	commandShmID=shmget((key_t)MOTION_COMMAND_KEY,sizeof(MotionCommandBuffer),IPC_CREAT);
	if(commandShmID==-1)
	{
		perror("command shmget:");
		exit(1);
	}
	commandShmem = (MotionCommandBuffer*)shmat(commandShmID,NULL,0);
	if((int)commandShmem == -1)
	{
		perror("command shmat:");
		shmdt(commandShmem);
		if(shmctl(commandShmID,IPC_RMID,0)<0)
		{
			perror("shmctl:");
		}
		exit(1);
	}  

	/*     状态队列      */

	statusShmID=shmget((key_t)MOTION_STATUS_KEY,sizeof(MotionStatus),IPC_CREAT);
	if(statusShmID==-1)
	{
		perror("status shmget:");
		exit(1);
	}
	emcmotStatus = (MotionStatus*)shmat(statusShmID,NULL,0);
	if((int)emcmotStatus == -1)
	{
		perror("status shmat:");
		shmdt(emcmotStatus);
		if(shmctl(statusShmID,IPC_RMID,0)<0)
		{
			perror("shmctl:");
		}
		exit(1);
	}

	/*     参数配置    */
	
	configShmID=shmget((key_t)MOTION_CONFIG_KEY,sizeof(MotionConfig),IPC_CREAT);
	if(configShmID==-1)
	{
		perror("config shmget:");
		exit(1);
	}
	emcmotConfig = (MotionConfig*)shmat(configShmID,NULL,0);
	if((int)emcmotConfig == -1)
	{
		perror("config shmat:");
		shmdt(emcmotConfig);
		if(shmctl(configShmID,IPC_RMID,0)<0)
		{
			perror("shmctl:");
		}
		exit(1);
	}


	/*     反馈共享内存   */
	
	feedbackShmID=shmget((key_t)MOTION_FEEDBACK_KEY,sizeof(MotionFeedback),IPC_CREAT);
	if(feedbackShmID==-1)
	{
		perror("feedback shmget:");
		exit(1);
	}
	motionFb = (MotionFeedback*)shmat(feedbackShmID,NULL,0);
	if((int)motionFb == -1)
	{
		perror("feedback shmat:");
		shmdt(motionFb);
		if(shmctl(feedbackShmID,IPC_RMID,0)<0)
		{
			perror("shmctl:");
		}
		exit(1);
	}


	/*     关节参数共享内存   */
	
	jointShmID=shmget((key_t)MOTION_JOINTS_KEY,((sizeof(MotionJointParameter))*MAX_JOINTS),IPC_CREAT);
	if(jointShmID==-1)
	{
		perror("config shmget:");
		exit(1);
	}
	joints = (MotionJointParameter*)shmat(jointShmID,NULL,0);
	if((int)joints == -1)
	{
		perror("config shmat:");
		shmdt(joints);
		if(shmctl(jointShmID,IPC_RMID,0)<0)
		{
			perror("shmctl:");
		}
		exit(1);
	}

	/*     debug共享内存   */
	
	debugShmID=shmget((key_t)MOTION_DEBUG_KEY,sizeof(MotionDebug),IPC_CREAT);
	if(debugShmID==-1)
	{
		perror("debug shmget:");
		exit(1);
	}
	emcmotDebug = (MotionDebug*)shmat(debugShmID,NULL,0);
	if((int)emcmotDebug == -1)
	{
		perror("debug shmat:");
		shmdt(emcmotDebug);
		if(shmctl(debugShmID,IPC_RMID,0)<0)
		{
			perror("shmctl:");
		}
		exit(1);
	}



	 PUMA_A1=emcmotConfig->PUMA_A[0];
	 PUMA_A2=emcmotConfig->PUMA_A[1];
	 PUMA_A3=emcmotConfig->PUMA_A[2];
	 PUMA_D3=emcmotConfig->PUMA_A[3];
	 PUMA_D4=emcmotConfig->PUMA_A[4];
	 PUMA_D6=emcmotConfig->PUMA_A[5];

	
	motCmd = &(commandShmem->motionCmd);
	interpList = &(commandShmem->motionCmdQ);

	printf("finished the initiate\n");
	return 0;
}

void CTRL_USR_Exit(void)
{

	 shmdt(commandShmem);
	 if(shmctl(commandShmID,IPC_RMID,0)<0)
	 {
		 perror("commandShmem shmctl:");
	 }

	 
	 shmdt(emcmotStatus);
	 if(shmctl(statusShmID,IPC_RMID,0)<0)
	 {
		 perror("emcmotStatus shmctl:");
	 }

	shmdt(emcmotConfig);
	if(shmctl(configShmID,IPC_RMID,0)<0)
	{
		perror("emcmotConfig shmctl:");
	}

	shmdt(motionFb);
	if(shmctl(feedbackShmID,IPC_RMID,0)<0)
	{
		perror("motionFb shmctl:");
	}

	shmdt(joints);
	if(shmctl(jointShmID,IPC_RMID,0)<0)
	{
		perror("joints shmctl:");
	}


}


