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






//MotionDebug motionDebugBuf;
//MotionJointParameter jointBuf[MAX_JOINTS];
//MotionJointParameter *joints = jointBuf;
MotionJointParameter *joints;

MotionStatus *emcmotStatus ;

MotionConfig *emcmotConfig ;
//MotionDebug *emcmotDebug = &motionDebugBuf;
MotionDebug *emcmotDebug;

MotionCommandBuffer *commandShmem ;
VRobotMotionCommand  *motCmd ;
MotionCommandQueue *interpList ;
VRobotMotionCommand *emcmotCommand;
PmRotationMatrix Rtmp;
PmRotationVector Vector;
MotionFeedback *motionFb;
t_lock Mutex;

//MotionParameter  parameterBuf;
//MotionCommandBuffer commandBuf;
//MotionStatus motionStatusBuf;
//MotionConfig motConfigBuf;
MotionFeedback fb;

//t_servo* g_servo[6]={NULL,NULL,NULL,NULL,NULL,NULL};
double PUMA_A1 = 100.0;
double PUMA_A2 = 300.0;
double PUMA_A3 = 20.0;
double PUMA_D3 = 10.0;
double PUMA_D4 = 300.0;
double PUMA_D6 = 100.0;
double SCARA_L1=325;
double SCARA_L2=275;
 int num_joints = 6;

 int errorCode = 0;




int commandShmID,statusShmID,configShmID,feedbackShmID,jointShmID,debugShmID;


/*
int CTRL_USR_Init(void)
{
	//creat the share memory , map to the global
	
	//get the share memory for command
	//UsrCommandShmID,UsrStatusShmID,usrConfigShmID;
	UsrCommandShmID=shmget((key_t)MOTION_COMMAND_KEY,sizeof(MotionCommandBuffer),IPC_CREAT);
	if(UsrCommandShmID==-1)
	{
		perror("command shmget:");
		exit(1);
	}
	commandShmem = (MotionCommandBuffer*)shmat(UsrCommandShmID,NULL,0);
	if((int)commandShmem == -1)
	{
		perror("command shmat:");
		shmdt(commandShmem);
		if(shmctl(UsrCommandShmID,IPC_RMID,0)<0)
		{
			perror("command shmctl:");
		}
		exit(1);
	}
	

	//get the share memory for status 

	UsrStatusShmID=shmget((key_t)MOTION_STATUS_KEY,sizeof(MotionFeedback),IPC_CREAT);
	if(UsrStatusShmID==-1)
	{
		shmdt(commandShmem);
		if(shmctl(UsrCommandShmID,IPC_RMID,0)<0)
		{
			perror("command shmctl:");
		}
		perror("status shmget:");
		exit(1);
	}
	motionFb = (MotionFeedback*)shmat(UsrStatusShmID,NULL,0);
	if((int)motionFb == -1)
	{
		perror("command shmat:");
		shmdt(commandShmem);
		if(shmctl(UsrCommandShmID,IPC_RMID,0)<0)
		{
			perror("command shmctl:");
		}
		shmdt(motionFb);
		if(shmctl(UsrStatusShmID,IPC_RMID,0)<0)
		{
			perror("status shmctl:");
		}		
		exit(1);
	}
	
	//get the share memory for config

	usrConfigShmID=shmget((key_t)MOTION_CONFIG_KEY,sizeof(MotionConfig),IPC_CREAT);
	if(usrConfigShmID==-1)
	{
		shmdt(commandShmem);
		if(shmctl(UsrCommandShmID,IPC_RMID,0)<0)
		{
			perror("command shmctl:");
		}
		shmdt(motionFb);
		if(shmctl(UsrStatusShmID,IPC_RMID,0)<0)
		{
			perror("status shmctl:");
		}
		perror("config shmget:");
		exit(1);
	}
	emcmotConfig = (MotionConfig*)shmat(usrConfigShmID,NULL,0);
	if((int)emcmotConfig == -1)
	{
		perror("config shmat:");
		shmdt(commandShmem);
		if(shmctl(UsrCommandShmID,IPC_RMID,0)<0)
		{
			perror("command shmctl:");
		}
		shmdt(motionFb);
		if(shmctl(UsrStatusShmID,IPC_RMID,0)<0)
		{
			perror("status shmctl:");
		}
		shmdt(emcmotConfig);
		if(shmctl(usrConfigShmID,IPC_RMID,0)<0)
		{
			perror("status shmctl:");
		}	
		exit(1);
	}
	printf("finished the initiate\n");
	return 0;
}

void CTRL_USR_Exit(void)
{
	shmdt(commandShmem);
	if(shmctl(UsrCommandShmID,IPC_RMID,0)<0)
	{
		perror("command shmctl:");
	}
	shmdt(motionFb);
	if(shmctl(UsrStatusShmID,IPC_RMID,0)<0)
	{
		perror("status shmctl:");
	}
	shmdt(emcmotConfig);
	if(shmctl(usrConfigShmID,IPC_RMID,0)<0)
	{
		perror("status shmctl:");
	}
}


void* getShareMemory(key_t key,int size)
{
	int id;
	void *address;
	id=shmget((key_t)key,sizeof(size),IPC_CREAT);
	if(id==-1)
	{
		printf("%d ",key);
		perror(" shmget:");
		exit(1);
	}
	address = (void*)shmat(id,NULL,0);
	if((int)address == -1)
	{
		printf("%d ",key);
		perror(" shmat:");
		shmdt(address);
		if(shmctl(id,IPC_RMID,0)<0)
		{
			printf("%d ",key);
			perror("shmctl:");
		}
		exit(1);
	}
	return address;
}

 */

 int CTRL_Init(long us)
{
	INIT_LOCK(Mutex);
	
	//获取伺服信息
	int i=0;	
	int ret1,ret2,ret3,ret4,ret5,ret6;
	
	
	
	double period = (double)us;

//	commandShmem=(MotionCommandBuffer *)getShareMemory(MOTION_COMMAND_KEY,sizeof(MotionCommandBuffer));
//	motionFb=(MotionFeedback *)getShareMemory(MOTION_FEEDBACK_KEY,sizeof(MotionFeedback));
//	emcmotConfig=(MotionConfig *)getShareMemory(MOTION_CONFIG_KEY,sizeof(MotionConfig));
//	emcmotStatus=(MotionStatus *)getShareMemory(MOTION_STATUS_KEY,sizeof(MotionStatus));
//	joints=(MotionJointParameter *)getShareMemory(MOTION_JOINTS_KEY,sizeof(MotionJointParameter)*MAX_JOINTS);

//	emcmotConfig=&motConfigBuf;
//	emcmotStatus=&motionStatusBuf;
	
//	commandShmem=&commandBuf;
//	motionFb=&fb;

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



	motCmd = &(commandShmem->motionCmd);
	interpList = &(commandShmem->motionCmdQ);

//	g_servo[0]=MOTION_GetServo("J1");
//	g_servo[1]=MOTION_GetServo("J2");
//	g_servo[2]=MOTION_GetServo("J3");
//	g_servo[3]=MOTION_GetServo("J4");
//	g_servo[4]=MOTION_GetServo("J5");
//	g_servo[5]=MOTION_GetServo("J6");
	

	//printf("enable finished!\n");
//	for(i=0;i<6;i++)
//	{
//		if(NULL==g_servo[i])
//		{
//			printf("can not get the servo information \n");
//			return MOTION_ERROR_CANNOT_ACCESS_SERVO_FAILED_TYPE;
//		}
//		if(Servo_Init(g_servo[i]))
//		{
//			printf("can not init the servo struct \n");
//			return MOTION_ERROR_CANNOT_INIT_SERVO_FAILED_TYPE;			
//		}
//		Servo_Refresh(g_servo[i]);
//	}
	if (us <= 0)
	{
		printf("cycle time error %ld", us);
		return MOTION_ERROR_CYCLE_TIME_ERROR_TYPE;
	}
	//initialize some motion state....
	//first put some default value in to the inner variable
	// motion status
	PmJoint temp;

	emcmotStatus->commandEcho = -1;
	emcmotStatus->commandNumEcho = 0;
	emcmotStatus->commandStatus = EMCMOT_COMMAND_OK;
	emcmotStatus->motion_state = EMCMOT_MOTION_DISABLED;
	emcmotStatus->IObyte = -1; //do we still need it now?
	emcmotStatus->rapidMode = -1; //delete it later
	emcmotStatus->joint_type = 0; // 0 1 2 blending? , so now we do not need it.
	emcmotStatus->config_num = 0; // configuration changed? we don't need it now.
	printf("get servo done\n");

	emcmotStatus->net_feed_scale = 0; // freed? 
	emcmotStatus->linkErr = 0; //  we do not have any error now.
	emcmotStatus->distance_was_gone = 0; //come from tc.progress, i don't think it is necessary now
	emcmotStatus->distance_to_go = 0; //  samo samo
	emcmotStatus->motionFlag = 0x00000001; // if we are now just considering the algorithm this flag  should be a stable status and let it be
	emcmotStatus->motErrorFlag = 0; // we're not considering about any faults here at this moment, so....., let it be no faults.
	emcmotStatus->id = 0; //
	(void)emcmotStatus->fileName; //just don't touch it, or maybe i should just delete it.
	//emcmotStatus->coordinate_type = 0; //
	emcmotStatus->on_soft_limit = 0; //just as its name, when we are passing the limit, we need to stop,but...,do we need do something more?

	emcmotStatus->estop = 0; //useless, now
	emcmotStatus->pause = 0; //useless, now
	emcmotStatus->playing = 0; //now we never care about whether we are playing or what,right?
	emcmotStatus->enble = 0; //always enabled, right? what about joints flag?

	emcmotStatus->running = 0; //
	emcmotStatus->queueFull = 0; //
	emcmotStatus->linkAlmFlag = 0; //
	emcmotStatus->linkAlmReport = 0; //
	emcmotStatus->speed_scale = 150.0; //  i don't think we need it now, now we are running under a specific velocity environment

	emcmotStatus->joint_cmd.j0 = 0.0;
	emcmotStatus->joint_cmd.j1 = 0.0;
	emcmotStatus->joint_cmd.j2 = 0.0;
	emcmotStatus->joint_cmd.j3 = 0.0;
	emcmotStatus->joint_cmd.j4 = 0.0;
	emcmotStatus->joint_cmd.j5 = 0.0;


	//motion configuration
	emcmotConfig->PUMA_A[0] = PUMA_A1;
	emcmotConfig->PUMA_A[1] = PUMA_A2;
	emcmotConfig->PUMA_A[2] = PUMA_A3;
	emcmotConfig->PUMA_A[3] = PUMA_D4;
	emcmotConfig->PUMA_A[4] = PUMA_D6;

	emcmotConfig->teachLinearV = 100.0;
	emcmotConfig->playLinearV = 7000.0;
	emcmotConfig->linearMaxV = 7000.0;
	emcmotConfig->linearMoveAcc = 35000.0;
	emcmotConfig->rotateMoveAcc = 100.0;
	emcmotConfig->rotateMoveV = 100.0;
	emcmotConfig->linearPosLimit[0] = 1000000.0;
	emcmotConfig->linearPosLimit[1] = 1000000.0;
	emcmotConfig->linearPosLimit[2] = 1000000.0;
	emcmotConfig->linearPosLimit[3] = 180.0;
	emcmotConfig->linearPosLimit[4] = 180.0;
	emcmotConfig->linearPosLimit[5] = 180.0;

	emcmotConfig->linearNegaLimit[0] = -1000.0;
	emcmotConfig->linearNegaLimit[1] = -1000.0;
	emcmotConfig->linearNegaLimit[2] = -1000.0;
	emcmotConfig->linearNegaLimit[3] = -180.0;
	emcmotConfig->linearNegaLimit[4] = -180.0;
	emcmotConfig->linearNegaLimit[5] = -180.0;
	emcmotDebug->linkSimTest = 0;
	emcmotDebug->playing = 0;// never play again
	emcmotDebug->enabling = 0;
	for (i = 0; i<6; i++)
	{
		emcmotConfig->jointPosLimit[i] = 180.0;
		emcmotConfig->jointNegaLimit[i] = -180;
		emcmotConfig->jointMoveVel[i] = 180.0;
		emcmotConfig->jointMoveAcc[i] = 300.0;
		emcmotConfig->jointMoveMaxVel[i] = 180.0;
		emcmotConfig->mechTransRatio[i] = 1.0;
		//emcmotConfig->servoPulsePerRound[i] = g_servo[i]->plusPerRound;
		emcmotConfig->servoPulsePerRound[i]=36000;
		
		emcmotConfig->ServoMaxVelocity[i] = 1000;
		//emcmotConfig->encodeOriginOffset[i] = 0;
		//emcmotConfig->servoPolarity[i]=g_servo[i]->servoPolarity;
		emcmotConfig->servoPolarity[i]=1;
		//emcmotConfig->encoderType[i]=g_servo[i]->encoderType;
		emcmotConfig->encoderType[i]=1;
	}


	emcmotConfig->mechTransRatio[0] = 101.0;
	emcmotConfig->mechTransRatio[1] = -101.0;
	emcmotConfig->mechTransRatio[2] = 101.0;
	emcmotConfig->mechTransRatio[3] = -173.142857;
	emcmotConfig->mechTransRatio[4] = 101.0;
	emcmotConfig->mechTransRatio[5] = -51.0;
	emcmotConfig->coupleRate=51;
	emcmotConfig->mechTransDenominator[0]=1;
	emcmotConfig->mechTransDenominator[1]=-1;
	emcmotConfig->mechTransDenominator[2]=1;
	emcmotConfig->mechTransDenominator[3]=14;
	emcmotConfig->mechTransDenominator[4]=1;
	emcmotConfig->mechTransDenominator[5]=-1;
	
	emcmotConfig->mechTransNumerator[0]=101;
	emcmotConfig->mechTransNumerator[1]=101;
	emcmotConfig->mechTransNumerator[2]=101;
	emcmotConfig->mechTransNumerator[3]=2424;
	emcmotConfig->mechTransNumerator[4]=101;
	emcmotConfig->mechTransNumerator[5]=51;



	
	//joints
	for (i = 0; i<MAX_JOINTS; i++)
	{
		joints[i].motorInputScale = 1.0;	//we don't care about the motor now
		joints[i].motorInverseInputScale = 1.0;
		joints[i].svtype = 0;
		joints[i].not_first_input = 0;
		joints[i].max_pos_limit = 1000.0;
		joints[i].min_pos_limit = -1000.0;
		joints[i].min_jog_limit = -3600.0;
		joints[i].max_jog_limit = 3600.0;
		joints[i].Jvel_limit = 360.0;
		joints[i].vel_limit = 360.0;
		joints[i].acc_limit = 1000.0;
		joints[i].min_ferror = 50.0;
		joints[i].max_ferror = 50.0;
		//joints[i].servoEncoderPolarity = (emcmotConfig->servoPolarity[i]== 1)?1:0;
		joints[i].servoEncoderPolarity = (emcmotConfig->servoPolarity[i]== 1)?1:0;
		/* init status info */
		joints[i].motorInputOffset = 0;
		joints[i].offsetInit = 0;
		joints[i].flag = 2;
		joints[i].errorFlag = 0;
		joints[i].coarse_pos = 0.0;
		joints[i].pos_cmd = 0.0;
		joints[i].vel_cmd = 0.0;
		joints[i].pos_fb = 0.0;
		joints[i].ferror = 0.0;
		joints[i].ferror_limit = joints[i].min_ferror;
		joints[i].ferror_high_mark = 0.0;
		joints[i].encoder_counts = emcmotConfig->servoPulsePerRound[i];
	}
	//joints[0].max_jog_limit=170;
	//joints[0].min_jog_limit=-170;
	//joints[1].max_jog_limit=160;
	//joints[1].min_jog_limit=-60;
	//joints[2].max_jog_limit=210;
	//joints[2].min_jog_limit=-65;
	//joints[3].max_jog_limit=180;
	//joints[3].min_jog_limit=-180;
	
	//joints[4].max_jog_limit=135;
	//joints[4].min_jog_limit=-135;
	//joints[5].max_jog_limit=360;
	//joints[5].min_jog_limit=-359.9;

	for (i = 0; i<6; i++)
	{
		emcmotConfig->jointPosLimit[i] = joints[i].max_jog_limit;
		emcmotConfig->jointNegaLimit[i] = joints[i].min_jog_limit;
	}



	if (-1 == tpCreate(&emcmotDebug->queue, DEFAULT_TC_QUEUE_SIZE, emcmotDebug->queueTcSpace))
	{
		printf("can not init the motion tp\n");
		return MOTION_ERROR_CANNOT_CREAT_TP_TYPE;
	}
	tpSetCycleTime(&emcmotDebug->queue, period/1000000);
	//    tpSetPos(&emcmotDebug->queue, emcmotStatus->carte_pos_cmd);
	tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);
	printf("Motion : finished the initiation \n");
	
	return 0;
}
 void CTRL_Exit(void)
{
	 shmdt(commandShmem);
	 if(shmctl(commandShmID,IPC_RMID,0)<0)
	 {
		 perror("shmctl:");
	 }

	 
	 shmdt(emcmotStatus);
	 if(shmctl(statusShmID,IPC_RMID,0)<0)
	 {
		 perror("shmctl:");
	 }

	shmdt(emcmotConfig);
	if(shmctl(configShmID,IPC_RMID,0)<0)
	{
		perror("shmctl:");
	}

	shmdt(motionFb);
	if(shmctl(feedbackShmID,IPC_RMID,0)<0)
	{
		perror("shmctl:");
	}

	shmdt(joints);
	if(shmctl(jointShmID,IPC_RMID,0)<0)
	{
		perror("shmctl:");
	}

	 
	// pthread_mutexattr_destroy(&Mutex);
}

 int CTRL_GetTCPInUserSpace(PointPose *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
{
	PmHomogeneous TBE;
	PmHomogeneous TET,TBU,TUT,tempT;
	PmHomogeneous TBUIN;
	PmPose PBE,PET,PBU,PUT,PBUIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	int i = 0;
	PmEulerZyx toolZyx,userZyx,tempZyx;
	tempJoints[0]=joint->j1;
	tempJoints[1]=joint->j2;
	tempJoints[2]=joint->j3;
	tempJoints[3]=joint->j4;
	tempJoints[4]=joint->j5;
	tempJoints[5]=joint->j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	
	
	TBE.tran.x=world.tran.x;
	TBE.tran.y=world.tran.y;
	TBE.tran.z=world.tran.z;
	
	TBE.rot=Ehom;

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx,&(TET.rot));

	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;

	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;

	pmZyxMatConvert(userZyx,&(TBU.rot));
	


	pmHomInv(TBU,&TBUIN);



	pmMatMatMult(TBUIN.rot, TBE.rot,&(tempT.rot));
	pmMatCartMult(TBUIN.rot,TBE.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBUIN.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TET.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TET.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));



/*	
	pmMatMatMult(TBE.rot, TET.rot,&(tempT.rot));
	pmMatCartMult(TBE.rot,TET.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBE.tran,&(tempT.tran));
	
	pmMatMatMult(tempT.rot, TBUIN.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TBUIN.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));
*/
	pmMatZyxConvert(TUT.rot,&tempZyx);
	
	pos->x=TUT.tran.x;
	pos->y=TUT.tran.y;
	pos->z=TUT.tran.z;
	pos->a=tempZyx.x*180.0/PM_PI;
	pos->b=tempZyx.y*180.0/PM_PI;
	pos->c=tempZyx.z*180.0/PM_PI;

	return 0 ;

}
 int CTRL_GetTCPInUserSpaceInMatrix(PmHomogeneous *pos,JointPoint *joint,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
 {
 	PmHomogeneous TBE;
	PmHomogeneous TET,TBU,TUT,tempT;
	PmHomogeneous TBUIN;
	PmPose PBE,PET,PBU,PUT,PBUIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;
	int i = 0;
	PmEulerZyx toolZyx,userZyx,tempZyx;
	tempJoints[0]=joint->j1;
	tempJoints[1]=joint->j2;
	tempJoints[2]=joint->j3;
	tempJoints[3]=joint->j4;
	tempJoints[4]=joint->j5;
	tempJoints[5]=joint->j6;
	kinematicsForward(tempJoints,&world,&Ehom,&fflags,&iflags);
	
	
	TBE.tran.x=world.tran.x;
	TBE.tran.y=world.tran.y;
	TBE.tran.z=world.tran.z;
	
	TBE.rot=Ehom;

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx,&(TET.rot));

	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;

	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;

	pmZyxMatConvert(userZyx,&(TBU.rot));
	


	pmHomInv(TBU,&TBUIN);



	pmMatMatMult(TBUIN.rot, TBE.rot,&(tempT.rot));
	pmMatCartMult(TBUIN.rot,TBE.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBUIN.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TET.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TET.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));



/*	
	pmMatMatMult(TBE.rot, TET.rot,&(tempT.rot));
	pmMatCartMult(TBE.rot,TET.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBE.tran,&(tempT.tran));
	
	pmMatMatMult(tempT.rot, TBUIN.rot,&(TUT.rot));
	pmMatCartMult(tempT.rot,TBUIN.tran,&(TUT.tran));
	pmCartCartAdd(TUT.tran, tempT.tran,&(TUT.tran));
*/
	
	*pos=TUT;

	return 0 ;
 }
 int CTRL_GetTCPInJoint(JointPoint *joint,PointPose *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
{
	PmHomogeneous TBE,TET,TBU,TUT,tempT;
	PmHomogeneous TETIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	double pre_joints[6];
	JointPoint startJoint;
	PointPose startPoint;
	int ret=0;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;

	PmEulerZyx toolZyx,userZyx,tempZyx;
	
	startJoint.j1=emcmotDebug->queue.goalJPos.j0;
	startJoint.j2=emcmotDebug->queue.goalJPos.j1;
	startJoint.j3=emcmotDebug->queue.goalJPos.j2;
	startJoint.j4=emcmotDebug->queue.goalJPos.j3;
	startJoint.j5=emcmotDebug->queue.goalJPos.j4;
	startJoint.j6=emcmotDebug->queue.goalJPos.j5;

	CTRL_GetTCPInUserSpace(&startPoint,&startJoint,user,tool);
	if(point->x>=1e99)
	{
		TUT.tran.x=startPoint.x;
	}
	else
	{
		TUT.tran.x=point->x;
	}
	if(point->y>=1e99)
	{
		TUT.tran.y=startPoint.y;
	}
	else
	{
		TUT.tran.y=point->y;
	}
	if(point->z>=1e99)
	{
		TUT.tran.z=startPoint.z;
	}
	else
	{
		TUT.tran.z=point->z;
	}
	if(point->a>=1e99)
	{
		toolZyx.x=startPoint.a*PM_PI/180.0;
	}
	else
	{
		toolZyx.x=point->a*PM_PI/180.0;
	}
	if(point->b>=1e99)
	{
		toolZyx.y=startPoint.b*PM_PI/180.0;
	}
	else
	{
		toolZyx.y=point->b*PM_PI/180.0;
	}
	if(point->c>=1e99)
	{
		toolZyx.z=startPoint.c*PM_PI/180.0;
	}
	else
	{
		toolZyx.z=point->c*PM_PI/180.0;
	}


	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;
	
	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;
	pmZyxMatConvert(userZyx, &(TBU.rot));

//	TUT.tran.x=point->x;
//	TUT.tran.y=point->y;
//	TUT.tran.z=point->z;

//	toolZyx.r=point->a*PM_PI/180.0;
//	toolZyx.p=point->b*PM_PI/180.0;
//	toolZyx.y=point->c*PM_PI/180.0;

	pmZyxMatConvert(toolZyx,&(TUT.rot));

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx, &(TET.rot));
	pmHomInv(TET,&TETIN);

	pmMatMatMult(TBU.rot, TUT.rot,&(tempT.rot));
	pmMatCartMult(TBU.rot,TUT.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBU.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TETIN.rot,&(TBE.rot));
	pmMatCartMult(tempT.rot,TETIN.tran,&(TBE.tran));
	pmCartCartAdd(TBE.tran,tempT.tran,&(TBE.tran));
	world.tran=TBE.tran;
	pre_joints[0]= emcmotDebug->queue.goalJPos.j0;
	pre_joints[1]= emcmotDebug->queue.goalJPos.j1;
	pre_joints[2]= emcmotDebug->queue.goalJPos.j2;
	pre_joints[3]= emcmotDebug->queue.goalJPos.j3;
	pre_joints[4]= emcmotDebug->queue.goalJPos.j4;
	pre_joints[5]= emcmotDebug->queue.goalJPos.j5;

	ret=kinematicsInverse( &world,&TBE,tempJoints,pre_joints,0);
	joint->j1=tempJoints[0];
	joint->j2=tempJoints[1];
	joint->j3=tempJoints[2];
	joint->j4=tempJoints[3];
	joint->j5=tempJoints[4];
	joint->j6=tempJoints[5];
	if(ret)
	{
		return MOTION_ERROR_KINEMATICSINVERSE_FAILED_TYPE;
	}
	return 0;
}

 int CTRL_GetTCPInJointInMatrix(JointPoint *joint,JointPoint *preJoint,PmHomogeneous *point,UserCoordianteInformation* user,ToolCoordianteInformation* tool)
 {
	PmHomogeneous TBE,TET,TBU,TUT,tempT;
	PmHomogeneous TETIN;
	RobotPose world;
	PmRotationMatrix  Ehom;
	double tempJoints[6];
	double pre_joints[6];
	int ret=0;
	KINEMATICS_FORWARD_FLAGS fflags;
	KINEMATICS_INVERSE_FLAGS iflags;

	PmEulerZyx toolZyx,userZyx,tempZyx;

	TBU.tran.x=user->x;
	TBU.tran.y=user->y;
	TBU.tran.z=user->z;

	userZyx.x=user->a*PM_PI/180.0;
	userZyx.y=user->b*PM_PI/180.0;
	userZyx.z=user->c*PM_PI/180.0;

	pmZyxMatConvert(userZyx, &(TBU.rot));

	TUT.tran=point->tran;
	TUT.rot=point->rot;

//	toolRpy.r=point->a*PM_PI/180.0;
//	toolRpy.p=point->b*PM_PI/180.0;
//	toolRpy.y=point->c*PM_PI/180.0;
//	pmRpyMatConvert(toolRpy,&(TUT.rot));

	TET.tran.x=tool->x;
	TET.tran.y=tool->y;
	TET.tran.z=tool->z;

	toolZyx.x=tool->rx*PM_PI/180.0;
	toolZyx.y=tool->ry*PM_PI/180.0;
	toolZyx.z=tool->rz*PM_PI/180.0;

	pmZyxMatConvert(toolZyx, &(TET.rot));
	pmHomInv(TET,&TETIN);

	pmMatMatMult(TBU.rot, TUT.rot,&(tempT.rot));
	pmMatCartMult(TBU.rot,TUT.tran,&(tempT.tran));
	pmCartCartAdd(tempT.tran,TBU.tran,&(tempT.tran));


	pmMatMatMult(tempT.rot, TETIN.rot,&(TBE.rot));
	pmMatCartMult(tempT.rot,TETIN.tran,&(TBE.tran));
	pmCartCartAdd(TBE.tran,tempT.tran,&(TBE.tran));
	world.tran=TBE.tran;
	pre_joints[0]=preJoint->j1;
	pre_joints[1]=preJoint->j2;
	pre_joints[2]=preJoint->j3;
	pre_joints[3]=preJoint->j4;
	pre_joints[4]=preJoint->j5;
	pre_joints[5]=preJoint->j6;

	ret=kinematicsInverse( &world,&TBE,tempJoints,pre_joints,0);
	joint->j1=tempJoints[0];
	joint->j2=tempJoints[1];
	joint->j3=tempJoints[2];
	joint->j4=tempJoints[3];
	joint->j5=tempJoints[4];
	joint->j6=tempJoints[5];
	if(ret)
	{
		return MOTION_ERROR_KINEMATICSINVERSE_FAILED_TYPE;
	}
	return 0;
 }


 int CTRL_SCARA_KineticForward(JointPoint *jp,PmHomogeneous *pos)
 {
 	double s[3],c[3];
 	s[0]=sin(180/3.1415926*jp->j1);
 	c[0]=cos(180/3.1415926*jp->j1);
 	s[1]=sin(180/3.1415926*jp->j2);
 	c[1]=cos(180/3.1415926*jp->j2);
 	s[2]=sin(180/3.1415926*jp->j3);
 	c[2]=cos(180/3.1415926*jp->j3);
 		
 	
 	pos->tran.x=(c[0]*c[1]-s[0]*s[1])*SCARA_L2+c[0]*SCARA_L1;
 	pos->tran.y=(s[0]*c[1]+c[0]*s[1])*SCARA_L2+s[0]*SCARA_L1;
 	pos->tran.z=jp->j4;

	pos->rot.x.x=c[0]*c[1]*c[2]-s[0]*s[1]*c[2]-c[1]*s[1]*s[2]-s[0]*c[1]*s[2];
	pos->rot.x.y=s[0]*c[1]*c[2]+c[0]*s[1]*c[3]-s[0]*s[1]*s[2]+c[0]*c[1]*s[2];
	pos->rot.x.z=0.0;
	pos->rot.y.x=-c[0]*c[1]*s[2]+s[0]*s[1]*s[2]-c[0]*s[1]*s[2]-s[0]*c[1]*c[2];
	pos->rot.y.y=-s[0]*c[1]*s[2]-c[0]*s[1]*s[2]-s[0]*s[1]*c[2]+c[0]*c[1]*c[2];
	pos->rot.y.z=0.0;
	pos->rot.z.x=0.0;
	pos->rot.z.y=0.0;
	pos->rot.z.z=1;
 	return 0;
 }
 

int CTRL_SCARA_KineticInverse(PmHomogeneous *pos,JointPoint *jp)
{
	double th1,th2,th3,th23;
	double x,y;
	double c1,c2,s1,c23;
	x=pos->tran.x;
	y=pos->tran.y;
	c2=(x*x+y*y-SCARA_L1*SCARA_L1-SCARA_L2*SCARA_L2)/SCARA_L1*SCARA_L2;
	th2=acos(c2);
	s1=sin(th2);
	c1=(c2*SCARA_L2*x+SCARA_L1*x+s1*SCARA_L2*y)/(x*x+y*y);
	th1=acos(c1);

	c23=c1*pos->rot.x.x+s1*pos->rot.x.y;
	th23=acos(c23);

	th3=th23-th2;
	jp->j1=th1;
	jp->j2=th2;
	jp->j3=th3;
	jp->j4=pos->tran.z;


	return 0;
}

 int CTRL_SCARA_KineticForwardInABC(JointPoint *jp,PointPose *point)
 {
	PmHomogeneous a;
	PmHomogeneous *pos=&a;
	PmEulerZyx zyx;


 	double s[3],c[3];
	double th123,c123,s123;
 	s[0]=sin(3.1415926*jp->j1/180.0);
 	c[0]=cos(3.1415926*jp->j1/180.0);
 	s[1]=sin(3.1415926*jp->j2/180.0);
 	c[1]=cos(3.1415926*jp->j2 / 180.0);
 	s[2]=sin(3.1415926*jp->j3 / 180.0);
 	c[2]=cos(3.1415926*jp->j3 / 180.0);
 		
	th123 = jp->j1 + jp->j2 + jp->j3;
	c123 = cos(th123*3.1415926/180.0);
	s123 = sin(th123*3.1415926 / 180.0);
 	pos->tran.x=(c[0]*c[1]-s[0]*s[1])*SCARA_L2+c[0]*SCARA_L1;
 	pos->tran.y=(s[0]*c[1]+c[0]*s[1])*SCARA_L2+s[0]*SCARA_L1;
 	pos->tran.z=jp->j4;

	pos->rot.x.x=c123;
	pos->rot.x.y=s123;
	pos->rot.x.z=0.0;
	pos->rot.y.x=-s123;
	pos->rot.y.y=c123;
	pos->rot.y.z=0.0;
	pos->rot.z.x=0.0;
	pos->rot.z.y=0.0;
	pos->rot.z.z=1;	

	point->x=pos->tran.x;
	point->y=pos->tran.y;
	point->z=pos->tran.z;
	
	pmMatZyxConvert(pos->rot,&zyx);
	point->a=zyx.x*180.0/3.1415926;
	point->b=zyx.y*180.0 / 3.1415926;
	point->c=zyx.z*180.0 / 3.1415926;
	
 
 	return 0;
 }
 int CTRL_SCARA_KineticInverseInABC(PointPose *point,JointPoint *jp)
 {
	PmHomogeneous a;
	PmHomogeneous *pos=&a;
	PmEulerZyx zyx;
	double th1,th2,th3,th23;
	double x,y;
	double c1,c2,s1,s2,c23;
	zyx.x=point->a/180.0*3.1415926;
	zyx.y=point->b / 180.0*3.1415926;
	zyx.z=point->c / 180.0*3.1415926;
	
	pmZyxMatConvert(zyx, &(pos->rot));
	pos->tran.x=point->x;
	pos->tran.y=point->y;
	pos->tran.z=point->z;


	x=pos->tran.x;
	y=pos->tran.y;
	c2=(x*x+y*y-SCARA_L1*SCARA_L1-SCARA_L2*SCARA_L2)/(2*SCARA_L1*SCARA_L2);
	th2=acos(c2);
	s2=sin(th2);
	c1=(c2*SCARA_L2*x+SCARA_L1*x+s2*SCARA_L2*y)/(x*x+y*y);
	th1=acos(c1);
	s1 = sin(th1);
	c23=c1*pos->rot.x.x+s1*pos->rot.x.y;
	th23=acos(c23);
	th23 = th23*180.0 / 3.1415926;
	jp->j1=th1/3.1415926*180.0;
	jp->j2=th2/3.1415926*180.0;
	jp->j3=th23-jp->j2;
	jp->j4=pos->tran.z;	
	


	
 	return 0;
 }





 
