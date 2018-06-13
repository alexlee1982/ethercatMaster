#include <math.h>
#include <stdio.h>
#include "motionInterfaceBuffer.h"
#include "motionParameterType.h"
#include "motionParameter.h"
#include "motionIner.h"
#include "motionConfig.h"
#include "mot_priv.h"
#include "motionCommandType.h"
#include "contrller.h"
#include "motionErrorType.h"
#include "ec_interface.h"

extern int temporaryTestFlag;
void reportErrorType(int type)
{
	interpList->motionState=COMMAND_ERROR;
	errorCode=type;
	printf("errortype = %d\n",errorCode);
}
int getLastJoint(double *joint,int num)
{
	double pos;
	switch(num)
	{
		case 1:
			pos=emcmotDebug->queue.goalJPos.j0;
			break;
		case 2:
			pos=emcmotDebug->queue.goalJPos.j1;
			break;
		case 3:
			pos=emcmotDebug->queue.goalJPos.j2;
			break;
		case 4:
			pos=emcmotDebug->queue.goalJPos.j3;
			break;
		case 5:
			pos=emcmotDebug->queue.goalJPos.j4;
			break;
		case 6:
			pos=emcmotDebug->queue.goalJPos.j6;
			break;
		default:
			return -1;
			break;
	}
	if(*joint>=1e99)
	{
		*joint=pos;
	}
	return 0;
}
int adjustJoint(JointPoint *pos)
{
	double *joint;
	getLastJoint(&(pos->j1),1);
	getLastJoint(&pos->j2,2);
	getLastJoint(&pos->j3,3);
	getLastJoint(&pos->j4,4);
	getLastJoint(&pos->j5,5);
	getLastJoint(&pos->j6,6);

	return 0;
}

void commandBufferClear()
{
	interpList->start=interpList->end=0;
}

void refresh_jog_limits(MotionJointParameter*joint)
{
	 double range;
	 range = joint->max_pos_limit - joint->min_pos_limit;
	 if (GET_JOINT_HOMED_FLAG(joint))
	 {
		joint->max_jog_limit = joint->max_pos_limit;
		joint->min_jog_limit = joint->min_pos_limit;
	} 
	else
	{
		/* not homed, set limits based on current position */
		joint->max_jog_limit = joint->pos_fb + range;
		joint->min_jog_limit = joint->pos_fb -range;
	}
}


void setRotationToPm(RotationMatrix rot ,PmRotationMatrix *pm)
{
	pm->x.x=rot.x.x;
	pm->x.y=rot.x.y;
	pm->x.z=rot.x.z;

	pm->y.x=rot.y.x;
	pm->y.y=rot.y.y;
	pm->y.z=rot.y.z;

	pm->z.x=rot.z.x;
	pm->z.y=rot.z.y;
	pm->z.z=rot.z.z;

}


void setPointToPm(PmJoint *joint,JointPoint point)
{
	joint->j0=point.j1;
	joint->j1=point.j2;
	joint->j2=point.j3;
	joint->j3=point.j4;
	joint->j4=point.j5;
	joint->j5=point.j6;
	joint->j6=0.0;
	joint->j7=0.0;
}

void setPmToPoint(PmJoint joint,JointPoint *point)
{
	point->j1=joint.j0;
	point->j2=joint.j1;
	point->j3=joint.j2;
	point->j4=joint.j3;
	point->j5=joint.j4;
	point->j6=joint.j5;
}
void setArrayToJoint(PmJoint *joint,double *array)
{
	joint->j0=array[0];
	joint->j1=array[1];
	joint->j2=array[2];
	joint->j3=array[3];
	joint->j4=array[4];
	joint->j5=array[5];
	joint->j6=0.0;
	joint->j7=0.0;
}
static int inRange(JointPoint pos)
{
	double joint_pos[MAX_JOINTS];
	int joint_num;
	MotionJointParameter*joint;
	joint_pos[0]=pos.j1;
	joint_pos[1]=pos.j2;
	joint_pos[2]=pos.j3;
	joint_pos[3]=pos.j4;
	joint_pos[4]=pos.j5;
	joint_pos[5]=pos.j6;
	for (joint_num = 0; joint_num <6; joint_num++)
	{
		/* point to joint data */
		joint = &joints[joint_num];
		//refresh_jog_limits(joint);
		if ((joint_pos[joint_num] > joint->max_jog_limit) ||(joint_pos[joint_num] < joint->min_jog_limit))
	 	{
	    		return 0;		/* can't move further past limit */
		}
	}
   	 /* okay to move */
   	 return 1;
}
int checkAllHomed(void) 
{
	int joint_num;
	MotionJointParameter*joint;

    /* bail out if the allHomed flag is already set */
	if (0 != emcmotDebug)
	{
		if (emcmotDebug->allHomed) 
			return 1;
	}

	for (joint_num = 0; joint_num < 6; joint_num++)
	{
		/* point to joint data */
		joint = &joints[joint_num];
		if (!GET_JOINT_HOMED_FLAG(joint))
		{
	   		 /* if any of the joints is not homed return false */
	    		return 0;
		}
	}
	/* set the global flag that all axes are homed */
	if (0 != emcmotDebug)
	{
		emcmotDebug->allHomed = 1;
	}
		/* return true if all are actives are homed*/
	return 1;
}

void setJointMoveParam()
{
	PmJoint temp;
	tpSetId(&emcmotDebug->queue, emcmotCommand->currentLineNumber);                 
	tpSetFileName(&emcmotDebug->queue, emcmotCommand->currentFileNumber);                  
	tpSetJVScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movJ.vJ);
	setArrayToJoint(&temp,emcmotConfig->jointMoveVel);
	tpSetJVmax(&emcmotDebug->queue, temp);
	tpSetJVlimit(&emcmotDebug->queue, temp);
	setArrayToJoint(&temp,emcmotConfig->jointMoveAcc);
	tpSetJaccmax(&emcmotDebug->queue, temp);
	tpSetJdecmax(&emcmotDebug->queue, temp);
	//tpSetPL(&emcmotDebug->queue, emcmotCommand->data.trajJointsMove.PL);
	//tpSetTermCond(&emcmotDebug->queue, 
	//		 	emcmotCommand->data.trajJointsMove.PL ? TC_TERM_COND_BLEND : TC_TERM_COND_STOP, 
	//		 	(double)(emcmotCommand->data.trajJointsMove.PL / 50.0));
	//tpSetStopCondition(&emcmotDebug->queue, emcmotCommand->data.trajJointsMove.stopcondition);
	//tpSetIOaddress(&emcmotDebug->queue, emcmotCommand->data.trajJointsMove.ioaddr);
	//tpSetIOType(&emcmotDebug->queue, emcmotCommand->data.trajJointsMove.IOtype);
	//tpSetAddrType(&emcmotDebug->queue, emcmotCommand->data.trajJointsMove.addrtype);
	tpSetaccScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movJ.acc);
	tpSetdecScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movJ.dec);
}


int setLinearMoveParam()
{
	tpSetId(&emcmotDebug->queue, emcmotCommand->currentLineNumber);                  
	tpSetFileName(&emcmotDebug->queue, emcmotCommand->currentFileNumber);                 
	tpSetVmax(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movL.vL);
	tpSetVlimit(&emcmotDebug->queue, emcmotConfig->linearMaxV);
	tpSetAmax(&emcmotDebug->queue, emcmotConfig->linearMoveAcc);
	tpSetVR(&emcmotDebug->queue, emcmotConfig->rotateMoveV);
	tpSetRLimit(& emcmotDebug ->queue, emcmotConfig->rotateMoveV);
	tpSetRamax(&emcmotDebug->queue, emcmotConfig->rotateMoveAcc);
	tpSetaccScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movL.acc);
	tpSetdecScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movL.dec);
	//tpSetcordType(&emcmotDebug->queue, emcmotCommand->data.trajLinearMove.cord_type);
	return 0;
}
void seDescartestLinearMoveParam()
{
	tpSetId(&emcmotDebug->queue, emcmotCommand->currentLineNumber);                  
	tpSetFileName(&emcmotDebug->queue, emcmotCommand->currentFileNumber);                 
	tpSetVmax(&emcmotDebug->queue,emcmotCommand->motionCmdParameter.movDL.vL);
	tpSetVlimit(&emcmotDebug->queue, emcmotConfig->linearMaxV);
	tpSetAmax(&emcmotDebug->queue, emcmotConfig->linearMoveAcc);
	tpSetVR(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movDL.vR);
	tpSetRLimit(& emcmotDebug ->queue, emcmotConfig->rotateMoveV);
	tpSetRamax(&emcmotDebug->queue, emcmotConfig->rotateMoveAcc);
	tpSetaccScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movDL.acc);
	tpSetdecScale(&emcmotDebug->queue,  emcmotCommand->motionCmdParameter.movDL.acc);
}

void setSingeAxisMoveParam()
{
	PmJoint temp;
	setArrayToJoint(&temp,emcmotConfig->jointMoveVel);
	tpSetJVmax(&emcmotDebug->queue, temp);
	tpSetJVlimit(&emcmotDebug->queue, temp);
	setArrayToJoint(&temp,emcmotConfig->jointMoveAcc);
	tpSetJaccmax(&emcmotDebug->queue,temp);
	tpSetJdecmax(&emcmotDebug->queue,temp);
	tpSetaccScale(&emcmotDebug->queue, 1.0);
	tpSetdecScale(&emcmotDebug->queue, 1.0);

	tpSetVmax(&emcmotDebug->queue, emcmotConfig->teachLinearV);
	tpSetVlimit(&emcmotDebug->queue, emcmotConfig->teachLinearV);
	tpSetAmax(&emcmotDebug->queue, emcmotConfig->linearMoveAcc);
	tpSetVR(&emcmotDebug->queue, emcmotConfig->rotateMoveV);
	tpSetRLimit(& emcmotDebug ->queue, emcmotConfig->rotateMoveV);
	tpSetRamax(&emcmotDebug->queue, emcmotConfig->rotateMoveAcc);
}
void setCircularMoveParam()
{
	tpSetId(&emcmotDebug->queue, emcmotCommand->currentLineNumber);                  
	tpSetFileName(&emcmotDebug->queue, emcmotCommand->currentFileNumber);
	if(MOTION_COMMAND_CIRCULARMOVE_TYPE==emcmotCommand->type)
	{
		tpSetVmax(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movC.vL);
		tpSetaccScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movC.acc);
		tpSetdecScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movC.dec);
		tpSetVR(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movC.vR);
	}
	else
	{
		tpSetVmax(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movDC.vL);
		tpSetaccScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movDC.acc);
		tpSetdecScale(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movDC.dec);
		tpSetVR(&emcmotDebug->queue, emcmotCommand->motionCmdParameter.movDC.vR);
	}
	tpSetVlimit(&emcmotDebug->queue, emcmotConfig->teachLinearV);
	tpSetAmax(&emcmotDebug->queue, emcmotConfig->linearMoveAcc);
	tpSetRLimit(& emcmotDebug ->queue, emcmotConfig->rotateMoveV);
	tpSetRamax(&emcmotDebug->queue, emcmotConfig->rotateMoveAcc);
}

int  getCommand()
{
	int joint_num;
	MotionJointParameter*joint;                                                                           // ''motion.h''
	PmJoint end;
	static int incLine_flag,incJoint_flag;
	static int nowait;
	static int count_wait=0,timeout;
	int i;
	double pos_tmp;
	double positions[MAX_JOINTS];
	RobotPose world,world1,world2,world3,world4;
	int byte_num,bit_num;
	PmJoint endPoint;
	static int io_input_flag;
	int retval;
	int first_offset =1,first_home =1;
	int lastInputOffset[MAX_JOINTS],home_flag[MAX_JOINTS];	

	PmCartesian v1, v2,uVec, uVec1,uVec2,uVec3;
	double delta1, delta2, delta,tmpk;
  	PmRotationMatrix R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R0,Rx,Rz,h;
   	PmRotationMatrix	RT1,RT2,RT3,RT4 ; //R1-R4 ×ªÖÃ¾ØÕó
    	PmRotationMatrix RS0,RS1,RS2,RS3,RS4,RS5,RS6,RS7,RS8,RS9,RS10;
	PmCartesian P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13;
	PmRotationMatrix Rm1,Rm2,Rm3,Rmm,R6T,R8T,R10T;
	PmRotationMatrix invtcf;
	PmRpy rpy;
	PmCartesian Poe,Pxe,Pze,Pm;
	PmCartesian Pot,Pxt,Pzt,Pmt;
	PmCartesian Pxv,Pzv;
	PmCartesian kosn,koso,kosa;
	PmEulerZyx tempZyx;
    	PmHomogeneous hom;
    	PmPose worldPose;
    	double p[27];
	if (motCmd->head != motCmd->tail)
	{
       	return 0;			/* not really an error */
	}
	if (motCmd->serialNumber!=emcmotStatus->commandNumEcho)
	{
	        emcmotStatus->head++;
	        emcmotStatus->commandEcho = motCmd->type;
	        emcmotStatus->commandNumEcho = motCmd->serialNumber;
	        emcmotCommand=motCmd;
	} 
	else if (interpList->start!=interpList->end)
	{ 
		emcmotCommand=&interpList->motionCmd[interpList->start];
	}
	else
	{
       	return 0;
	} 
	emcmotStatus->commandStatus = EMCMOT_COMMAND_OK;
	printf("emcmotCommand->type=%d\n", emcmotCommand->type);
	switch(emcmotCommand->type)
	{
		case MOTION_COMMAND_SINGLEAXISMOVE_TYPE :
		{
			//temporaryTestFlag=1;
			int axis = 0;
			axis = emcmotCommand->motionCmdParameter.singleMove.axis-1;
			double position=0.0;
			//refresh_jog_limits(&joints[axis]);		
			if(!GET_MOTION_ENABLE_FLAG())
			{
				reportErrorType(MOTION_ERROR_CANNOT_MOVE_WITHOUT_ENABLE_TYPE);
			 	break;
			}
			if(!GET_MOTION_INPOS_FLAG())
			{
				reportErrorType(MOTION_ERROR_CANNOT_MOVE_WHEN_STILL_MOVING_TYPE);
				break;
			}
			
			setSingeAxisMoveParam();
			for(axis=0;axis<6;axis++)
			{
				emcmotDebug->net_feed_scale[axis]=emcmotCommand->motionCmdParameter.singleMove.vel;
			}
			axis = emcmotCommand->motionCmdParameter.singleMove.axis-1;
			

			switch(emcmotCommand->motionCmdParameter.singleMove.coordinate.coordinateType)
			{
				case COORDINATE_TYPE_JOINT :			
					if(MOVE_DIR_POSITIVE==emcmotCommand->motionCmdParameter.singleMove.direction)
						position=joints[axis].max_jog_limit;
					else
						position=joints[axis].min_jog_limit;
					emcmotStatus->coordinate_type=JOINT_COOR;
					position=emcmotCommand->motionCmdParameter.singleMove.endPoint;
					if(-1==tpAddFreeJoints(&emcmotDebug->queue,position,axis))
					{
						reportErrorType(MOTION_ERROR_CANNOT_ADD_JOINT_MOVE_TOTP_TYPE);
						//printf("failed to add a free joint move to the tp \n");
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						break;
					}
					break;
				case COORDINATE_TYPE_LINEAR :	
					if(MOVE_DIR_POSITIVE==emcmotCommand->motionCmdParameter.singleMove.direction)
						position=joints[axis].max_pos_limit;
					else
						position=joints[axis].min_pos_limit;
					if(MOVE_DIR_POSITIVE==emcmotCommand->motionCmdParameter.singleMove.direction)
						position=emcmotCommand->motionCmdParameter.singleMove.endPoint;
					else
						position=emcmotCommand->motionCmdParameter.singleMove.endPoint;
					emcmotStatus->coordinate_type=CAR_COOR;
					if(-1==tpAddFreeLine(&emcmotDebug->queue,position,1,axis))
					{
						reportErrorType(MOTION_ERROR_CANNOT_ADD_LINEAR_MOVE_TOTP_TYPE);
//						printf("failed to add a free linear move to the tp\n");
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						break;
					}
					break;
				case COORDINATE_TYPE_USER:
				{
					PmRpy tempRpy;
					PmRotationMatrix tempRotation;
					if(MOVE_DIR_POSITIVE==emcmotCommand->motionCmdParameter.singleMove.direction)
						position=joints[axis].max_pos_limit;
					else
						position=joints[axis].min_pos_limit;
					if(MOVE_DIR_POSITIVE==emcmotCommand->motionCmdParameter.singleMove.direction)
						position=emcmotCommand->motionCmdParameter.singleMove.endPoint;
					else
						position=emcmotCommand->motionCmdParameter.singleMove.endPoint;
					emcmotStatus->coordinate_type=USER_COOR;

					tempZyx.x = emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.user.a*PM_PI / 180.0;
					tempZyx.y = emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.user.b*PM_PI / 180.0;
					tempZyx.z = emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.user.c*PM_PI / 180.0;
					pmZyxMatConvert(tempZyx, &tempRotation);
					emcmotDebug->ORG.x=emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.user.x;
					emcmotDebug->ORG.y=emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.user.y;
					emcmotDebug->ORG.z=emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.user.z;
					emcmotDebug->Xvector= tempRotation.x;
					emcmotDebug->Yvector= tempRotation.y;
					emcmotDebug->Zvector= tempRotation.z;
					motionFb->user=emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.user;			
					if(-1==tpAddFreeLine(&emcmotDebug->queue,position,2,axis))
					{
						reportErrorType(MOTION_ERROR_CANNOT_ADD_LINEAR_MOVE_TOTP_TYPE);
//						printf("failed to add a free linear move to the tp\n");
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						break;
					}
				}
					break;		
				case COORDINATE_TYPE_TOOL:
				{
					PmRpy tempRpy;
					PmRotationMatrix tempRotation;
					if(MOVE_DIR_POSITIVE==emcmotCommand->motionCmdParameter.singleMove.direction)
						position=joints[axis].max_pos_limit;
					else
						position=joints[axis].min_pos_limit;
					if(MOVE_DIR_POSITIVE==emcmotCommand->motionCmdParameter.singleMove.direction)
						position=emcmotCommand->motionCmdParameter.singleMove.endPoint;
					else
						position=emcmotCommand->motionCmdParameter.singleMove.endPoint;
					emcmotStatus->coordinate_type=TOOL_COOR;
					tempZyx.x = emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.tool.rx*PM_PI / 180.0;
					tempZyx.y = emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.tool.ry*PM_PI / 180.0;
					tempZyx.z = emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.tool.rz*PM_PI / 180.0;
					pmZyxMatConvert(tempZyx, &tempRotation);
					
					emcmotDebug->robot_tcp.x=emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.tool.x;
					emcmotDebug->robot_tcp.y=emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.tool.y;
					emcmotDebug->robot_tcp.z=emcmotCommand->motionCmdParameter.singleMove.coordinate.CoordinateInformation.tool.z;

					emcmotDebug->robot_tcf=tempRotation;

					pmMatInv(emcmotDebug->robot_tcf,&(emcmotDebug->robot_invtcf));
					
					if(-1==tpAddFreeLine(&emcmotDebug->queue,position,3,axis))
					{
						reportErrorType(MOTION_ERROR_CANNOT_ADD_LINEAR_MOVE_TOTP_TYPE);
//						printf("failed to add a free linear move to the tp\n");
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						break;
					}
				}
					break;
				default:
					reportErrorType(MOTION_ERROR_UNKOWN_COORINATE_TYPE);
					printf("unknown coordinate type %d\n",emcmotCommand->motionCmdParameter.singleMove.coordinate.coordinateType);
					break;
			}
			interpList->motionState=COMMAND_EXEC;
		}	
			break;
		case MOTION_COMMAND_SERVO_RESET_TYPE :
			errorCode=0;
			interpList->motionState=COMMAND_DONE;
			tpAbort(&emcmotDebug->queue);
			SET_MOTION_ERROR_FLAG(0);
			for (joint_num = 0; joint_num < num_joints; joint_num++)
			{
            	joint = &joints[joint_num];
            	SET_JOINT_ERROR_FLAG(joint, 0);
				ec_slave_reset(joint_num);
        	}
			emcmotStatus->running=0;

	/*		for(joint_num=0;joint_num<6;joint_num++)
			{
				Servo_Reset(g_servo[joint_num]);
			}*/
			
			break;
		case MOTION_COMMAND_SERVO_ENABLE_TYPE :		
			if(0==emcmotCommand->motionCmdParameter.svEable.enable)
			{
				emcmotDebug->enabling=0;
			}
			else if(1==emcmotCommand->motionCmdParameter.svEable.enable)
			{
				if((!emcmotStatus->enble))
				{
					emcmotDebug->enabling=1;
					tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);
				}
			}
			else
			{
				reportErrorType(MOTION_ERROR_INPUT_ENABLE_VALUE_ERROR_TYPE);
				//there should not be any way to get in here,we should already checked the enable value in CTRL_ServoEnable
				//printf("input enble value error %d \n",emcmotCommand->motionCmdParameter.svEable.enable);
			}
			break;
		case MOTION_COMMAND_MOVEMENT_STOP_TYPE:
			
			tpAbort(&emcmotDebug->queue);
			SET_MOTION_ERROR_FLAG(0);
			for (joint_num = 0; joint_num < num_joints; joint_num++)
			{
            			joint = &joints[joint_num];
            			SET_JOINT_ERROR_FLAG(joint, 0);
        		}
			emcmotStatus->running=0;
			break;
		case MOTION_COMMAND_TRAJ_PAUSE_TYPE:
			
			tpPause(&emcmotDebug->queue);
			emcmotStatus->pause = 1;
			break;
		case MOTION_COMMAND_TRAJ_RESUME_TYPE:
			
			tpResume(&emcmotDebug->queue);
			emcmotStatus->pause = 0;
			break;
		case MOTION_COMMAND_JOINTMOVE_TYPE:
			 if (!checkAllHomed())
		 	{
		 		printf("not all axis is homed \n");
		 		commandBufferClear();
				break;
		 	}
			 if (!GET_MOTION_ENABLE_FLAG())
			{
				reportErrorType(MOTION_ERROR_CANNOT_MOVE_WITHOUT_ENABLE_TYPE);
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
            			break;
        		}
			else if(adjustJoint(&emcmotCommand->motionCmdParameter.movJ.endPoint))
			{
				break;
			}
		 	else if(!inRange(emcmotCommand->motionCmdParameter.movJ.endPoint))
		 	{
				reportErrorType(MOTION_ERROR_INPUT_OUTOF_SOFT_LIMIT_TYPE);
		 		printf("out of the soft limit\n");
				commandBufferClear();
				break;
		 	}
			else
			{
				PmJoint temp;
				setJointMoveParam();
				setPointToPm(&temp,emcmotCommand->motionCmdParameter.movJ.endPoint);
				//printf("j1= %f j2 =%f j3 =%f j4=%f j5=%f j6=%f \r\n",
				//emcmotCommand->motionCmdParameter.movJ.endPoint.j1,
				//emcmotCommand->motionCmdParameter.movJ.endPoint.j2,
				//emcmotCommand->motionCmdParameter.movJ.endPoint.j3,
				//emcmotCommand->motionCmdParameter.movJ.endPoint.j4,
				//emcmotCommand->motionCmdParameter.movJ.endPoint.j5,
				//emcmotCommand->motionCmdParameter.movJ.endPoint.j6
				//);
				if(-1==tpAddJoints(&emcmotDebug->queue,temp,0))
				{
					if(1==emcmotDebug->queue.queue.allFull)
					{
						break;
					}
					else
					{
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						reportErrorType(MOTION_ERROR_CANNOT_ADD_JOINT_MOVE_TOTP_TYPE);
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						printf("failed to add a joint move to the tp \n");
						commandBufferClear();
						break;
					}
				}
			}
		       interpList->start++; 
        		interpList->start= interpList->start % MOTION_COMMAD_QUEUE_SIZE;
        		interpList->motionState=COMMAND_EXEC;

		
			break;
		case MOTION_COMMAND_LINEARMOVE_TYPE:
			 if (!checkAllHomed())
		 	{
			 	printf("not all axis is homed \n");
				commandBufferClear();
			 	break;
		 	}
			 if (!GET_MOTION_ENABLE_FLAG())
			{
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
				break;
			}
			else if(adjustJoint(&emcmotCommand->motionCmdParameter.movL.endPoint))
			{
				break;
			}
			 else if(!inRange(emcmotCommand->motionCmdParameter.movL.endPoint))
		 	{
				reportErrorType(MOTION_ERROR_CANNOT_MOVE_WITHOUT_ENABLE_TYPE);
			 	printf("out of the soft limit\n");
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
				tpAbort(&emcmotDebug->queue);
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
				break;
		 	}
			 else
		 	{
			 	PmJoint temp;
				setLinearMoveParam();
				setPointToPm(&temp,emcmotCommand->motionCmdParameter.movL.endPoint);
				if(-1==tpAddLine(&emcmotDebug->queue,temp,1))
				{
					if(1==emcmotDebug->queue.queue.allFull)
					{
						break;
					}
					else
					{
						reportErrorType(MOTION_ERROR_CANNOT_ADD_LINEAR_MOVE_TOTP_TYPE);
						printf("failed to add a linear move to the tp\n");
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						commandBufferClear();
						break;
					}
				}
		 	}
			interpList->start++; 
	        	interpList->start= interpList->start % MOTION_COMMAD_QUEUE_SIZE;
	        	interpList->motionState=COMMAND_EXEC;
			break;
		case MOTION_COMMAND_DESCARTES_LINEARMOVE_TYPE:
		{
			int ret=0;
			
			PmJoint temp;
			JointPoint temp1;
			RobotPose world,world1;
			PmRotationMatrix Ehom;
			PmHomogeneous hom;
			double joint[6];
			double pre_joint[6];
			temp1.j1=emcmotDebug->queue.goalJPos.j0;
			temp1.j2=emcmotDebug->queue.goalJPos.j1;
			temp1.j3=emcmotDebug->queue.goalJPos.j2;
			temp1.j4=emcmotDebug->queue.goalJPos.j3;
			temp1.j5=emcmotDebug->queue.goalJPos.j4;
			temp1.j6=emcmotDebug->queue.goalJPos.j5;
			CTRL_GetTCPInUserSpaceInMatrix(&hom,&temp1,&emcmotCommand->motionCmdParameter.movDL.user, &emcmotCommand->motionCmdParameter.movDL.tool);
			
			//ret=kinematicsInverse(&world,&hom,joint,pre_joint,0);
			/*
			if(ret)
			{
				printf("can not add descartes linear move ,kinematicsInverse failed \n ");
				emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
				tpAbort(&emcmotDebug->queue);
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
				break;
			}
			else
			{
				setArrayToJoint(&temp,joint);
				setPmToPoint(temp,&temp1);
			}
			 if (!checkAllHomed())
		 	{
			 	printf("not all axis is homed \n");
				commandBufferClear();
			 	break;
		 	}
		 	*/
			 if (!GET_MOTION_ENABLE_FLAG())
			{
				reportErrorType(MOTION_ERROR_CANNOT_MOVE_WITHOUT_ENABLE_TYPE);
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
				break;
        		}
			 /*
			 else if(!inRange(temp1))
		 	{
			 	printf("out of the soft limit\n");
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
				tpAbort(&emcmotDebug->queue);
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
				break;
		 	}
		 	*/
			 else
		 	{
				seDescartestLinearMoveParam();
				PointPose tempEnd;
				
				tempEnd.x=emcmotCommand->motionCmdParameter.movDL.endPoint.x;
				tempEnd.y=emcmotCommand->motionCmdParameter.movDL.endPoint.y;
				tempEnd.z=emcmotCommand->motionCmdParameter.movDL.endPoint.z;

				tempEnd.a=emcmotCommand->motionCmdParameter.movDL.zyx.a;
				tempEnd.b=emcmotCommand->motionCmdParameter.movDL.zyx.b;
				tempEnd.c=emcmotCommand->motionCmdParameter.movDL.zyx.c;
				

				if(tempEnd.x>1e99)
				{
					tempEnd.x=hom.tran.x;
				}
				if(tempEnd.y>1e99)
				{
					tempEnd.y=hom.tran.y;
				}
				if(tempEnd.z>1e99)
				{
					tempEnd.z=hom.tran.z;
				}
				PmEulerZyx tempZyx;
				pmMatZyxConvert(hom.rot, &tempZyx);
				if(1==emcmotCommand->motionCmdParameter.movDL.free)
				{
					tempEnd.a=tempZyx.x;
					tempEnd.b=tempZyx.y;
					tempEnd.c=tempZyx.z;
					
				}
				else
				{
					if(tempEnd.a>1e99)
					{
						tempEnd.a=tempZyx.x;
					}
					else
					{
						tempEnd.a=emcmotCommand->motionCmdParameter.movDL.zyx.a*PM_PI/180.0;
					}
					if(tempEnd.b>1e99)
					{
						tempEnd.b=tempZyx.y;
					}
					else
					{
						tempEnd.b=emcmotCommand->motionCmdParameter.movDL.zyx.b*PM_PI/180.0;
					}
					if(tempEnd.c>1e99)
					{
						tempEnd.c=tempZyx.z;
					}
					else
					{
						tempEnd.c=emcmotCommand->motionCmdParameter.movDL.zyx.c*PM_PI/180.0;
					}
				}
				
				if(-1== tpAddUsrLine(&emcmotDebug->queue,tempEnd, emcmotCommand->motionCmdParameter.movDL.user, emcmotCommand->motionCmdParameter.movDL.tool))
				{
					if(1==emcmotDebug->queue.queue.allFull)
					{
						break;
					}
					else
					{
						reportErrorType(MOTION_ERROR_CANNOT_ADD_LINEAR_MOVE_TOTP_TYPE);
						printf("failed to add a descartes linear move to the tp\n");
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						commandBufferClear();
						break;
					}
				}
		 	}
		}
			interpList->start++; 
	        	interpList->start= interpList->start % MOTION_COMMAD_QUEUE_SIZE;
	        	interpList->motionState=COMMAND_EXEC;
			break;
		case MOTION_COMMAND_CIRCULARMOVE_TYPE:
		{
			if (!checkAllHomed())
		 	{
			 	printf("not all axis is homed \n");
				commandBufferClear();
			 	break;
		 	}
			 if (!GET_MOTION_ENABLE_FLAG())
			{
				reportErrorType(MOTION_ERROR_CANNOT_MOVE_WITHOUT_ENABLE_TYPE);
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
	            		break;
        		}
			 else 
		 	{
			 	int i=0,outOfRange=0;
			 	for(i=0;i<3;i++)
		 		{
			 		if(-1==inRange(emcmotCommand->motionCmdParameter.movC.endPoint[i]))
					{
						outOfRange=1;
		 			}
		 		}
				if(1==outOfRange)
				{
					reportErrorType(MOTION_ERROR_INPUT_OUTOF_SOFT_LIMIT_TYPE);
			 		//printf("out of the soft limit\n");
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
					tpAbort(&emcmotDebug->queue);
					SET_MOTION_ERROR_FLAG(1);
					commandBufferClear();
					break;
				}
		 	}
			 {
				int i=0;
			 	PmJoint temp[3];
				setCircularMoveParam();
				for(i=0;i<3;i++)
				{
					setPointToPm(&temp[i],emcmotCommand->motionCmdParameter.movC.endPoint[i]);
				}
				if(-1==tpAdd3DCircle(&emcmotDebug->queue,temp[0],temp[1],temp[2]))
				{
					if(1==emcmotDebug->queue.queue.allFull)
					{
						break;
					}
					else
					{
						reportErrorType(MOTION_ERROR_CANNOT_ADD_CIRCULAR_MOVE_TOTP_TYPE);
						//printf("failed to add a linear  to the tp\n");
						emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
						tpAbort(&emcmotDebug->queue);
						SET_MOTION_ERROR_FLAG(1);
						commandBufferClear();
						break;
					}
				}
			 }
			 interpList->start++; 
			 interpList->start= interpList->start % MOTION_COMMAD_QUEUE_SIZE;
			interpList->motionState=COMMAND_EXEC;	
		}
			break;
		case MOTION_COMMAND_DESCARTES_CIRCULARMOVE_TYPE:
		{
			int i=0;
			int j=0;
			int ret=0;
			PmJoint temp;
			JointPoint temp1,preJoint;
			RobotPose world,world1;
			PmRotationMatrix Ehom;
			PmHomogeneous hom;
			PointPose end[3];
			
			double joints[6];
			double pre_joints[6];
			PmEulerZyx tempZyx;
			if (!checkAllHomed())
			 {
			 	printf("not all axis is homed \n");
				commandBufferClear();
			 	break;
			 }
			 if (!GET_MOTION_ENABLE_FLAG())
			{
				reportErrorType(MOTION_ERROR_CANNOT_MOVE_WITHOUT_ENABLE_TYPE);
				emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_COMMAND;
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
	            		break;
	        	}
			temp1.j1=emcmotDebug->queue.goalJPos.j0;
			temp1.j2=emcmotDebug->queue.goalJPos.j1;
			temp1.j3=emcmotDebug->queue.goalJPos.j2;
			temp1.j4=emcmotDebug->queue.goalJPos.j3;
			temp1.j5=emcmotDebug->queue.goalJPos.j4;
			temp1.j6=emcmotDebug->queue.goalJPos.j5;		
			//kinematicsForward(pre_joints,&world1, &Ehom, &fflags,&iflags);
			CTRL_GetTCPInUserSpaceInMatrix(&hom,&temp1,&emcmotCommand->motionCmdParameter.movDC.user, &emcmotCommand->motionCmdParameter.movDC.tool);
			world.tran.x=emcmotCommand->motionCmdParameter.movDC.endPoint[0].x;
			world.tran.y=emcmotCommand->motionCmdParameter.movDC.endPoint[0].y;
			world.tran.z=emcmotCommand->motionCmdParameter.movDC.endPoint[0].z;
			
			hom.tran=world.tran;
			Ehom = hom.rot;
			preJoint.j1=pre_joints[0];
			preJoint.j2=pre_joints[1];
			preJoint.j3=pre_joints[2];
			preJoint.j4=pre_joints[3];
			preJoint.j5=pre_joints[4];
			preJoint.j6=pre_joints[5];
			if(emcmotCommand->motionCmdParameter.movDC.free==0)
			{
				tempZyx.x=emcmotCommand->motionCmdParameter.movDC.zyx[0].a*PM_PI/180.0;
				tempZyx.y=emcmotCommand->motionCmdParameter.movDC.zyx[0].b*PM_PI/180.0;
				tempZyx.z=emcmotCommand->motionCmdParameter.movDC.zyx[0].c*PM_PI/180.0;
				pmZyxMatConvert(tempZyx,&hom.rot);
			}
//			CTRL_GetTCPInJoint(&temp1, &preJoint, &hom, &emcmotCommand->motionCmdParameter.movDC.user, &emcmotCommand->motionCmdParameter.movDC.tool)
			ret=CTRL_GetTCPInJointInMatrix(&temp1, &preJoint, &hom, &emcmotCommand->motionCmdParameter.movDC.user, &emcmotCommand->motionCmdParameter.movDC.tool);
			if(ret)
			{
				reportErrorType(MOTION_ERROR_KINEMATICSINVERSE_FAILED_TYPE);
				//printf("can not add linear move ,kinematicsInverse failed \n ");
				emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
				tpAbort(&emcmotDebug->queue);
				SET_MOTION_ERROR_FLAG(1);
				commandBufferClear();
				break;
			}
			else
			{
				setCircularMoveParam();
//				setArrayToJoint(&temp,joints);
//				setPmToPoint(temp,&temp1);
				if(!inRange(temp1))
				{
					reportErrorType(MOTION_ERROR_INPUT_OUTOF_SOFT_LIMIT_TYPE);
				 	printf("out of the soft limit\n");
					emcmotStatus->commandStatus = EMCMOT_COMMAND_INVALID_PARAMS;
					tpAbort(&emcmotDebug->queue);
					SET_MOTION_ERROR_FLAG(1);
					commandBufferClear();
					break;
			 	}
//				if(-1==tpAddLine(&emcmotDebug->queue,temp,1))
				//{
				//	if(1==emcmotDebug->queue.queue.allFull)
				//	{
				//		break;
				//	}
				//	else
				//	{
				//		printf("failed to add a descartes linear move to the tp\n");
				//		emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
				//		tpAbort(&emcmotDebug->queue);
				//		SET_MOTION_ERROR_FLAG(1);
				//		commandBufferClear();
				//		break;
				//	}
				//}
			}
			for(i=0;i<3;i++)
			{
				
				end[i].x=emcmotCommand->motionCmdParameter.movDC.endPoint[i].x;
				end[i].y=emcmotCommand->motionCmdParameter.movDC.endPoint[i].y;
				end[i].z=emcmotCommand->motionCmdParameter.movDC.endPoint[i].z;
				if(end[i].x>1e99)
				{
					end[i].x=hom.tran.x;
				}
				if(end[i].y>1e99)
				{
					end[i].y=hom.tran.y;
				}
				if(end[i].z>1e99)
				{
					end[i].z=hom.tran.z;
				}
				pmMatZyxConvert(Ehom, &tempZyx);

				if(1==emcmotCommand->motionCmdParameter.movDC.free)
				{
					end[i].a=tempZyx.x*180.0/PM_PI;
					end[i].b=tempZyx.y*180.0/PM_PI;
					end[i].c=tempZyx.z*180.0/PM_PI;
				}
				else
				{
					if(end[i].a>1e99)
					{
						end[i].a=tempZyx.x*180.0/PM_PI;
					}
					else
					{
						end[i].a=emcmotCommand->motionCmdParameter.movDC.zyx[i].a;
					}
					if(end[i].b>1e99)
					{
						end[i].b=tempZyx.y*180.0/PM_PI;
					}
					else
					{
						end[i].b=emcmotCommand->motionCmdParameter.movDC.zyx[i].b;
					}
					if(end[i].c>1e99)
					{
						end[i].c=tempZyx.z*180.0/PM_PI;
					}
					else
					{
						end[i].c=emcmotCommand->motionCmdParameter.movDC.zyx[i].c;	
					}
				}	
			}
			/*			
			for(i=0;i<3;i++)
			{
				world.tran.x=emcmotCommand->motionCmdParameter.movDC.endPoint[i].x;
				world.tran.y=emcmotCommand->motionCmdParameter.movDC.endPoint[i].y;
				world.tran.z=emcmotCommand->motionCmdParameter.movDC.endPoint[i].z;
				hom.tran=world.tran;
				if(1==emcmotCommand->motionCmdParameter.movDC.free)
				{
					hom.rot=Ehom;
				}
				else
				{
					PmEulerZyx tempZyx;
					tempZyx.x=emcmotCommand->motionCmdParameter.movDL.zyx.a*108.0/PM_PI;
					tempZyx.y=emcmotCommand->motionCmdParameter.movDL.zyx.b*180.0/PM_PI;
					tempZyx.z=emcmotCommand->motionCmdParameter.movDL.zyx.c*180.0/PM_PI;
					pmZyxMatConvert(tempZyx,&(hom.rot));
				
					//setRotationToPm(emcmotCommand->motionCmdParameter.movDC.rot[i],&(hom.rot));
					//hom.rot=emcmotCommand->motionCmdParameter.movDC.rot[i];
				}	
				ret=kinematicsInverse(&world,&hom,joints,pre_joints, 0);
				if(ret)
				{
					printf("can not add descartes circular move ,kinematicsInverse failed \n ");
					emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
					tpAbort(&emcmotDebug->queue);
					SET_MOTION_ERROR_FLAG(1);
					commandBufferClear();
					break;
				}
				setArrayToJoint(&(end[i]),joints);
				for(j=0;j<6;j++)
				{
					pre_joints[j]=joints[j];
				}
			}
			*/
			 setCircularMoveParam();
			if(-1== tpAdd3DUsrCircle(&emcmotDebug->queue,end[0],end[1],end[2],emcmotCommand->motionCmdParameter.movDC.user,emcmotCommand->motionCmdParameter.movDC.tool))
			{
				if(1==emcmotDebug->queue.queue.allFull)
				{
					break;
				}
				else
				{
					reportErrorType(MOTION_ERROR_CANNOT_ADD_CIRCULAR_MOVE_TOTP_TYPE);
					printf("failed to add a circular move  to the tp\n");
					emcmotStatus->commandStatus = EMCMOT_COMMAND_BAD_EXEC;
					tpAbort(&emcmotDebug->queue);
					SET_MOTION_ERROR_FLAG(1);
					commandBufferClear();
					break;
				}
			}
			interpList->start++; 
			interpList->start= interpList->start % MOTION_COMMAD_QUEUE_SIZE;
			interpList->motionState=COMMAND_EXEC;		
		}
			break;		
		default:
			emcmotStatus->commandStatus = EMCMOT_COMMAND_UNKNOWN_COMMAND;
			break;
	}

	emcmotStatus->tail = emcmotStatus->head;
	return 0;
}
