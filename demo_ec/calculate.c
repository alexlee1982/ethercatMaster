#include <float.h>              /* DBL_MIN */
#include <math.h>
#include <stdio.h>
#include "motionIner.h"
#include "mot_priv.h"
#include "contrller.h"
#include "Common.h"
//#include "../include/contrller.h"
#include "motionErrorType.h"
//#include "MotionApi.h"
//#include "stdafx.h"
#include "ec_interface.h"

static double traj_period;
static double traj_freq;
static double servo_period;
static double servo_freq;
int over_vel;
static int  enbl_flag,enbl_count;

extern void reportErrorType(int type);

double newCorse(double corse)
{
	double tmpCorse;
	tmpCorse = corse - (int)corse / 360 * 360.0;
	if (tmpCorse < 0.0) {
		tmpCorse += 360.0;
	}

	if (tmpCorse > 180.0)
		tmpCorse -= 360.0;
	if (tmpCorse <= -180.0)
		tmpCorse += 360.0;

	return tmpCorse;
}


int readInputPosition()
{
	MotionJointParameter *joint;
	double joint_pos[MAX_JOINTS] = {0,};
	double abs_ferror;
	RobotPose world;
	unsigned long inPutPulse=0;
	int i=0;
	ec_user_statust_t status; 
	//now we are  doing simulation ,so the command positon will be the feedback postion
	//we don't have any environment now,so....,there is no input
	//printf("joint[5].pos_fb=%f \r\n",joints[5].pos_fb);
	for(i=0;i<6;i++)
	{
		joint=&joints[i];
		//emcmotDebug->inputRot[i]=0;
		//emcmotDebug->inputInitCount[i]=emcmotDebug->inputInitCount[i] = emcmotDebug->inputRot[i] * joint->encoder_counts;
		//joint->rawMotorInput=0;
		//Servo_Refresh(g_servo[i]);
		ec_slave_status(i,&status);
		if(5==i&&emcmotConfig->coupleRate!=0)
		{
			joint->pos_fb = 360.0*(status.actual_position-emcmotConfig->encodeOriginOffset[i])/emcmotConfig->servoPulsePerRound[i]/emcmotConfig->mechTransRatio[i]-joints[4].pos_cmd/emcmotConfig->coupleRate;
			//joint->pos_fb=joint->pos_cmd;
		}
		else
		{
			joint->pos_fb = 360.0*(status.actual_position-emcmotConfig->encodeOriginOffset[i])/emcmotConfig->servoPulsePerRound[i]/emcmotConfig->mechTransRatio[i];
			//joint->pos_fb=joint->pos_cmd;
		}
			
		
		
		//check limit, do it later....,just check the algorithm first
		emcmotDebug->net_feed_scale[i] = emcmotStatus->speed_scale*100.0/150.0;
		if (emcmotDebug->linkSimTest) 
		{
			joint->pos_fb=joint->pos_cmd;
			joint_pos[i] = joint->pos_fb;	
		}
		else
		{
			//inPutPulse=
			//do nothing by now
			joint_pos[i]=joint->pos_fb;
			//printf("need the simTest flag\n");
			//joint->pos_fb=(inPutPulse-emcmotConfig->encodeOriginOffset[i])*360*emcmotConfig->mechTransRatio/emcmotConfig->servoPulsePerRound;
		}
		//leave them be by now
		//joint->ferror = joint->pos_cmd - joint->pos_fb;
		//abs_ferror = fabs(joint->ferror);                          

	}
	kinematicsForward(joint_pos, &world,&Rtmp, &fflags,&iflags);

	emcmotStatus->joint_fb.j0=joints[0].pos_fb;
	emcmotStatus->joint_fb.j1=joints[1].pos_fb;
	emcmotStatus->joint_fb.j2=joints[2].pos_fb;
	emcmotStatus->joint_fb.j3=joints[3].pos_fb;
	emcmotStatus->joint_fb.j4=joints[4].pos_fb;
	emcmotStatus->joint_fb.j5=joints[5].pos_fb;
	
	emcmotStatus->carte_pos_fb.tran=world.tran;
	emcmotStatus->carte_pos_fb.a=world.a;
	emcmotStatus->carte_pos_fb.b=world.b; 
	emcmotStatus->carte_pos_fb.c=world.c; 

	return 0;
}
void checkForError()
{

	int joint_num;
	MotionJointParameter *joint;
	int joint_flag=0,flag=0;
	/* only check active */
	for (joint_num = 0; joint_num < 6; joint_num++)
	{
		joint = &joints[joint_num];

		joint_flag = 0;
		SET_JOINT_ERROR_FLAG(joint, 0);
		
		if (GET_JOINT_ENABLE_FLAG(joint))
		{
			/* check for excessive following error */
			if (GET_JOINT_FERROR_FLAG(joint))
			{
				if (!GET_JOINT_ERROR_FLAG(joint))
				{
					/* report the error just this once */
			        }
				SET_JOINT_ERROR_FLAG(joint, 1);
				flag |=2;
				break;
			}
		}
 		if (joint_flag)
		{
			SET_JOINT_ERROR_FLAG(joint, 1);
		}
		else
		{
			SET_JOINT_ERROR_FLAG(joint, 0);
		}
	}

	if (flag)
	{
		
		printf("do we disable it here?\n");
		emcmotDebug->enabling = 0;
		SET_MOTION_ERROR_FLAG(1);
	}
	else
	{
		SET_MOTION_ERROR_FLAG(0);
	}
}
void setMotionState()
{
	int joint_num;
	MotionJointParameter *joint;
	int axis;
	unsigned short status;
	int error = 0;
	if (!emcmotDebug->enabling && !GET_MOTION_ENABLE_FLAG())
	{
		for (joint_num = 0; joint_num < 6; joint_num++)
		{
			/* point to joint data */
			joint = &joints[joint_num];
			SET_JOINT_ENABLE_FLAG(joint, 0);	
		}
	}
	for(joint_num = 0; joint_num < 6; joint_num++)
	{
		//if((1==g_servo[joint_num]->alarm)||(1==g_servo[joint_num]->fault))
	//	if(1==g_servo[joint_num]->fault)
	//	{
	//		printf("servo alarmed !!!!!!\r\n");
	//		error=1;
	//		emcmotStatus->linkErr=1;
	//	}			
	}
	if ((!emcmotDebug->enabling && GET_MOTION_ENABLE_FLAG())||1==error)
	{
		//调用接口使伺服下使能
		for(joint_num=0;joint_num<6;joint_num++)
		{	
			ec_slave_enable(joint_num,0);
			//Servo_Enable(g_servo[joint_num],0);
			printf("Motion : axis %d disabled \n",joint_num);
		}
		
		 //clear out the motion emcmotDebug->queue and interpolators 
		tpAbort(&emcmotDebug->queue);
		tpClear(&emcmotDebug->queue);
//		tpSetPos(&emcmotDebug->queue, emcmotStatus->carte_pos_cmd);
		tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);
		for (joint_num = 0; joint_num < 6; joint_num++)
		{
			joint = &joints[joint_num];
			SET_JOINT_ENABLE_FLAG(joint, 0);
		}	
		emcmotStatus->enble=0;
		SET_MOTION_ENABLE_FLAG(0);
	}
   	 if(emcmotDebug->enabling && !GET_MOTION_ENABLE_FLAG()) 
	 {
    	for(joint_num=0;joint_num<6;joint_num++)
		{
			ec_slave_enable(joint_num,1);
			//Servo_Enable(g_servo[joint_num],1);
			printf("Motion : axis %d enabled \n",joint_num);
		}
    	tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);	
		for (joint_num = 0; joint_num < 6; joint_num++)
		{
			// point to joint data 
			joint = &joints[joint_num];
			SET_JOINT_ENABLE_FLAG(joint, 1);
			SET_JOINT_ERROR_FLAG(joint, 0);
		}
		emcmotStatus->enble=1;
		SET_MOTION_ENABLE_FLAG(1);
		SET_MOTION_ERROR_FLAG(0);
		enbl_flag = 1;
	}
	 if(emcmotDebug->playing&& !GET_MOTION_PLAY_FLAG()) 
	{
		if(!GET_MOTION_INPOS_FLAG())
		{
			emcmotStatus->playing=0;
			SET_MOTION_PLAY_FLAG(0);
	        }
		else
		{	
			
			tpClear(&emcmotDebug->queue);
					
			/* preset traj planner to current position */
			//tpSetPos(&emcmotDebug->queue, emcmotStatus->carte_pos_cmd);
			tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);	
	              /* drain the cubics so they'll synch up */
			SET_MOTION_PLAY_FLAG(1);
			SET_MOTION_ERROR_FLAG(0);
			emcmotStatus->playing=1;
		} 
    	}
	 if (!emcmotDebug->playing&& GET_MOTION_PLAY_FLAG()) 
	{
		if(GET_MOTION_INPOS_FLAG()||(emcmotStatus->pause==1))
		{
			
			tpClear(&emcmotDebug->queue);
			printf("so we clear it here?\n");

	//		tpSetPos(&emcmotDebug->queue, emcmotStatus->carte_pos_cmd);
			tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);	
			SET_MOTION_PLAY_FLAG(0);
			SET_MOTION_ERROR_FLAG(0);
			emcmotStatus->playing=0;
		 	tpResume(&emcmotDebug->queue);
		} 
		else 
		{
			/* not in position-- don't honor mode change */
			emcmotStatus->playing=1;
			SET_MOTION_PLAY_FLAG(1);
		}
	}   
	if (!GET_MOTION_ENABLE_FLAG()) 
	{
		emcmotStatus->motion_state = EMCMOT_MOTION_DISABLED;
	} 
	else if (GET_MOTION_PLAY_FLAG()) 
	{
	        emcmotStatus->motion_state = EMCMOT_MOTION_PLAY;
    	} 
	else 
	{
	        emcmotStatus->motion_state = EMCMOT_MOTION_TEACH;
	}
}




int getCommandPosition()
{
//change nothing now, but eventually......

	int joint_num;
	MotionJointParameter *joint=NULL;
	double positions[MAX_JOINTS];
	double pre_joints[MAX_JOINTS];
	int onlimit, jointOnLimit;
	RobotPose world = { 0 };
	int fflag;
	static int old_type,count=0;
	PmHomogeneous hom;
	PmRotationMatrix RS;
	PmPose worldPose;
	PmQuaternion rotation;
	PmCartesian Pm;
	PmRpy feedbackRpy;
	int i=0;
	switch ( emcmotStatus->motion_state) 
	{
		case EMCMOT_MOTION_TEACH:
	    	case EMCMOT_MOTION_PLAY:
			//printf("pre.emcmotStatus->joint_cmd.j6=%f \r\n", emcmotStatus->joint_cmd.j4);


			tpRunCycle(&emcmotDebug->queue,(int)(emcmotDebug->queue.cycleTime*1000000000));
			
			
			if((emcmotStatus->joint_type==1)||(emcmotStatus->joint_type==0))
			{
				//printf("######### joint_type = %d  free = %d ########\n",emcmotStatus->joint_type,emcmotDebug->queue.free);
				emcmotStatus->joint_cmd = tpGetJPos(&emcmotDebug->queue);
				//printf("ext.emcmotStatus->joint_cmd.j6=%f \r\n", emcmotStatus->joint_cmd.j4);
				positions[0]=emcmotStatus->joint_cmd.j0;
				positions[1]=emcmotStatus->joint_cmd.j1;
				positions[2]=emcmotStatus->joint_cmd.j2;
				positions[3]=emcmotStatus->joint_cmd.j3;
				positions[4]=emcmotStatus->joint_cmd.j4;
				positions[5]=emcmotStatus->joint_cmd.j5;
				positions[6]=emcmotStatus->joint_cmd.j6;
				positions[7]=emcmotStatus->joint_cmd.j7;
				kinematicsForward(positions,&world,&Rtmp,&fflags,&iflags);
				emcmotStatus->carte_pos_cmd.tran=world.tran;
				emcmotStatus->carte_pos_cmd.a=world.a;
				emcmotStatus->carte_pos_cmd.b=world.b;
				emcmotStatus->carte_pos_cmd.c=world.c;
				emcmotDebug->currentFrame.rot=Rtmp;
			}
			else if (emcmotStatus->joint_type==2)
			{
				//printf("######### getCommandPosition free = %d ########\n",emcmotDebug->queue.free);
				emcmotStatus->carte_pos_cmd = tpGetPos(&emcmotDebug->queue);
				if(emcmotDebug->queue.free==1)
				{
					if(4==emcmotStatus->coordinate_type)
					{
						Vector.s = emcmotStatus->carte_pos_cmd.a*PM_PI/180;
						if(fabs(Vector.s)>0.000001)
						{
							PmCartesian center;
							PmCartesian temp;
							pmRotQuatConvert(Vector, &rotation);
							pmQuatNorm(rotation, &rotation);
							pmQuatMatConvert(rotation, &RS);
							pmMatCartMult(RS, emcmotDebug->robot_RT.x, &hom.rot.x);
							pmMatCartMult(RS, emcmotDebug->robot_RT.y, &hom.rot.y);
							pmMatCartMult(RS, emcmotDebug->robot_RT.z, &hom.rot.z);

							

							pmMatMatMult(hom.rot, emcmotDebug->robot_invtcf,  &hom.rot);//旋转后末端坐标系
							pmMatCartMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcp,&center);

							pmCartCartAdd(emcmotDebug->robot_P0, center, &center);

							pmMatCartMult(hom.rot,emcmotDebug->robot_tcp,&temp);
							pmCartCartSub(center,temp, &(world.tran));


							
						//	tempCircle.rStart=emcmotDebug->robot_P0;

						//	pmMatCartMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcp,&(tempCircle.center));

						//	pmCartCartAdd(emcmotDebug->robot_P0, tempCircle.center, &(tempCircle.center));
						//	tempCircle.normal.x = Vector.x + tempCircle.center.x;
						//	tempCircle.normal.y = Vector.y + tempCircle.center.y;
						//	tempCircle.normal.z = Vector.z + tempCircle.center.z;
						//	pm3DCirclePoint(&tempCircle,Vector.s,&worldPose);
							
						//	world.tran = worldPose.tran;

							//pmMatCartMult(hom.rot,emcmotDebug->robot_tcp,&Pm);
							//pmCartScalMult(Pm, -1.0, &Pm);
							//pmCartCartAdd(emcmotStatus->carte_pos_cmd.tran,Pm,&world.tran);

							emcmotStatus->carte_pos_cmd.tran=world.tran;
							emcmotStatus->carte_pos_cmd.a=world.a;
							emcmotStatus->carte_pos_cmd.b=world.b;
							emcmotStatus->carte_pos_cmd.c=world.c;
							hom.tran = world.tran;
							
						}
						else
						{
							world.tran = emcmotStatus->carte_pos_cmd.tran;
							hom.rot=emcmotDebug->robot_R0;
							hom.tran=world.tran;
						}
					}
					else
					{
						world.tran = emcmotStatus->carte_pos_cmd.tran;
						world.a = emcmotStatus->carte_pos_cmd.a;
						world.b = newCorse(emcmotStatus->carte_pos_cmd.b);
						world.c = newCorse(emcmotStatus->carte_pos_cmd.c);
						
						Vector.s=world.a*PM_PI/180;
						//printf("######### getCommandPosition s=%f ########",Vector.s=world.a*PM_PI/180);
						pmRotQuatConvert(Vector, &rotation);
						pmQuatNorm(rotation, &rotation);
						pmQuatMatConvert(rotation, &RS);
						pmMatCartMult(RS, emcmotDebug->robot_R0.x, &hom.rot.x);
						pmMatCartMult(RS, emcmotDebug->robot_R0.y, &hom.rot.y);
						pmMatCartMult(RS, emcmotDebug->robot_R0.z, &hom.rot.z);
						hom.tran = world.tran;
						worldPose.tran = world.tran;
					}
				}
				else
				{
					world.tran = emcmotStatus->carte_pos_cmd.tran;
					world.a = emcmotStatus->carte_pos_cmd.a;
		//			world.a = newCorse(emcmotStatus->carte_pos_cmd.a);
					world.b = newCorse(emcmotStatus->carte_pos_cmd.b);
					world.c = newCorse(emcmotStatus->carte_pos_cmd.c);
					emcmotDebug->queue.Vector.s = world.a*PM_PI/180;
					//printf("######### getCommandPosition s=%f x= %f  y = %f z = %f   ######## \n",
					//	emcmotDebug->queue.Vector.s,emcmotDebug->queue.Vector.x,emcmotDebug->queue.Vector.y,emcmotDebug->queue.Vector.z);
					pmRotQuatConvert(emcmotDebug->queue.Vector, &rotation);
					pmQuatNorm(rotation, & rotation);
					pmQuatMatConvert(rotation, &RS);
					pmMatCartMult(RS, emcmotDebug->queue.robot_R0.x, &hom.rot.x);
					pmMatCartMult(RS, emcmotDebug->queue.robot_R0.y, &hom.rot.y);
					pmMatCartMult(RS, emcmotDebug->queue.robot_R0.z, &hom.rot.z);
					hom.tran = world.tran;
					worldPose.tran = world.tran;
					//printf(" hom.rot.x = %f %f %f   y= %f  %f  %f  z= %f  %f  %f \n",
				//		hom.rot.x.x,hom.rot.x.y,hom.rot.x.z,
				//		hom.rot.y.x,hom.rot.y.y,hom.rot.y.z,
				//		hom.rot.z.x,hom.rot.z.y,hom.rot.z.z
				//	);
					
				}	
				for (joint_num = 0; joint_num < num_joints; joint_num++)
				 {
					/* point to joint data */
					joint = &joints[joint_num];
					pre_joints[joint_num] = joint->pos_cmd;
				}
				//printf("x =%f ,y=%f ,z=%f  a=%f b=%f c=%f \n",
				//	world.tran.x,world.tran.y,world.tran.z,world.a,world.b,world.c);
				//x=world.tran.x;
				//y=world.tran.y;
				//z=world.tran.z;

				Rtmp=hom.rot;
				if(emcmotDebug->queue.motionType==2)
				{
					JointPoint joint;
					JointPoint preJoint;
					PmHomogeneous point;
					preJoint.j1=pre_joints[0];
					preJoint.j2=pre_joints[1];
					preJoint.j3=pre_joints[2];
					preJoint.j4=pre_joints[3];
					preJoint.j5=pre_joints[4];
					preJoint.j6=pre_joints[5];
					point.tran=world.tran;
					point.rot=hom.rot;

					//printf("preJoint.j1 = %lf  preJoint.j2 = %lf preJoint.j3 = %lf preJoint.j4 = %lf preJoint.j5 = %lf preJoint.j6 = %lf\n",
					//		preJoint.j1, preJoint.j2, preJoint.j3, preJoint.j4, preJoint.j5, preJoint.j6);
	
					fflag=CTRL_GetTCPInJointInMatrix(&joint,&preJoint,&point,&(emcmotDebug->queue.usr),&(emcmotDebug->queue.tool));
					positions[0]=joint.j1;
					positions[1]=joint.j2;
					positions[2]=joint.j3;
					positions[3]=joint.j4;
					positions[4]=joint.j5;
					positions[5]=joint.j6;

					//for (int i = 0; i < 6; i++)
					//{
					//	printf("positions[%d] = %lf   ", i, positions[i]);
					//}
					//printf("\n");
				}

				else
				{
					fflag = kinematicsInverse(&world, &hom,positions,pre_joints,fflags);
				}
				if (fflag)
				{
					printf("kinematicsInverse failed \n");
				}
				else
				{
					for (joint_num = 0; joint_num < 6; joint_num++)
					{
						if (fabs(positions[joint_num]-pre_joints[joint_num])>(emcmotConfig->jointMoveMaxVel[joint_num]*emcmotDebug->queue.cycleTime))
						{
						
							//printf("axis %d exeed the speed limit  pre=%f next=%f \r\n",joint_num,pre_joints[joint_num],positions[joint_num]);
							//double slope;
							//slope=0.02*(emcmotConfig->jointMoveMaxVel[joint_num]*emcmotDebug->queue.cycleTime)/fabs(positions[joint_num]-pre_joints[joint_num]);
							//for(int i=0;i<6;i++)
							//{
							//	positions[i]=slope*(positions[i]-pre_joints[i])+pre_joints[i];
							//}
							fflag=1;
							//break;
						}
					}
					
				}
				if (fflag)
				{
			//		over_vel = 1;
					tpAbort(&emcmotDebug->queue);
							
					tpClear(&emcmotDebug->queue);
//					tpSetPos(&emcmotDebug->queue, emcmotStatus->carte_pos_cmd);
//					tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);	
					for (joint_num = 0; joint_num < num_joints; joint_num++)
					{
						joint = &joints[joint_num];
						joint->cur_vel =  joint->vel_cmd;
						positions[joint_num] = joint->pos_cmd;
					}
					reportErrorType(MOTION_ERROR_KINEMATICSINVERSE_FAILED_TYPE);
					//return -1;
				}
				emcmotDebug->currentFrame.rot=hom.rot;
		//		free_tprun();
			}
			for (joint_num = 0; joint_num < num_joints; joint_num++)
			{
				joint = &joints[joint_num];
				if(8==joint_num) break;
				
				joint->pos_cmd =positions[joint_num];
				switch(joint_num)
				{
					case 0:
						emcmotStatus->joint_cmd.j0=joint->pos_cmd;
						break;
					case 1:
						emcmotStatus->joint_cmd.j1=joint->pos_cmd;

						break;
					case 2:
						emcmotStatus->joint_cmd.j2=joint->pos_cmd;

						break;
					case 3:
						emcmotStatus->joint_cmd.j3=joint->pos_cmd;
						break;
					case 4:
						emcmotStatus->joint_cmd.j4=joint->pos_cmd;

						break;
					case 5:
						emcmotStatus->joint_cmd.j5=joint->pos_cmd;
						break;
					default:
						break;
				}
	       	}
			SET_MOTION_INPOS_FLAG(0);
	        	if (tpIsDone(&emcmotDebug->queue))
			{
				if(interpList->motionState!=COMMAND_ERROR)
					interpList->motionState=COMMAND_DONE;
				SET_MOTION_INPOS_FLAG(1);
			}
	        	break;
		case EMCMOT_MOTION_DISABLED:
			emcmotStatus->carte_pos_cmd = emcmotStatus->carte_pos_fb;	
			for (joint_num = 0; joint_num < num_joints; joint_num++)
			{
				joint = &joints[joint_num];
				if(8==joint_num) break;
				joint->pos_cmd = joint->pos_fb;
				joint->vel_cmd = 0.0;
			}
			emcmotStatus->joint_cmd= emcmotStatus->joint_fb;
			tpSetJPos(&emcmotDebug->queue, emcmotStatus->joint_cmd);	
			emcmotStatus->distance_to_go = 0.0;    // 2010.10.22
			SET_MOTION_INPOS_FLAG(1);
			if(interpList->motionState!=COMMAND_ERROR)
				interpList->motionState=COMMAND_DONE;
			break;
	   	 default:
	        	break;
	}
	onlimit = 0;
	for (joint_num = 0; joint_num < num_joints; joint_num++)
	{
	/* point to joint data */
		joint = &joints[joint_num];
		if(8==joint_num) 
			break;
		jointOnLimit = 0;
		if(GET_JOINT_HOMED_FLAG(joint))
		{
			if (joint->pos_cmd > joint->max_jog_limit)
			{
				SET_JOINT_ERROR_PSL_FLAG(joint, 1);
				jointOnLimit = 1;
				onlimit = 1;
			}
			else
			{
				SET_JOINT_ERROR_PSL_FLAG(joint, 0);
			}
			if (joint->pos_cmd < joint->min_jog_limit)
			{
				jointOnLimit = 1;
				onlimit = 1;
				SET_JOINT_ERROR_NSL_FLAG(joint, 1);
			}
			else
			{
				SET_JOINT_ERROR_NSL_FLAG(joint, 0);
			}
		}
	}

	if (onlimit && ! emcmotStatus->on_soft_limit)
	{	
		for (joint_num = 0; joint_num < num_joints; joint_num++)
		{
			/* point to joint data */
			joint = &joints[joint_num];
		}
	        tpAbort(&emcmotDebug->queue);
		//	 tpClear(&emcmotDebug->queue);		 
	}
	emcmotStatus->on_soft_limit = onlimit;
	pmMatRpyConvert(Rtmp,&feedbackRpy);
	emcmotStatus->carte_pos_cmd.tran=world.tran;
	emcmotStatus->carte_pos_cmd.a=feedbackRpy.r*180.0/PM_PI;
	emcmotStatus->carte_pos_cmd.b=feedbackRpy.p*180.0/PM_PI;
	emcmotStatus->carte_pos_cmd.c=feedbackRpy.y*180.0/PM_PI;
	
	return 0;
}
int writeOutPosition()
{
	int joint_num;
	MotionJointParameter *joint;
	s64 outPutPulse=0;

	static s64 lastPulse=0;

	
	for (joint_num = 0; joint_num < 6; joint_num++)
	{
		joint = &joints[joint_num];
		if(((joint->pos_cmd-joint->old_pos_cmd>0.0)&&(joint->pos_cmd>joint->max_jog_limit))
			||((joint->pos_cmd-joint->old_pos_cmd<0.0)&&(joint->pos_cmd<joint->min_jog_limit)))
		{
			reportErrorType(MOTION_ERROR_OUTPUT_OUTOF_SOFT_LIMIT_TYPE);
		//	printf("axis %d exeed the limit pre=%f cmd=%f \r\n",
		//		joint_num,joint->old_pos_cmd,joint->pos_cmd);
			tpAbort(&emcmotDebug->queue);
			//return -1;
		}
		if(1==emcmotDebug->linkSimTest)
		{
			joint->pos_fb = joint->pos_cmd;
			switch(joint_num)
			{
				case 0:
					emcmotStatus->joint_fb.j0=joint->pos_cmd;
					break;
				case 1:
					emcmotStatus->joint_fb.j1=joint->pos_cmd;
					break;
				case 2:
					emcmotStatus->joint_fb.j2=joint->pos_cmd;
					break;
				case 3:
					emcmotStatus->joint_fb.j3=joint->pos_cmd;
					break;
				case 4:
					emcmotStatus->joint_fb.j4=joint->pos_cmd;
					break;
				case 5:
					emcmotStatus->joint_fb.j5=joint->pos_cmd;
					break;
				default:
					break;
			}
		}
		else
		{
			if(5==joint_num&&emcmotConfig->coupleRate!=0)
			//if(0)
			{
				outPutPulse=emcmotConfig->encodeOriginOffset[joint_num]+(s64)((joint->pos_cmd+joints[4].pos_cmd/emcmotConfig->coupleRate)*emcmotConfig->servoPulsePerRound[joint_num]/360*emcmotConfig->mechTransRatio[joint_num]);
			}
			else
			{
				outPutPulse=emcmotConfig->encodeOriginOffset[joint_num]+(s64)(joint->pos_cmd*emcmotConfig->servoPulsePerRound[joint_num]/360*emcmotConfig->mechTransRatio[joint_num]);
			}				
		}
		if(1==emcmotConfig->encoderType[joint_num]&&emcmotStatus->enble&&interpList->motionState==COMMAND_EXEC)
		{
		
			//Servo_CSPTo(g_servo[joint_num],outPutPulse);
			ec_slave_csp_target_position(joint_num,(int32_t)outPutPulse);	
		}
		else
		{
			// do nothing by now.
		}
		joint->vel_cmd = (joint->pos_cmd - joint->old_pos_cmd) * servo_freq;
		joint->old_pos_cmd=joint->pos_cmd;
	}
	//printf("robotJ0=%f \n", robotJ0);
	return 0;
}
int updateStatus()
{
	long int ret = 0;
	int i=0;
	// ret = LOCK(Mutex);
	LOCK(Mutex);
	if (1)	// (0 == ret)
	{
		//printf("A");
//		PUMA_A1 = emcmotConfig->PUMA_A[0];
//		PUMA_A2	= emcmotConfig->PUMA_A[1];
//		PUMA_A3	= emcmotConfig->PUMA_A[2];
//		PUMA_D4	= emcmotConfig->PUMA_A[3];
//		PUMA_D6	= emcmotConfig->PUMA_A[4];

		motionFb->jointCmd.j1 = 0.0;
		motionFb->jointCmd.j2 = 0.0;
		motionFb->jointCmd.j3 = 0.0;
		motionFb->jointCmd.j4 = 0.0;
		motionFb->jointCmd.j5 = 0.0;
		motionFb->jointCmd.j6 = 0.0;

		motionFb->linkError=emcmotStatus->linkErr;
		
		motionFb->motionState=interpList->motionState;
		motionFb->errCode=errorCode;
		motionFb->commandStatus = emcmotStatus->commandStatus;
		motionFb->distance_to_go = emcmotStatus->distance_to_go;
		motionFb->distance_was_gone = emcmotStatus->distance_was_gone;
		motionFb->jointCmd.j1 = joints[0].pos_cmd;
		motionFb->jointCmd.j2 = joints[1].pos_cmd;
		motionFb->jointCmd.j3 = joints[2].pos_cmd;
		motionFb->jointCmd.j4 = joints[3].pos_cmd;
		motionFb->jointCmd.j5 = joints[4].pos_cmd;
		motionFb->jointCmd.j6 = joints[5].pos_cmd;
		//printf("j1=%f j2=%f j3=%f j4=%f j5=%f j6=%f \r\n", motionFb->jointCmd.j1, motionFb->jointCmd.j2, motionFb->jointCmd.j3, motionFb->jointCmd.j4, motionFb->jointCmd.j5, motionFb->jointCmd.j6);

		motionFb->jointFeedback.j1 = joints[0].pos_fb;
		motionFb->jointFeedback.j2 = joints[1].pos_fb;
		motionFb->jointFeedback.j3 = joints[2].pos_fb;
		motionFb->jointFeedback.j4 = joints[3].pos_fb;
		motionFb->jointFeedback.j5 = joints[4].pos_fb;
		motionFb->jointFeedback.j6 = joints[5].pos_fb;

		motionFb->positionCmd.x=emcmotStatus->carte_pos_cmd.tran.x;
		motionFb->positionCmd.y=emcmotStatus->carte_pos_cmd.tran.y;
		motionFb->positionCmd.z=emcmotStatus->carte_pos_cmd.tran.z;
		motionFb->positionCmd.a=emcmotStatus->carte_pos_cmd.a;
		motionFb->positionCmd.b=emcmotStatus->carte_pos_cmd.b;
		motionFb->positionCmd.c=emcmotStatus->carte_pos_cmd.c;
		
		motionFb->errCode=errorCode;
		//CTRL_GetTCPInUserSpace(&motionFb.usrCmd,joints,&motionFb.user,&motionFb.tool);

		for(i=0;i<6;i++)
		{
			motionFb->enable[i]=GET_MOTION_ENABLE_FLAG();
			//motionFb.servoEncoderValue[i]=g_servo[i]->currentPosition;
			//motionFb.servoEncoderDestination[i]=g_servo[i]->destination;
			//motionFb.enable[i]=g_servo[i]->enabled;
			//motionFb.servoEncoderValue[i]=;
			//motionFb.servoEncoderDestination[i]=g_servo[i]->destination;
			//motionFb.enable[i]=g_servo[i]->enabled;
		}
		motionFb->onSoftLimit = emcmotStatus->on_soft_limit;
		motionFb->queueFull = tcqFull(&(emcmotDebug->queue.queue));
		motionFb->lineNumber = tpGetExecId(&(emcmotDebug->queue));
		UNLOCK(Mutex); 
		//printf("x=%f y=%f z=%f a=%f b=%f c=%f \n",emcmotStatus->carte_pos_cmd.tran.x, emcmotStatus->carte_pos_cmd.tran.y,
		//	emcmotStatus->carte_pos_cmd.tran.z, emcmotStatus->carte_pos_cmd.a, emcmotStatus->carte_pos_cmd.b, emcmotStatus->carte_pos_cmd.c
		//	);
	}
	else
	{
		//printf("C");
		return ret;
	}
	return 0;
}
int analyzePostion()
{
//set the cycle time
	
	traj_period =CALCULATECYCLETIME*0.000000001;
	traj_freq = 1.0 / traj_period;
	servo_period =CALCULATECYCLETIME*0.000000001;
	servo_freq = 1.0 / servo_period;

	//读取输入
	readInputPosition();
	//检查运动是否出错
	checkForError();
	//变更运动状态
	setMotionState();
	//计算位置
	getCommandPosition();
	//写输出
	writeOutPosition();
	updateStatus();

	return 0;
}

