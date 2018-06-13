#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>


#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>




//#include "contrller.h"
#include "usr_motion_api.h"
#include "language_struct.h"
#include "read_file.h"
int done;
//extern MotionConfig *emcmotConfig ;


void exitMain(int sig)
{
	done=0;
	//CTRL_USR_Exit();
}
int main(int argc,char ** argv)
{
	done=1;
	signal(SIGINT,exitMain);
	MotionFeedback fb;
	int configShmID;
	int axis;


	CTRL_USR_Init();
	//printf("config->teachLinearV=%f\n",emcmotConfig->teachLinearV);

	while(done)
	{
		/*
		CTRL_GetMotionStatus(&fb);
		printf("get the status done\n");
		if(fb.enable[0]==1)
		{
			CTRL_ServoEnable(0);
		}
		else
		{
			CTRL_ServoEnable(1);
		}
		*/
		char buf[256];
		char* p=buf;
		char ch;
		for(;;)
		{
			ch=getchar();
			if(ch=='\r' || ch=='\n')
			{
				putchar('\r');
				putchar('\n');
				if(p==buf)
				{
					printf("you have to input sth ,right ? \n");
				}
				*p++ = 0;
				break;
			}
			else if(ch==-32)
			{
				ch = getchar();
				continue;
			}
			putchar(ch);
			*p++ = ch;	
		}
		if(strncmp(buf,"EN",2)==0)
		{
			CTRL_ServoEnable(1);
		}
		else if(strncmp(buf,"DIS",3)==0)
		{
			CTRL_ServoEnable(0);
		}	
		else if(strcmp(buf,"C")==0)
		{
			MotionFeedback fb;
			CTRL_GetMotionStatus(&fb);
			printf("j1=%f j2=%f j3=%f j4=%f j5=%f j6=%f \r\n",
				fb.jointCmd.j1,fb.jointCmd.j2,fb.jointCmd.j3,fb.jointCmd.j4,fb.jointCmd.j5,fb.jointCmd.j6);
			printf("motionState=%d linkerr=%d  \r\n",fb.motionState,fb.linkError);
			printf("x =%f y=%f z=%f a=%f b=%f c=%f \r\n ",
			fb.positionCmd.x,fb.positionCmd.y,fb.positionCmd.z,fb.positionCmd.a,fb.positionCmd.b,fb.positionCmd.c);
			for(axis=0;axis<6;axis++)
			{
				//printf("servoEncoderValue[%d]=%lld \r\n",axis,fb.servoEncoderValue[axis]);		
			}
		}	
		else if(strncmp(buf,"ABS ",4)==0)
		{
			double value=0.0;
			if(strncmp(&buf[4],"J1 ",3)==0)
			{
				axis=1;
			}
			else if(strncmp(buf+4,"J2 ",3)==0)
			{
				axis=2;
			}
			else if(strncmp(buf+4,"J3 ",3)==0)
			{
				axis=3;
			}
			else if(strncmp(buf+4,"J4 ",3)==0)
			{
				axis=4;
			}
			else if(strncmp(buf+4,"J5 ",3)==0)
			{
				axis=5;
			}
			else if(strncmp(buf+4,"J6 ",3)==0)
			{
				axis=6;
			}
			else	
			{
				axis=-1;
			}
			value=atof(buf+7);
			printf("ABS MOVE   axis =%d value =%f \r\n",axis,value);
			if(1)
			{
				SingleAxisMove singleMove;
				singleMove.axis=axis;
				singleMove.direction=MOVE_DIR_NEGATIVE;
				singleMove.endPoint=value;
				singleMove.vel=0.05;
				{
					singleMove.coordinate.coordinateType=COORDINATE_TYPE_JOINT;
				}
				
				CTRL_AddSingleAxisMove(&singleMove);
			}
		}
		else if(strncmp(buf,"INC ",4)==0)
		{
			double value=0.0;
			int type=0;
			if(strncmp(&buf[4],"J1 ",3)==0)
			{
				axis=1;
			}
			else if(strncmp(buf+4,"J2 ",3)==0)
			{
				axis=2;
			}
			else if(strncmp(buf+4,"J3 ",3)==0)
			{
				axis=3;
			}
			else if(strncmp(buf+4,"J4 ",3)==0)
			{
				axis=4;
			}
			else if(strncmp(buf+4,"J5 ",3)==0)
			{
				axis=5;
			}
			else if(strncmp(buf+4,"J6 ",3)==0)
			{
				axis=6;
			}
			else if(strncmp(buf+4,"PX ",3)==0)
			{
				axis=1;
				type=1;
			}
			else if(strncmp(buf+4,"PY ",3)==0)
			{
				axis=2;
				type=1;
			}
			else if(strncmp(buf+4,"PZ ",3)==0)
			{
				axis=3;
				type=1;
			}
			else if(strncmp(buf+4,"PA ",3)==0)
			{
				axis=4;
				type=1;
			}
			else if(strncmp(buf+4,"PB ",3)==0)
			{
				axis=5;
				type=1;
			}	
			else if(strncmp(buf+4,"PC ",3)==0)
			{
				axis=6;
				type=1;
			}			
	
			else
			{
				axis=-1;
			}
			value=atof(buf+7);
			
			
			if(value<=200.0&&value>=-200.0)
			{
				MotionFeedback fb;

				LinearDescartesMoveInformation DL;
				CTRL_GetMotionStatus(&fb);
				DL.endPoint.x=fb.positionCmd.x;
				DL.endPoint.y=fb.positionCmd.y;
				DL.endPoint.z=fb.positionCmd.z;
				if(0==type)
				{
					switch(axis)
					{
						case 1:
							value+=fb.jointCmd.j1;
							break;
						case 2:
							value+=fb.jointCmd.j2;
							break;
						case 3:
							value+=fb.jointCmd.j3;
							break;
						case 4:
							value+=fb.jointCmd.j4;
							break;
						case 5:
							value+=fb.jointCmd.j5;
							break;
						case 6:
							value+=fb.jointCmd.j6;
							break;
						default:
							printf("input axis error axis=%d\n",axis);
							break;
					}
				}
				if(1==type)
				{
					switch(axis)
					{
						case 1:
							value+=fb.positionCmd.x;
							break;
						case 2:
							value+=fb.positionCmd.y;
							break;
						case 3:
							value+=fb.positionCmd.z;
							break;
						case 4:
							value+=fb.positionCmd.a;
							break;
						case 5:
							value+=fb.positionCmd.b;
							break;
						case 6:
							value+=fb.positionCmd.c;
							break;
						default:
							printf("input axis error axis=%d\n",axis);
							break;
					}
				}
				SingleAxisMove singleMove;
				singleMove.axis=axis;
				singleMove.direction=MOVE_DIR_NEGATIVE;
				singleMove.endPoint=value;
				singleMove.vel=0.05;
				if(1==type)
				{
					singleMove.coordinate.coordinateType=COORDINATE_TYPE_LINEAR;
				}
				else
				{
					singleMove.coordinate.coordinateType=COORDINATE_TYPE_JOINT;
				}
				printf("INC MOVE   axis =%d value =%f \r\n",axis,value);
				CTRL_AddSingleAxisMove(&singleMove);
			}
		}	
		else if(strcmp(buf,"S")==0)
		{
			CTRL_MovementStop();
		}
		else
		{	
			int ret=0;
			Line line;
			int i=0;
			JointMoveInformation jm;
			LinearDescartesMoveInformation DL;
			MotionFeedback fb;
			CTRL_GetMotionStatus(&fb);

			ret=readLine(buf,&line);
			
			//printf("ret=%d \n",ret);
//			printf("cmd=%d j1=%f j2=%f j3=%f j4=%f j5=%f j6=%f \n",
//			line.cmdType,line.pos.jp.joint[0],line.pos.jp.joint[1],line.pos.jp.joint[2],
//			line.pos.jp.joint[3],line.pos.jp.joint[4],line.pos.jp.joint[5]);
//			printf(" x=%f y=%f z=%f a=%f b=%f c=%f \n",
//			line.pos.p.x,line.pos.p.y,line.pos.p.z,line.pos.p.a,line.pos.p.b,line.pos.p.c);
//			printf(" VJ=%f VL=%F VR=%F \n",
//			line.vel.jointVelocity,line.vel.linearVelocity,line.vel.angularVelocity);
			switch(line.cmdType)
			{
				case C_MOVJ:
					jm.acc=1.0;
					jm.dec=1.0;
					if(line.pos.jp.joint[0]>=ABNORMAL_LARGE)
					{
						jm.endPoint.j1=fb.jointCmd.j1;
					}
					else
					{
						jm.endPoint.j1=line.pos.jp.joint[0];
					}
					if(line.pos.jp.joint[1]>=ABNORMAL_LARGE)
					{
						jm.endPoint.j2=fb.jointCmd.j2;
					}
					else
					{
						jm.endPoint.j2=line.pos.jp.joint[1];
					}
					if(line.pos.jp.joint[2]>=ABNORMAL_LARGE)
					{
						jm.endPoint.j3=fb.jointCmd.j3;
					}
					else
					{
						jm.endPoint.j3=line.pos.jp.joint[2];
					}
					if(line.pos.jp.joint[3]>=ABNORMAL_LARGE)
					{
						jm.endPoint.j4=fb.jointCmd.j4;
					}
					else
					{
						jm.endPoint.j4=line.pos.jp.joint[3];
					}
					if(line.pos.jp.joint[4]>=ABNORMAL_LARGE)
					{
						jm.endPoint.j5=fb.jointCmd.j5;
					}
					else
					{
						jm.endPoint.j5=line.pos.jp.joint[4];
					}
					if(line.pos.jp.joint[5]>=ABNORMAL_LARGE)
					{
						jm.endPoint.j6=fb.jointCmd.j6;
					}
					else
					{
						jm.endPoint.j6=line.pos.jp.joint[5];
					}

					if(line.vel.jointVelocity>=ABNORMAL_LARGE)
					{
						jm.vJ=0.1;
					}
					else
					{
						jm.vJ=line.vel.jointVelocity;
					}
					CTRL_AddJointMove(&jm,0,0);
					break;
				case C_MOVL:
					DL.acc=1.0;
					DL.dec=1.0;
					DL.free=0;
					DL.tool.x=0;
					DL.tool.y=0;
					DL.tool.z=0;
					DL.tool.rx=0;
					DL.tool.ry=0;
					DL.tool.rz=0;

					DL.user.x=0;
					DL.user.y=0;
					DL.user.z=0;
					DL.user.a=0;
					DL.user.b=0;
					DL.user.c=0;

	
					if(line.pos.p.x>=ABNORMAL_LARGE)
					{
						DL.endPoint.x=fb.positionCmd.x;
					}
					else
					{
						DL.endPoint.x=line.pos.p.x;
					}
					
					if(line.pos.p.y>=ABNORMAL_LARGE)
					{
						DL.endPoint.y=fb.positionCmd.y;
					}
					else
					{
						DL.endPoint.y=line.pos.p.y;
					}
					
					if(line.pos.p.z>=ABNORMAL_LARGE)
					{
						DL.endPoint.z=fb.positionCmd.z;
					}
					else
					{
						DL.endPoint.z=line.pos.p.z;
					}
					
					if(line.pos.p.a>=ABNORMAL_LARGE)
					{
						DL.zyx.a=fb.positionCmd.a;
					}
					else
					{
						DL.zyx.a=line.pos.p.a;
					}
					
					if(line.pos.p.b>=ABNORMAL_LARGE)
					{
						DL.zyx.b=fb.positionCmd.b;
					}
					else
					{
						DL.zyx.b=line.pos.p.b;
					}

					if(line.pos.p.c>=ABNORMAL_LARGE)
					{
						DL.zyx.c=fb.positionCmd.c;
					}
					else
					{
						DL.zyx.c=line.pos.p.c;
					}

					if(line.vel.linearVelocity>=ABNORMAL_LARGE)
					{
						DL.vL=10.0;
					}
					else
					{
						DL.vL=line.vel.linearVelocity;
					}

					if(line.vel.angularVelocity>=ABNORMAL_LARGE)
					{
						DL.vR=10.0;
					}
					else
					{
						DL.vR=line.vel.angularVelocity;
					}					
					CTRL_AddDescartesLinearMove(&DL,0,0);		
					break;
				case C_MOVC:
					break;
				default:
					break;
				
			}

		}




		sleep(1);
	}
	CTRL_USR_Exit();
	return 0;
}

