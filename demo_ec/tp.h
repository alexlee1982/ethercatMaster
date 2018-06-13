
#ifndef TP_H
#define TP_H


#include "tc.h"

#define TP_DEFAULT_QUEUE_SIZE 32
#define TP_PURE_ROTATION_EPSILON 1e-6
#define TP_VEL_EPSILON 1e-6
#define TP_ACCEL_EPSILON 1e-6

#define MAX_FILENAME_LENGTH       255

typedef struct {
	TC_QUEUE_STRUCT queue;
	int queueSize;
	double cycleTime;
	double ini_maxvel;          /* max velocity allowed by machine 
	                           constraints (ini file) for
	subsequent moves */ 
	double jvscale;                                                                  // ! \todo FIXME - 
	double PL;
	double CR;
	int  nwait;
	int iotype;
//	int addrtype;
	int stopcondition;
	int ioaddress;
    
	int addrtype;
	int ioaddr;
	int state;

	double accScale;
	double decScale;

	double jRap;
	double wRap;
	double vRap;

	int joint_num;
	PmJoint jvLimit;
	PmJoint jvmax;
	PmJoint jaccmax;
	PmJoint jdecmax;

	double vScale;		/* feed override value */// 速度修调值// ! \todo FIXME - 

	double aMax;                                                                         // ! \todo FIXME - 
	double vMax; 
	double vLimit;		/* absolute upper limit on all vels */

	double wDotMax;		/* rotational accelleration max */             // ! \todo FIXME - 
	double wMax;		/* rotational velocity max */              // ! \todo FIXME - 
	double wLimit;		/* rotational velocity max */              // ! \todo FIXME - 
	int nextId;
	int execId;
	unsigned int fileName[MAX_FILENAME_LENGTH];                    // 文件号---11.07.25
	unsigned int execFileName[MAX_FILENAME_LENGTH];            // 当前执行的文件号
	int termCond;                                                     // ! \todo FIXME - 
	double  currentVel;
	RobotPose currentPos;
	RobotPose goalPos;
	PmHomogeneous  goalFrame;	
	PmHomogeneous  currentFrame;	
	PmJoint   currentJPos;
	PmJoint goalJPos;
	int done;
	int depth;			/* number of total queued motions */
	int activeDepth;		/* number of motions blending */
	int aborting;
	int pausing;
	int motionType;
	double tolerance;           /* for subsequent motions, stay within this
	                           distance of the programmed path during
	                           blends */
	int synchronized;       // spindle sync required for this move
	int velocity_mode; 	        /* TRUE if spindle sync is in velocity mode,
			   FALSE if in position mode */
	double uu_per_rev;          /* user units per spindle revolution */
	int dispon_flag;   //0:cancel offset; 1: Cartesian offset; 2:joint offset;
       RobotPose disp_vector;
       PmJoint disp_joint;
 	PmRotationVector Vector;
	PmRotationMatrix   robot_R0;
	PmRotationMatrix  robot_RT;
	int free;
	int cord_type;
       PmCartesian   robot_tcp;
       PmRotationMatrix   robot_tcf;
       PmRotationMatrix   robot_invtcf;
	int is_extaxis;
	UserCoordianteInformation usr;
	ToolCoordianteInformation tool;

	int decelFlag;
} TP_STRUCT;
extern int tpCreate(TP_STRUCT * tp, int _queueSize, TC_STRUCT * tcSpace);
extern int tpClear(TP_STRUCT * tp);
extern int tpInit(TP_STRUCT * tp);
extern int tpSetCycleTime(TP_STRUCT * tp, double secs);
extern int tpSetJVScale(TP_STRUCT * tp, double jvscale);
extern int tpSetVR(TP_STRUCT * tp, double vr);
extern int	tpSetRLimit(TP_STRUCT * tp, double vr);
extern int	tpSetRamax(TP_STRUCT * tp, double vr);
extern int tpSetPL(TP_STRUCT * tp, int pl);
extern int tpSetCR(TP_STRUCT * tp, double cr);
extern int tpSetStopCondition(TP_STRUCT * tp, int stopconditon);
extern int tpSetIOaddress(TP_STRUCT * tp, int ioaddress);
extern int tpSetIOType(TP_STRUCT * tp, int iotype);
extern int tpSetAddrType(TP_STRUCT * tp, int addrtype);

extern int tpSetDO(TP_STRUCT * tp, int addrtype, int ioaddr, int state);

extern int tpSetDispon(TP_STRUCT * tp,int dispon_flag);
extern int tpSetVector(TP_STRUCT * tp,    RobotPose disp_vector) ;
extern int tpSetJVector(TP_STRUCT * tp,PmJoint joint);

extern int tpSetaccScale(TP_STRUCT * tp, double accscale);
extern int tpSetdecScale(TP_STRUCT * tp, double decscale);
extern int tpSetJVmax(TP_STRUCT * tp, PmJoint pos);
extern int tpSetJVlimit(TP_STRUCT * tp, PmJoint vLimit);
extern int tpSetJaccmax(TP_STRUCT * tp, PmJoint pos);
extern int tpSetJdecmax(TP_STRUCT * tp, PmJoint pos);
extern int tpSetJPos(TP_STRUCT * tp, PmJoint pos);
extern int tpAdjustJoints(TC_STRUCT * tc);
extern int tpAdjustLine(TC_STRUCT * tc);
extern int tpAddJoints(TP_STRUCT * tp, PmJoint joint,int flag);
extern int tpAddFreeJoints(TP_STRUCT * tp, double joint,int type);
extern int tpAdd3DCircle(TP_STRUCT * tp, PmJoint end,PmJoint end2,PmJoint end3);
extern int tpAddUsrLine(TP_STRUCT * tp, PointPose end, UserCoordianteInformation usr,ToolCoordianteInformation tool);
extern int tpAdd3DUsrCircle(TP_STRUCT * tp, PointPose end, PointPose end2, PointPose end3, UserCoordianteInformation usr, ToolCoordianteInformation tool);

extern int tpAddLine(TP_STRUCT * tp, PmJoint end,int type);
extern int tpAddFreeLine(TP_STRUCT * tp, double end,int cord_type,int axis_num);
extern int tpSetVmax(TP_STRUCT * tp, double vmax);   // ! \todo FIXME - 
extern double tpGetFeedVel(TP_STRUCT * tp);
extern int tpSetVlimit(TP_STRUCT * tp, double limit);
extern int tpSetAmax(TP_STRUCT * tp, double amax);     // ! \todo FIXME - 
extern int tpSetLSAmax(TP_STRUCT * tp, double ls_aMax);
extern int tpSetId(TP_STRUCT * tp, int id);
extern int tpSetFileName(TP_STRUCT * tp, unsigned int fileName);
extern int tpGetExecId(TP_STRUCT * tp);
extern int tpSetTermCond(TP_STRUCT * tp, int cond, double tolerance);
//extern int tpAddLine(TP_STRUCT * tp, RobotPose end, int type, double vel, double
//        ini_maxvel, double acc, unsigned char enables);
extern int tpAddCircle(TP_STRUCT * tp, RobotPose end, PmCartesian center,
        PmCartesian normal, int turn, int type, double vel, double ini_maxvel,
        double acc, unsigned char enables, int *active_g_codes);
extern int tpRunCycle(TP_STRUCT * tp, long period);
extern int tpPause(TP_STRUCT * tp);
extern int tpResume(TP_STRUCT * tp);
extern int tpAbort(TP_STRUCT * tp);
extern int tpSetPos(TP_STRUCT * tp,RobotPose end);
extern int tpGetIOType(TP_STRUCT * tp);
extern int tpGetAddrtype(TP_STRUCT * tp);
extern int tpGetIOaddress(TP_STRUCT * tp);
extern int tpGetStopCondition(TP_STRUCT * tp);
extern double tpGetCurVel(TP_STRUCT * tp);
extern RobotPose tpGetPos(TP_STRUCT * tp);
extern PmJoint tpGetJPos(TP_STRUCT * tp);
extern int tpIsDone(TP_STRUCT * tp);
extern int tpGetTapAccelDecelPhase(TP_STRUCT * tp);
extern int tpGetActiveGCodes(TP_STRUCT * tp, int *active_g_codes);    
extern int tpQueueDepth(TP_STRUCT * tp);
extern int tpActiveDepth(TP_STRUCT * tp);
extern int tpGetMotionType(TP_STRUCT * tp);
extern int tpSetSpindleSync(TP_STRUCT * tp, int mode);
extern int tpSetTapParam(TP_STRUCT * tp, double uu_per_rev, int velocity_mode);
extern int tpSetTapType(TP_STRUCT * tp, int tapType);

extern int tpSetAout(TP_STRUCT * tp, unsigned char index, double start, double end);      // ! \todo FIXME - 
extern int tpSetDout(TP_STRUCT * tp, int index, unsigned char start, unsigned char end);  // ! \todo FIXME - 

extern int tpSetPosErr(TP_STRUCT * tp, double pos_err);
extern int tpGetDecelFlag(TP_STRUCT * tp);
extern int tpSetDecelFlag(TP_STRUCT * tp, int decelFlag);
extern  int tpSetTCF(TP_STRUCT * tp, PmRotationMatrix position);
extern   int tpSetInvTCF(TP_STRUCT * tp, PmRotationMatrix position);
extern int tpSetcordType(TP_STRUCT * tp, int type);
extern int tpSetTCP(TP_STRUCT * tp, PmCartesian position);
 



#endif				/* TP_H */
