/********************************************************************
* Description: tp.c
*   Trajectory planner based on TC elements
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: byf
* License: Version 2
* System: rtlinux
*
********************************************************************/
#include <string.h>
#include <math.h>
#include "stdio.h"

#include "posemath.h"
#include "tc.h"
#include "tp.h"

#include "motionConfig.h"
#include "motionIner.h"


#include "mot_priv.h" 
#include "contrller.h"
#include "motionInterfaceBuffer.h"


extern MotionJointParameter  *joints;
extern int num_joints;
//extern MotionStatus *emcmotStatus;
//extern MotionDebug *emcmotDebug;
//extern MotionConfig *emcmotConfig;
KINEMATICS_FORWARD_FLAGS fflags = 0;
KINEMATICS_INVERSE_FLAGS iflags = 0;
 //PmRotationMatrix Rtmp;
 //PmRotationVector Vector;


/***********************************************************************
*                   LOCAL FUNCTION PROTOTYPES  (called only  in tp.c)                        *
************************************************************************/

static void tcRunCycle(TC_STRUCT *tc, double *v, int *on_final_decel);
static double newEnd(double start, double end);


PmCartesian threePointCircle(PmCartesian A, PmCartesian B, PmCartesian C)
{
	PmCartesian u, v;		//求出的新坐标系X,Y方向上的向量
	PmCartesian center; 	//圆心	
	PmCartesian AB, AC; 	// AB: 向量AB， AC：向量AC	
	PmCartesian ex,ey;		//ex: 新坐标系X轴单位向量 ，ey: 新坐标系Y轴单位向量 
	PmCartesian vy,vx;			// C点做垂先和AB相交，所得的X轴上的向量。	
	double xB , xC , yC;				// AB的长度。

	
	PmCartesian tempEx,tempEy;	
	//计算 u = B - A;	
	pmCartCartSub(B, A, &u);	//计算 v = C - A;	
	pmCartCartSub(C, A, &v);	//计算 xB = norm(u)	
	pmCartMag(u,&xB);	//计算 ex = u/xB	
	pmCartUnit(u, &ex);	//vx = （v * ex）*ex;	
	pmCartCartDot(v, ex, &xC);	
	pmCartScalMult(ex, xC, &vx);	//计算 vy = v - vx;	
	pmCartCartSub(v, vx, &vy);	//计算Y轴单位向量 ey = ey/|ey|	
	pmCartUnit(vy, &ey);	//计算 yC = |vy|	
	pmCartMag(vy, &yC);	//计算center = A + 1/2 xB*ex + (-xB (xB - xC) xC + xB yC^2)/(2 xB yC)*ey	
	pmCartScalMult(ex, 0.5*xB, &tempEx);	
	pmCartCartAdd(A, tempEx, &tempEx);	
	pmCartScalMult(ey, (-1 * xB*(xB - xC)*xC + xB * yC*yC) / (2 * xB*yC), &tempEy);	
	pmCartCartAdd(tempEx, tempEy, &center);	

	return center;
}

int tcInit(TC_STRUCT *tc)
{
  
  if (0 == tc)
  {
    return -1;
  }
  tc->dotype=0;
  tc->stopcondition=-1;
  tc->ioaddress=0;
  tc->iotype=0;
  tc->addrtype=0;
  tc->is_extaxis =0;

  return 0;
}
int tpCreate(TP_STRUCT * tp, int _queueSize, TC_STRUCT * tcSpace)
{
    if (0 == tp) {
	return -1;
    }

    if (_queueSize <= 0) {
	tp->queueSize = TP_DEFAULT_QUEUE_SIZE;
    } else {
	tp->queueSize = _queueSize;
    }

    /* create the queue */
    if (-1 == tcqCreate(&tp->queue, tp->queueSize, tcSpace)) {
	return -1;
    }

    /* init the rest of our data */
    return tpInit(tp);
}


int tpClear(TP_STRUCT * tp)
{  	int i;
	tcqInit(&tp->queue);

	tp->nextId = 0;                           // todo - tpAbort() not dispose the following commented variables
	tp->execId = 0;
	for (i=0;i<MAX_FILENAME_LENGTH;i++)
	     tp->fileName[i]= '\0';

	tp->motionType = 0;
	tp->termCond = TC_TERM_COND_STOP;            // todo - 
	tp->decelFlag = 0;
	tp->tolerance = 3.0;                // todo - 
	
	tp->done = 1;
	tp->depth = tp->activeDepth = 0;
	tp->currentVel=0.0;
	tp->aborting = 0;
	tp->pausing = 0;
	tp->vScale = emcmotStatus->net_feed_scale;    //todo - tp->vScale not be used through the whole project 
	tp->synchronized = 0;
	tp->uu_per_rev = 0.0;                       // todo - 
//printf("finished the tp init \n");
    return 0;
}


// 2010.05.18 byf
int tpInit(TP_STRUCT * tp)
{
    tp->cycleTime = 0.0;
    tp->vLimit = 0.0;
    tp->vScale = 1.0;
    tp->aMax = 0.0;
    tp->vMax = 0.0;
    tp->ini_maxvel = 0.0;
    tp->wMax = 0.0;
    tp->wDotMax = 0.0;
    tp->currentVel=0.0;
    return tpClear(tp);
}

int tpSetCycleTime(TP_STRUCT * tp, double secs)
{
    if (0 == tp || secs <= 0.0) {
	return -1;
    }

    tp->cycleTime = secs;

    return 0;
}
// This is called before adding lines or circles, specifying
// vMax (the velocity requested by the F word) and
// ini_maxvel, the max velocity possible before meeting
// a machine constraint caused by an AXIS's max velocity.
// (the TP is allowed to go up to this high when feed 
// override >100% is requested)  These settings apply to
// subsequent moves until changed.
int tpSetJVScale(TP_STRUCT * tp, double jvscale){
 if (0 == tp ) {
	return -1;
    }
  tp->jvscale= (fabs( jvscale)>DOUBLE_FUZZ) ? jvscale:0.0;
    return 0;

};

int tpSetVR(TP_STRUCT * tp, double vr)
{
    if (0 == tp || vr< 0.0 ) {
	return -1;
    }
  tp->wMax= (fabs(vr)>DOUBLE_FUZZ) ? vr:0.0;

    return 0;
}
int	tpSetRLimit(TP_STRUCT * tp, double vr){
  if (0 == tp || vr< 0.0 ) {
	return -1;
    }
  tp->wLimit= (fabs(vr)>DOUBLE_FUZZ) ? vr:0.0;

    return 0;

}
int tpSetRamax(TP_STRUCT * tp, double vr){
  if (0 == tp || vr< 0.0 ) {
	return -1;
    }
  tp->wDotMax= (fabs(vr)>DOUBLE_FUZZ) ? vr:0.0;

    return 0;

}
int tpSetPL(TP_STRUCT * tp, int pl)
{
    if (0 == tp || pl<0 ) {
	return -1;
    }

    tp->PL= pl;
    return 0;
}

int tpSetCR(TP_STRUCT * tp, double cr)
{
    if (0 == tp || cr< 0.0 ) {
	return -1;
    }
  tp->CR= (fabs(cr)>DOUBLE_FUZZ) ? cr:0.0;

    return 0;
}
int tpSetIOType(TP_STRUCT * tp, int IOtype)
{
    if (0 == tp ) {
	return -1;
    }

    tp->iotype= IOtype;
    return 0;
}
int tpSetAddrType(TP_STRUCT * tp, int addrtype)
{
    if (0 == tp ) {
	return -1;
    }

    tp->addrtype= addrtype;
    return 0;
}
int tpSetStopCondition(TP_STRUCT * tp, int stopconditon)
{
    if (0 == tp ||(0 != stopconditon && 1!= stopconditon) ) {
	return -1;
    }

    tp->stopcondition= stopconditon;
    return 0;
}

int tpSetIOaddress(TP_STRUCT * tp, int ioaddress)
{
      if (0 == tp || ioaddress< 0 ) {
	return -1;
    }

    tp->ioaddress= ioaddress;
    return 0;
}
int tpSetDO(TP_STRUCT * tp, int addrtype,int ioaddr,int state )
{

    TC_STRUCT tc;

    if (!tp) {
        diagnostics("TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        diagnostics( "TP is aborting\n");
	return -1;
    }
	tcInit(&tc);
	tc.dotype=1;
	tc.addrtype= addrtype;
	tc.ioaddress= ioaddr;
	tc.iostate= state;

    if (tcqPut(&tp->queue, tc) == -1) {
        diagnostics("tcqPut failed.\n");
	return -1;
    }

    return 0;
}

int tpSetaccScale(TP_STRUCT * tp, double accscale)
{
    if (0 == tp || accscale< 0.0 ) {
	return -1;
    }
 tp->accScale= (fabs(accscale)>DOUBLE_FUZZ) ? accscale:0.0;
    return 0;
}

int tpSetDispon(TP_STRUCT * tp,int dispon_flag)
{
    if (0 == tp || ((dispon_flag != 0)&&(dispon_flag != 1)&&(dispon_flag != 2)) ) {
	return -1;
    }

    tp->dispon_flag = dispon_flag;
//diagnostics(" tp->dispon_flag %d \n", tp->dispon_flag);
    return 0;
}

int tpSetVector(TP_STRUCT * tp,RobotPose  disp_vector) 
{
	if (0 == tp ) {
		return -1;
	}
	tp->disp_vector.tran.x =   (fabs(disp_vector.tran.x)>DOUBLE_FUZZ) ? disp_vector.tran.x:0.0;
	tp->disp_vector.tran.y =   (fabs(disp_vector.tran.y)>DOUBLE_FUZZ) ? disp_vector.tran.y:0.0;
	tp->disp_vector.tran.z =   (fabs(disp_vector.tran.z)>DOUBLE_FUZZ) ? disp_vector.tran.z:0.0;
	tp->disp_vector.a =   (fabs(disp_vector.a)>DOUBLE_FUZZ) ? disp_vector.a:0.0;
	tp->disp_vector.b =   (fabs(disp_vector.b)>DOUBLE_FUZZ) ? disp_vector.b:0.0;
	tp->disp_vector.c =   (fabs(disp_vector.c)>DOUBLE_FUZZ) ? disp_vector.c:0.0;

	return 0;
}
int tpSetJVector(TP_STRUCT * tp, PmJoint disp_joint) 
{
	if (0 == tp ) {
		return -1;
	}
	tp->disp_joint.j0 =   (fabs(disp_joint.j0)>DOUBLE_FUZZ) ? disp_joint.j0:0.0;
	tp->disp_joint.j1 =   (fabs(disp_joint.j1)>DOUBLE_FUZZ) ? disp_joint.j1:0.0;
	tp->disp_joint.j2 =   (fabs(disp_joint.j2)>DOUBLE_FUZZ) ? disp_joint.j2:0.0;
	tp->disp_joint.j3 =   (fabs(disp_joint.j3)>DOUBLE_FUZZ) ? disp_joint.j3:0.0;
	tp->disp_joint.j4 =   (fabs(disp_joint.j4)>DOUBLE_FUZZ) ? disp_joint.j4:0.0;
	tp->disp_joint.j5 =   (fabs(disp_joint.j5)>DOUBLE_FUZZ) ? disp_joint.j5:0.0;
	//diagnostics(" tp->disp_joint %d %d %d %d %d %d \n", (int)(tp->disp_joint.j0),(int)(tp->disp_joint.j1),(int)(tp->disp_joint.j2),(int)(tp->disp_joint.j3),(int)(tp->disp_joint.j4),(int)(tp->disp_joint.j5));

	return 0;
}


int tpSetdecScale(TP_STRUCT * tp, double decscale)
{
    if (0 == tp || decscale< 0.0 ) {
	return -1;
    }

    tp->decScale= (fabs( decscale)>DOUBLE_FUZZ) ? decscale:0.0;
    return 0;
}
int tpSetcordType(TP_STRUCT * tp, int type)
{
    if (0 == tp || type< 0 ||type> 2) {
	return -1;
    }

    tp->cord_type= type;
    return 0;
}

 int tpSetTCP(TP_STRUCT * tp, PmCartesian position)
 {
    if (0 == tp) {
	return -1;
    }
	    tp->robot_tcp= position;
    return 0;


 }
  int tpSetTCF(TP_STRUCT * tp, PmRotationMatrix position)
 {
    if (0 == tp ) {
	return -1;
    }
	    tp->robot_tcf= position;
    return 0;


 }
   int tpSetInvTCF(TP_STRUCT * tp, PmRotationMatrix position)
 {
    if (0 == tp  ) {
	return -1;
    }
	    tp->robot_invtcf = position;
    return 0;


 }
int tpSetJVmax(TP_STRUCT * tp, PmJoint pos)
{
    if (0 == tp ) {
	return -1;
    }

    tp->jvmax.j0= (fabs(pos.j0)>DOUBLE_FUZZ) ? pos.j0:0.0;
    tp->jvmax.j1= (fabs(pos.j1)>DOUBLE_FUZZ) ? pos.j1:0.0;
    tp->jvmax.j2= (fabs(pos.j2)>DOUBLE_FUZZ) ? pos.j2:0.0;
    tp->jvmax.j3= (fabs(pos.j3)>DOUBLE_FUZZ) ? pos.j3:0.0;
    tp->jvmax.j4= (fabs(pos.j4)>DOUBLE_FUZZ) ? pos.j4:0.0;
    tp->jvmax.j5= (fabs(pos.j5)>DOUBLE_FUZZ) ? pos.j5:0.0;
    tp->jvmax.j6= (fabs(pos.j6)>DOUBLE_FUZZ) ? pos.j6:0.0;
    tp->jvmax.j7= (fabs(pos.j7)>DOUBLE_FUZZ) ? pos.j7:0.0;

    return 0;
}
int tpSetJVlimit(TP_STRUCT * tp, PmJoint vLimit)
{
    if (0 == tp ) {
	return -1;
    }
    tp->jvLimit.j0= (fabs(vLimit.j0)>DOUBLE_FUZZ) ? vLimit.j0:0.0;
    tp->jvLimit.j1= (fabs(vLimit.j1)>DOUBLE_FUZZ) ? vLimit.j1:0.0;
    tp->jvLimit.j2= (fabs(vLimit.j2)>DOUBLE_FUZZ) ? vLimit.j2:0.0;
    tp->jvLimit.j3= (fabs(vLimit.j3)>DOUBLE_FUZZ) ? vLimit.j3:0.0;
    tp->jvLimit.j4= (fabs(vLimit.j4)>DOUBLE_FUZZ) ? vLimit.j4:0.0;
    tp->jvLimit.j5= (fabs(vLimit.j5)>DOUBLE_FUZZ) ? vLimit.j5:0.0;
    tp->jvLimit.j6= (fabs(vLimit.j6)>DOUBLE_FUZZ) ? vLimit.j6:0.0;
    tp->jvLimit.j7= (fabs(vLimit.j6)>DOUBLE_FUZZ) ? vLimit.j7:0.0;

    return 0;
}

int tpSetJaccmax(TP_STRUCT * tp, PmJoint pos)
{
    if (0 == tp ) {
	return -1;
    }
    tp->jaccmax.j0= (fabs(pos.j0)>DOUBLE_FUZZ) ? pos.j0:0.0;
    tp->jaccmax.j1= (fabs(pos.j1)>DOUBLE_FUZZ) ? pos.j1:0.0;
    tp->jaccmax.j2= (fabs(pos.j2)>DOUBLE_FUZZ) ? pos.j2:0.0;
    tp->jaccmax.j3= (fabs(pos.j3)>DOUBLE_FUZZ) ? pos.j3:0.0;
    tp->jaccmax.j4= (fabs(pos.j4)>DOUBLE_FUZZ) ? pos.j4:0.0;
    tp->jaccmax.j5= (fabs(pos.j5)>DOUBLE_FUZZ) ? pos.j5:0.0;
    tp->jaccmax.j6= (fabs(pos.j6)>DOUBLE_FUZZ) ? pos.j6:0.0;
    tp->jaccmax.j7= (fabs(pos.j7)>DOUBLE_FUZZ) ? pos.j7:0.0;

    return 0;
}
int tpSetJdecmax(TP_STRUCT * tp, PmJoint pos)
{
    if (0 == tp ) {
	return -1;
    }
    tp->jdecmax.j0= (fabs(pos.j0)>DOUBLE_FUZZ) ? pos.j0:0.0;
    tp->jdecmax.j1= (fabs(pos.j1)>DOUBLE_FUZZ) ? pos.j1:0.0;
    tp->jdecmax.j2= (fabs(pos.j2)>DOUBLE_FUZZ) ? pos.j2:0.0;
    tp->jdecmax.j3= (fabs(pos.j3)>DOUBLE_FUZZ) ? pos.j3:0.0;
    tp->jdecmax.j4= (fabs(pos.j4)>DOUBLE_FUZZ) ? pos.j4:0.0;
    tp->jdecmax.j5= (fabs(pos.j5)>DOUBLE_FUZZ) ? pos.j5:0.0;
    tp->jdecmax.j6= (fabs(pos.j6)>DOUBLE_FUZZ) ? pos.j6:0.0;
    tp->jdecmax.j7= (fabs(pos.j7)>DOUBLE_FUZZ) ? pos.j7:0.0;

    return 0;
}
int tpSetVmax(TP_STRUCT * tp, double vMax)
{
    if (0 == tp || vMax < 0.0 ) {
	return -1;
    }
    tp->vMax= (fabs(vMax)>DOUBLE_FUZZ) ? vMax:0.0;

    return 0;
}

//get the velocity requested by the F word
double tpGetFeedVel(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }
   
    return tp->vMax;
}

// I think this is the [TRAJ] max velocity.  This should
// be the max velocity of the TOOL TIP, not necessarily
// any particular axis.  This applies to subsequent moves
// until changed.
/*! \todo FIXME - corresponding to the command EMCMOT_SET_VEL_LIMIT 
*/// 2010.05.18
int tpSetVlimit(TP_STRUCT * tp, double vLimit)
{
    if (0 == tp || vLimit < 0.0) {
	return -1;
    }
    tp->vLimit= (fabs( vLimit)>DOUBLE_FUZZ) ? vLimit:0.0;

    return 0;
}

// Set max accel

int tpSetAmax(TP_STRUCT * tp, double aMax)
{
    if (0 == tp || aMax < 0.0) {
	return -1;
    }
    tp->aMax= (fabs( aMax)>DOUBLE_FUZZ) ? aMax:0.0;

    return 0;
}


/*
  tpSetId() sets the id that will be used for the next appended motions.
  nextId is incremented so that the next time a motion is appended its id
  will be one more than the previous one, modulo a signed int. If
  you want your own ids for each motion, call this before each motion
  you append and stick what you want in here.
  */
int tpSetId(TP_STRUCT * tp, int id)
{
    if (0 == tp) {
	return -1;
    }

    tp->nextId = id;
    return 0;
}

int tpSetFileName(TP_STRUCT * tp, unsigned int  fileName)
{  int i;
    if (0 == tp) {
	return -1;
    }
tp->fileName[0]=fileName;
	/*
for (i=0;i<MAX_FILENAME_LENGTH;i++)
    tp->fileName[i]= fileName[i];
*/
    return 0;
}

/*
  tpGetExecId() returns the id of the last motion that is currently
  executing.
  */
int tpGetExecId(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }
    return tp->execId;
}

int tpGetStopCondition(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }

    return tp->stopcondition;
}

int tpGetIOaddress(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }

    return tp->ioaddress;
}
int tpGetIOType(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }

    return tp->iotype;
}
int tpGetAddrtype(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }
    return tp->addrtype;
}


int tpSetTermCond(TP_STRUCT * tp, int cond, double tolerance)
{
    if (0 == tp) {
	return -1;
    }

    if (cond != TC_TERM_COND_STOP && cond != TC_TERM_COND_BLEND) {
	return -1;
    }

    tp->termCond = cond;  // 2011.06.15
//    tp->termCond = cond;
    tp->tolerance = tolerance;

    return 0;
}

// Used to tell the tp the initial position.  It sets
// the current position AND the goal position to be the same.  
// Used only at TP initialization and when switching modes.

int tpSetPos(TP_STRUCT * tp, RobotPose pos)
{
    if (0 == tp) {
	return -1;
    }

    tp->currentPos = pos;
    tp->goalPos = pos;

    return 0;
}
int tpSetJPos(TP_STRUCT * tp, PmJoint pos)
{
	double positions[MAX_JOINTS];
	RobotPose world;

    if (0 == tp) {
	return -1;
    }

	tp->currentJPos = pos;
	tp->goalJPos = pos;
	positions[0]=pos.j0;
	positions[1]=pos.j1;
	positions[2]=pos.j2;
	positions[3]=pos.j3;
	positions[4]=pos.j4;
	positions[5]=pos.j5;
	kinematicsForward(positions, &world, &Rtmp,&fflags,&iflags);	
	world.a=0.0;
	world.b=0.0;
	world.c=0.0;
	tp->currentPos = world;
	tp->goalPos = world;
	tp->goalFrame.rot= Rtmp ;
	tp->currentFrame.rot= Rtmp ;
#if 0
if(((emcmotDebug->queue.free==1)&&(4 == emcmotStatus->coordinate_type))||((emcmotDebug->queue.free!=1)&&(emcmotDebug->queue.cord_type ==2)))
{
	pmMatMatMult(Rtmp, emcmotDebug->robot_tcf,  &RS);//工具坐标系方向
	emcmotDebug->robot_RT =RS;
	tp->robot_RT =RS;
	pmMatCartMult(Rtmp,emcmotDebug->robot_tcp,&Pm);
	pmCartCartAdd(Pm,emcmotDebug->robot_P0,&tmp1);//原点在基坐标系坐标
	tp->currentPos.tran = tmp1;
	tp->goalPos.tran = tmp1;
}
	emcmotDebug->robot_R0=Rtmp;
	tp->robot_R0 = Rtmp;
#endif

    return 0;
}


 int tpAdjustJoints(TC_STRUCT * tc){
	PmJLine line_joints;
	PmJoint  start_joint, end_joint;
	int joint_num;
	double t0,t1,t2,t3,t4,t5;
	double d0,d1,d2,d3,d4,d5;
	double tmp;

	start_joint=emcmotStatus->joint_cmd;
	end_joint =tc->coords.jline.end;

	d0 = fabs(end_joint.j0-start_joint.j0);
	d1 = fabs(end_joint.j1-start_joint.j1);
	d2 = fabs(end_joint.j2-start_joint.j2);
	d3 = fabs(end_joint.j3-start_joint.j3);
	d4 = fabs(end_joint.j4-start_joint.j4);
	d5 = fabs(end_joint.j5-start_joint.j5);

	if(d0 < TP_VEL_EPSILON) d0 = 0.0;
	if(d1 < TP_VEL_EPSILON) d1 = 0.0;
	if(d2 < TP_VEL_EPSILON) d2 = 0.0;
	if(d3 < TP_VEL_EPSILON) d3 = 0.0;
	if(d4 < TP_VEL_EPSILON) d4 = 0.0;
	if(d5 < TP_VEL_EPSILON) d5 = 0.0;
	
	t0 = d0? (d0 /emcmotConfig->jointMoveVel[0]): 0.0;
	t1 = d1? (d1 /emcmotConfig->jointMoveVel[1]): 0.0;
	t2 = d2? (d2 /emcmotConfig->jointMoveVel[2]): 0.0;
	t3 = d3? (d3 /emcmotConfig->jointMoveVel[3]): 0.0;
	t4 = d4? (d4 /emcmotConfig->jointMoveVel[4]): 0.0;
	t5 = d5? (d5 /emcmotConfig->jointMoveVel[5]): 0.0;

	joint_num=0;
	tmp=t0;
	tc->reqvel = emcmotConfig->jointMoveVel[0]*tc->jvscale;
	tc->maxvel=emcmotConfig->jointMoveMaxVel[0];
	tc->maxaccel = emcmotConfig->jointMoveAcc[0]*tc->accScale;

      if(tmp<t1){
	joint_num=1;
	tmp=t1;
	tc->reqvel = emcmotConfig->jointMoveVel[1]*tc->jvscale;
	tc->maxvel=emcmotConfig->jointMoveMaxVel[1];
	tc->maxaccel = emcmotConfig->jointMoveAcc[1]*tc->accScale;
	}

	if(tmp<t2){
	joint_num=2;
	tmp=t2;
	tc->reqvel = emcmotConfig->jointMoveVel[2]*tc->jvscale;
	tc->maxvel=emcmotConfig->jointMoveMaxVel[2];
	tc->maxaccel = emcmotConfig->jointMoveAcc[2]*tc->accScale;
	}	 	

	if(tmp<t3){
	joint_num=3;
	tmp=t3;
	tc->reqvel = emcmotConfig->jointMoveVel[3]*tc->jvscale;
	tc->maxvel=emcmotConfig->jointMoveMaxVel[3];
	tc->maxaccel = emcmotConfig->jointMoveAcc[3]*tc->accScale;
	}	 	

	if(tmp<t4){
	joint_num=4;
	tmp=t4;
	tc->reqvel = emcmotConfig->jointMoveVel[4]*tc->jvscale;
	tc->maxvel=emcmotConfig->jointMoveMaxVel[4];
	tc->maxaccel = emcmotConfig->jointMoveAcc[4]*tc->accScale;
	}	 	

	if(tmp<t5){
	joint_num=5;
	tmp=t5;
	tc->reqvel = emcmotConfig->jointMoveVel[5]*tc->jvscale;
	tc->maxvel=emcmotConfig->jointMoveMaxVel[5];
	tc->maxaccel = emcmotConfig->jointMoveAcc[5]*tc->accScale;
	}	 	
	
      pmJointsInit(&line_joints, start_joint, end_joint,joint_num);
	tc->target= line_joints.jmag;
       tc->coords.jline = line_joints;

    return 0;

}
 int tpAdjustLine(TC_STRUCT * tc){
    PmLine line_xyz, line_uvw, line_abc;
    PmPose start_xyz, end_xyz;
    PmPose start_uvw, end_uvw;
    PmPose start_abc, end_abc;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
    double joint_pos[8];
    int joint_num,flag;
    RobotPose world;
    MotionJointParameter *joint;

if(tc->canon_motion_type==1){
       start_xyz.tran = emcmotStatus->carte_pos_cmd.tran;
	start_abc.tran.x = emcmotStatus->carte_pos_cmd.a;
	start_abc.tran.y = emcmotStatus->carte_pos_cmd.b;
	start_abc.tran.z = emcmotStatus->carte_pos_cmd.c;
	end_xyz.tran=tc->coords.line.xyz.end.tran;
	end_abc.tran=tc->coords.line.abc.end.tran;
      
}else{
	start_xyz.tran = emcmotStatus->carte_pos_cmd.tran;
	start_abc.tran.x = emcmotStatus->carte_pos_cmd.a;
	start_abc.tran.y = emcmotStatus->carte_pos_cmd.b;
	start_abc.tran.z = emcmotStatus->carte_pos_cmd.c;

	joint_pos[0] =emcmotStatus->joint_cmd.j0+tc->deltjoints.j0;
	joint_pos[1] =emcmotStatus->joint_cmd.j1+tc->deltjoints.j1;
	joint_pos[2] =emcmotStatus->joint_cmd.j2+tc->deltjoints.j2;
	joint_pos[3] =emcmotStatus->joint_cmd.j3+tc->deltjoints.j3;
	joint_pos[4] =emcmotStatus->joint_cmd.j4+tc->deltjoints.j4;
	joint_pos[5] =emcmotStatus->joint_cmd.j5+tc->deltjoints.j5;
        flag=0;
	for (joint_num = 0; joint_num < num_joints; joint_num++) {
	// point to joint data //
	joint = &joints[joint_num];
	if ((joint_pos[joint_num] > joint->max_jog_limit) ||
	    (joint_pos[joint_num] < joint->min_jog_limit)) {
               flag=1;
		break;		// can't move further past limit 
	}
    }
      if(1==flag){
	joint_pos[0] =emcmotStatus->joint_cmd.j0;
	joint_pos[1] =emcmotStatus->joint_cmd.j1;
	joint_pos[2] =emcmotStatus->joint_cmd.j2;
	joint_pos[3] =emcmotStatus->joint_cmd.j3;
	joint_pos[4] =emcmotStatus->joint_cmd.j4;
	joint_pos[5] =emcmotStatus->joint_cmd.j5;
      	}
	kinematicsForward(joint_pos,&world,&Rtmp,&fflags,&iflags);
	end_xyz.tran=world.tran;
	end_abc.tran.x=world.a;
	end_abc.tran.y=world.b;
	end_abc.tran.z=world.c;
}

    start_uvw.tran.x = 0.0;
    start_uvw.tran.y = 0.0;
    start_uvw.tran.z = 0.0;
    end_uvw.tran.x = 0.0;
    end_uvw.tran.y = 0.0;
    end_uvw.tran.z = 0.0;

    start_xyz.rot = identity_quat;
    end_xyz.rot = identity_quat;
    start_uvw.rot = identity_quat;
    end_uvw.rot = identity_quat;
    start_abc.rot = identity_quat;
    end_abc.rot = identity_quat;

//! \todo FIXEME - calculate the unit vector from start to end( as: line->uVec), and it calculate other
//   values, such as line->start, line->end, line->tmag, line->tmag_zero etc,which will be used by other
//    functions later. 
//// 2010.05.18 byf
    pmLineInit(&line_xyz, start_xyz, end_xyz);
    pmLineInit(&line_uvw, start_uvw, end_uvw);
    pmLineInit(&line_abc, start_abc, end_abc);

   if (!line_xyz.tmag_zero) 
        tc->target = line_xyz.tmag;
    else if (!line_uvw.tmag_zero)
        tc->target = line_uvw.tmag;
    else
        tc->target = line_abc.tmag;
		
    if (!line_xyz.tmag_zero) {
	tc->reqvel = tc->vMax;
	tc->maxaccel = tc->aMax;
	tc->maxvel = tc->vLimit;
	}
    else {
	tc->reqvel = tc->wMax;
	tc->maxaccel = tc->wDotMax;
	tc->maxvel = tc->wLimit;
    	}
    tc->coords.line.xyz = line_xyz;
    tc->coords.line.uvw = line_uvw;
    tc->coords.line.abc = line_abc;
    tc->motion_type = TC_LINEAR;

    return 0;


}


int tpAddJoints(TP_STRUCT * tp, PmJoint joint,int flag)
{

	TC_STRUCT tc;
	PmJLine line_joints;
	PmJoint  start_joint, end_joint;
	int joint_num;
	double t0,t1,t2,t3,t4,t5,t6,t7;
	double d0,d1,d2,d3,d4,d5,d6,d7;
	double tmp;
	double joint_pos[8];
       int i;
       RobotPose world;

    if (!tp) {
        diagnostics( "TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        diagnostics("TP is aborting\n");
	return -1;
    }

	start_joint=tp->goalJPos; //emcmotStatus->joint_cmd;
//diagnostics("2tpAddJoints %d,%d,%d,%d,%d,%d\n",(int)(start_joint.j0),(int)(start_joint.j1),(int)(start_joint.j2),(int)(start_joint.j3),(int)(start_joint.j4),(int)(start_joint.j5));
	if(2 == tp->dispon_flag) 
	{
		joint.j0 += tp->disp_joint.j0;
		joint.j1 += tp->disp_joint.j1;
		joint.j2 += tp->disp_joint.j2;
		joint.j3 += tp->disp_joint.j3;
		joint.j4 += tp->disp_joint.j4;
		joint.j5 += tp->disp_joint.j5;
	}

	joint_pos[0]=joint.j0;
	joint_pos[1]=joint.j1;
	joint_pos[2]=joint.j2;
	joint_pos[3]=joint.j3;
	joint_pos[4]=joint.j4;
	joint_pos[5]=joint.j5;
	joint_pos[6]=joint.j6;
	joint_pos[7]=joint.j7;

	end_joint = joint;

//	if(1 == tp->dispon_flag) 
//	{
//		pre_joints[0]= start_joint.j0;
//		pre_joints[1]= start_joint.j1;
//		pre_joints[2]= start_joint.j2;
//		pre_joints[3]= start_joint.j3;
//		pre_joints[4]= start_joint.j4;
//		pre_joints[5]= start_joint.j5;

//	       kinematicsForward(joint_pos,&world,&fflags,&iflags);
//		pmCartCartAdd(world.tran, tp->disp_vector.tran, &(world.tran));
//		world.a += tp->disp_vector.a;
//		world.b += tp->disp_vector.b;
//		world.c += tp->disp_vector.c;


//	      if ((fflag = kinematicsInverse(&world, positions,pre_joints,fflags)))
//	      {
  //                  reportError(3200, -1, 0);
//			diagnostics(" tpaddjoint kinematics error \n");}
//
//		end_joint.j0 = positions[0];
//		end_joint.j1 = positions[1];
//		end_joint.j2 = positions[2];
//		end_joint.j3 = positions[3];
//		end_joint.j4 = positions[4];
//		end_joint.j5 = positions[5];

 //            joint = end_joint;

//	}

// diagnostics("tpAddJoints start %d,%d,%d,%d,%d,%d\n",(int)(start_joint.j0),(int)(start_joint.j1),(int)(start_joint.j2),(int)(start_joint.j3),(int)(start_joint.j4),(int)(start_joint.j5));
// diagnostics("tpAddJoints end %d,%d,%d,%d,%d,%d\n",(int)(end_joint.j0),(int)(end_joint.j1),(int)(end_joint.j2),(int)(end_joint.j3),(int)(end_joint.j4),(int)(end_joint.j5));

	d0 = fabs(end_joint.j0 - start_joint.j0);
	d1 = fabs(end_joint.j1 - start_joint.j1);
	d2 = fabs(end_joint.j2 - start_joint.j2);
	d3 = fabs(end_joint.j3 - start_joint.j3);
	d4 = fabs(end_joint.j4 - start_joint.j4);
	d5 = fabs(end_joint.j5 - start_joint.j5);
	d6 = fabs(end_joint.j6 - start_joint.j6);
	d7 = fabs(end_joint.j7 - start_joint.j7);

	if(d0 < TP_VEL_EPSILON) d0 = 0.0;
	if(d1 < TP_VEL_EPSILON) d1 = 0.0;
	if(d2 < TP_VEL_EPSILON) d2 = 0.0;
	if(d3 < TP_VEL_EPSILON) d3 = 0.0;
	if(d4 < TP_VEL_EPSILON) d4 = 0.0;
	if(d5 < TP_VEL_EPSILON) d5 = 0.0;
	if(d6 < TP_VEL_EPSILON) d6 = 0.0;
	if(d7 < TP_VEL_EPSILON) d7 = 0.0;


	t0 = (d0 > 0.0) ? (d0 / tp->jvmax.j0): 0.0;
	t1 = (d1 > 0.0) ? (d1 / tp->jvmax.j1): 0.0;
	t2 = (d2 > 0.0) ? (d2 / tp->jvmax.j2): 0.0;
	t3 = (d3 > 0.0) ? (d3 / tp->jvmax.j3): 0.0;
	t4 = (d4 > 0.0) ? (d4 / tp->jvmax.j4): 0.0;
	t5 = (d5 > 0.0) ? (d5 / tp->jvmax.j5): 0.0;
	t6 = (d6 > 0.0) ? (d6 / tp->jvmax.j6): 0.0;
	t7 = (d7 > 0.0) ? (d7 / tp->jvmax.j7): 0.0;

	tcInit(&tc);
	tc.jvscale=tp->jvscale;
	tc.accScale=tp->accScale;
	  
	joint_num=0;
	tmp=t0;
	tc.reqvel = tp->jvmax.j0*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j0;
	tc.maxaccel = tp->jaccmax.j0*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j0*tp->decScale;

	if(tmp<t1){
	joint_num=1;
	tmp=t1;
	tc.reqvel = tp->jvmax.j1*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j1;
	tc.maxaccel = tp->jaccmax.j1*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j1*tp->decScale;

	}
	if(tmp<t2){
	joint_num=2;
	tmp=t2;
	tc.reqvel = tp->jvmax.j2*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j2;
	tc.maxaccel = tp->jaccmax.j2*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j2*tp->decScale;

	}	 	

	if(tmp<t3){
	joint_num=3;
	tmp=t3;
	tc.reqvel = tp->jvmax.j3*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j3;
	tc.maxaccel = tp->jaccmax.j3*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j3*tp->decScale;

	}	 	
	if(tmp<t4){
	joint_num=4;
	tmp=t4;
	tc.reqvel = tp->jvmax.j4*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j4;
	tc.maxaccel = tp->jaccmax.j4*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j4*tp->decScale;

	}	 	

	if(tmp<t5){
	joint_num=5;
	tmp=t5;
	tc.reqvel = tp->jvmax.j5*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j5;
	tc.maxaccel = tp->jaccmax.j5*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j5*tp->decScale;

	}
	if(tmp<t6){
	joint_num=6;
	tmp=t6;
	tc.reqvel = tp->jvmax.j6*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j6;
	tc.maxaccel = tp->jaccmax.j6*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j5*tp->decScale;
	}	
	if(tmp<t7){
	joint_num=7;
	tmp=t7;
	tc.reqvel = tp->jvmax.j7*tc.jvscale;
	tc.maxvel=tp->vLimit=tp->jvLimit.j7;
	tc.maxaccel = tp->jaccmax.j7*tp->accScale;
//	tc.maxdecel = tp->jdecmax.j5*tp->decScale;

	}	 	
      pmJointsInit(&line_joints, start_joint, end_joint,joint_num);
   
	tc.cycle_time = tp->cycleTime;
	tc.target= line_joints.jmag;
	tc.progress = 0.0;
	tc.feed_override = 1.0;
	tc.free=0;
	tc.joint_type=1;
	tc.PL=tp->PL;
	tc.stopcondition=tp->stopcondition;
	tc.ioaddress=tp->ioaddress;
	tc.iotype=tp->iotype;
	tc.addrtype=tp->addrtype;
	tc.is_extaxis=1==flag;

	tc.id = tp->nextId;
       for (i=0;i<MAX_FILENAME_LENGTH;i++)
        tc.fileName[i]= tp->fileName[i];

	tc.active = 0;
       tc.coords.jline = line_joints;
	tc.motion_type = TC_JLINEAR;
	tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
	tc.tolerance = tp->tolerance;

	tc.synchronized = tp->synchronized;

    if (tcqPut(&tp->queue, tc) == -1) {
      //  diagnostics( "tcqPut failed.\n");
	return -1;
    }


	tp->goalJPos= joint;      // remember the end of this move, as it's
	kinematicsForward(joint_pos,&world,&Rtmp,&fflags,&iflags);
	tp->goalPos.tran=world.tran;
//	tp->goalPos.a=world.b;
//	tp->goalPos.b=world.b;
//	tp->goalPos.c=world.c;
	tp->goalFrame.rot= Rtmp;
// diagnostics("1tpAddJoints cati %d,%d,%d,%d,%d,%d\n",(int)(tp->goalPos.tran.x),(int)(tp->goalPos.tran.y),(int)(tp->goalPos.tran.z),(int)(world.a),(int)(world.b),(int)(world.c));

    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;

    return 0;
}
int tpAddFreeJoints(TP_STRUCT * tp, double joint,int type){
    TC_STRUCT tc;
    PmJLine line_joints;
    PmJoint  start_joint = {0,0,0,0,0,0}, end_joint = {0,0,0,0,0,0};

    if (!tp) {
        diagnostics( "TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        diagnostics("TP is aborting\n");
	return -1;
    }
    tcInit(&tc);

      tc.feed_override = emcmotDebug->net_feed_scale[type];
	printf("tc.feed_override =%f \r\n ",tc.feed_override);
      tc.joint_num= type;

switch(type){
	case 0:
	start_joint = emcmotStatus->joint_cmd;
       end_joint=start_joint;
	end_joint.j0=joint;	
//	end_joint.j0=joint;	

       tc.reqvel = tp->jvmax.j0;
	tc.maxvel=tp->vLimit=tp->jvLimit.j0;
	tc.maxaccel = tp->jaccmax.j0;
//	tc.maxdecel = tp->jdecmax.j0;   
      
       break;
	case 1:
	start_joint=emcmotStatus->joint_cmd;
       end_joint=start_joint;
	end_joint.j1=joint;	
//	end_joint.j1=joint;	

	tc.reqvel = tp->jvmax.j1;
	tc.maxvel=tp->vLimit=tp->jvLimit.j1;
	tc.maxaccel = tp->jaccmax.j1;
//	tc.maxdecel = tp->jdecmax.j1;   

        break;
	case 2:
	start_joint=emcmotStatus->joint_cmd;
       end_joint=start_joint;
	end_joint.j2=joint;	
//	end_joint.j2=joint;	

       tc.reqvel = tp->jvmax.j2;
	tc.maxvel=tp->vLimit=tp->jvLimit.j2;
	tc.maxaccel = tp->jaccmax.j2;
//	tc.maxdecel = tp->jdecmax.j2;   

	break;
	case 3:
	start_joint=emcmotStatus->joint_cmd;
       end_joint=start_joint;
	end_joint.j3=joint;	
//	end_joint.j3=joint;	

	tc.reqvel = tp->jvmax.j3;
	tc.maxvel=tp->vLimit=tp->jvLimit.j3;
	tc.maxaccel = tp->jaccmax.j3;
//	tc.maxdecel = tp->jdecmax.j3;   

	break;
	case 4:
       start_joint=emcmotStatus->joint_cmd;
       end_joint=start_joint;
	end_joint.j4=joint;
//	end_joint.j4=joint;

       tc.reqvel = tp->jvmax.j4;
	tc.maxvel=tp->vLimit=tp->jvLimit.j4;
	tc.maxaccel = tp->jaccmax.j4;
//	tc.maxdecel = tp->jdecmax.j4;   

	break;
	case 5:
	start_joint=emcmotStatus->joint_cmd;
	end_joint=start_joint;
	end_joint.j5=joint;	
//	end_joint.j5=joint;	

       tc.reqvel = tp->jvmax.j5;
	tc.maxvel=tp->vLimit=tp->jvLimit.j5;
	tc.maxaccel = tp->jaccmax.j5;
//	tc.maxdecel = tp->jdecmax.j5;   
	break;

	case 6:
	start_joint =emcmotStatus->joint_cmd;
	end_joint=start_joint;
	end_joint.j6=joint;	
//	end_joint.j5=joint;	
       tc.reqvel = tp->jvmax.j6;
	tc.maxvel=tp->vLimit=tp->jvLimit.j6;
	tc.maxaccel = tp->jaccmax.j6;
//	tc.maxdecel = tp->jdecmax.j5;   
	break;

	case 7:
	start_joint =emcmotStatus->joint_cmd;
	end_joint=start_joint;
	end_joint.j7=joint;	
//	end_joint.j5=joint;	
       tc.reqvel = tp->jvmax.j7;
	tc.maxvel=tp->vLimit=tp->jvLimit.j7;
	tc.maxaccel = tp->jaccmax.j7;

	break;

	default:
	break;
}
//diagnostics("tpAddFreeJoints,joint %d, tc.maxaccel= %d \n",type,(int)(1000*tc.maxaccel));
//      diagnostics("000000000%d,%d,%d,%d,%d,%d\n",(int)(start_joint.j0),(int)(start_joint.j1),(int)(start_joint.j2),(int)(start_joint.j3),(int)(start_joint.j4),(int)(start_joint.j5));
//     diagnostics("3333333%d,%d,%d,%d,%d,%d\n",(int)(end_joint.j0),(int)(end_joint.j1),(int)(end_joint.j2),(int)(end_joint.j3),(int)(end_joint.j4),(int)(end_joint.j5));

       pmJointsInit(&line_joints, start_joint, end_joint,type);
	tc.cycle_time = tp->cycleTime;
	tc.accScale=tp->accScale;
	tc.target= line_joints.jmag;
	//   diagnostics("tpAddFreeJoints,%d\n",(int)(tc.target));

	tc.progress = 0.0;
//	tc.feed_override = 0.0;
	tc.free=1;
	tc.joint_type=1;
	tc.active = 0;
	tc.coords.jline= line_joints;
	tc.motion_type = TC_JLINEAR;
	tc.blend_with_next = 0;            

    if (tcqPut(&tp->queue, tc) == -1) {
        diagnostics( "tcqPut failed.\n");
	 return -1;
    }

    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;

    return 0;
}

double newEnd(double start, double end)
{
	double tmpInc;

	tmpInc = end - start;

	tmpInc = tmpInc - (int)tmpInc / 360 * 360.0;
	if (tmpInc < 0.0) {
		tmpInc += 360.0;
	}

	if (tmpInc > 180.0)
		tmpInc -= 360.0;
	if (tmpInc <= -180.0)
		tmpInc += 360.0;

	return (start + tmpInc);
}
int tpAddUsrLine(TP_STRUCT * tp, PointPose end, UserCoordianteInformation usr,ToolCoordianteInformation tool)
{
	TC_STRUCT tc;
	PmLine line_xyz, line_uvw, line_abc;
	PmPose start_xyz, end_xyz;
	PmPose start_uvw, end_uvw;
	PmPose start_abc, end_abc;
	PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
	double joint_pos[6];
	PmEulerZyx tempZyx;
	RobotPose world;
	PmHomogeneous start,endHomo;
	JointPoint startJoint,endJoint;
	PmRotationMatrix Rtmp1,Rtmp2,Rtmp3,Rtmp4,Rm1,Rm2;
	PmCartesian Pm,tmp1,tmp2;

	tc.vMax = tp->vMax;
	tc.aMax = tp->aMax;
	tc.vLimit = tp->vLimit;
	tc.wMax = tp->wMax;
	tc.wDotMax = tp->wDotMax;
	tc.wLimit = tp->wLimit;
	tc.canon_motion_type = 2;


	//计算起点
	startJoint.j1=tp->goalJPos.j0;
	startJoint.j2=tp->goalJPos.j1;
	startJoint.j3=tp->goalJPos.j2;
	startJoint.j4=tp->goalJPos.j3;
	startJoint.j5=tp->goalJPos.j4;
	startJoint.j6=tp->goalJPos.j5;


	CTRL_GetTCPInUserSpaceInMatrix(&start,&startJoint,&usr,&tool);
	start_xyz.tran=start.tran;
	Rtmp1=start.rot;
	tc.robot_R0= Rtmp1;
	
	tempZyx.x=end.a;
	tempZyx.y=end.b;
	tempZyx.z=end.c;




	//计算终点	
	pmZyxMatConvert(tempZyx, &Rtmp2);
	end_xyz.tran.x=end.x;
	end_xyz.tran.y=end.y;
	end_xyz.tran.z=end.z;

	//计算旋转
	pmMatInv(Rtmp1, &Rm1);
	pmMatMatMult(Rm1, Rtmp2,&Rm2);
	pmMatRotConvert(Rm2, &Vector);
	Pm.x=Vector.x;
	Pm.y=Vector.y;
	Pm.z=Vector.z;
	pmMatCartMult(Rtmp1, Pm, &tmp1);
	Vector.x=tmp1.x;
	Vector.y=tmp1.y;
	Vector.z=tmp1.z;

	start_abc.tran.x =0.0;
	start_abc.tran.y =0.0;
	start_abc.tran.z =0.0;

	end_abc.tran.x=Vector.s*180/PM_PI;
	end_abc.tran.y=0.0;
	end_abc.tran.z=0.0;


	start_uvw.tran.x = tp->goalPos.u;
	start_uvw.tran.y = tp->goalPos.v;
	start_uvw.tran.z = tp->goalPos.w;
	end_uvw.tran.x = tp->goalPos.u;
	end_uvw.tran.y = tp->goalPos.v;
	end_uvw.tran.z = tp->goalPos.w;

	start_xyz.rot = identity_quat;
	end_xyz.rot = identity_quat;
	start_uvw.rot = identity_quat;
	end_uvw.rot = identity_quat;
	start_abc.rot = identity_quat;
	end_abc.rot = identity_quat;

	pmLineInit(&line_xyz, start_xyz, end_xyz);
	pmLineInit(&line_uvw, start_uvw, end_uvw);
	pmLineInit(&line_abc, start_abc, end_abc);
	tcInit(&tc);
	tc.Vector= Vector;
	tc.cycle_time = tp->cycleTime;
	if (!line_xyz.tmag_zero)
	{ 
		if(line_abc.tmag*tc.vMax > line_xyz.tmag*tc.wMax)
		{
			tc.reqvel =  tp->wMax;
			tc.maxaccel = tp->wDotMax;
			tc.maxvel= tp->wLimit;
        		tc.target = line_abc.tmag;
		}
		else
		{
			tc.target = line_xyz.tmag;
			tc.reqvel = tp->vMax;
			tc.maxaccel = tp->aMax;
			tc.maxvel = tp->vLimit;
		}
   	}else if (!line_uvw.tmag_zero)
   	{
		tc.target = line_uvw.tmag;
   	}else
   	{
		tc.reqvel =  tp->wMax;
		tc.maxaccel = tp->wDotMax;
		tc.maxvel= tp->wLimit;
		tc.target = line_abc.tmag;
	}
	tc.progress = 0.0;
	tc.accScale=tp->accScale;
	tc.PL=tp->PL;
	tc.stopcondition=tp->stopcondition;
	tc.ioaddress=tp->ioaddress;
	tc.iotype=tp->iotype;
	tc.addrtype=tp->addrtype;
	tc.id = tp->nextId;
	tc.active = 0;
	tc.feed_override = 1.0;
	tc.free=0;
	tc.joint_type=2;
	tc.coords.line.xyz = line_xyz;
	tc.coords.line.uvw = line_uvw;
	tc.coords.line.abc = line_abc;
	tc.motion_type = TC_LINEAR;
	tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
	tc.tolerance = tp->tolerance;
	tc.synchronized = tp->synchronized;
 //   tc.enables = enables;
 	tc.goalFrame.rot=Rtmp2;
	tc.usr=usr;
	tc.tool=tool;
	tc.canon_motion_type=2;
	if (tcqPut(&tp->queue, tc) == -1) 
	{
		diagnostics( "tcqPut failed.\n");
		return -1;
	}
	endHomo.tran=end_xyz.tran;
	endHomo.rot=Rtmp2;

	CTRL_GetTCPInJointInMatrix(&endJoint,&startJoint,&endHomo,&usr,&tool);
	


	tp->goalJPos.j0=endJoint.j1;
	tp->goalJPos.j1=endJoint.j2;
	tp->goalJPos.j2=endJoint.j3;
	tp->goalJPos.j3=endJoint.j4;
	tp->goalJPos.j4=endJoint.j5;
	tp->goalJPos.j5=endJoint.j6;

	tp->goalPos.tran.x = end_xyz.tran.x;
	tp->goalPos.tran.y = end_xyz.tran.y;
	tp->goalPos.tran.z = end_xyz.tran.z;
	//printf("goalPos.x=%f goalPos.y=%f goalPos.z=%f \r\n",end_xyz.tran.x,end_xyz.tran.y,end_xyz.tran.z);
	//printf("tc.vMax=%f \r\n",tc.vMax);
//		tp->goalPos.a = end_abc.tran.x ;
//		tp->goalPos.b = end_abc.tran.y;
//		tp->goalPos.c = end_abc.tran.z;	
	tp->goalFrame.rot= Rtmp2 ;
	
	tp->done = 0;
	tp->depth = tcqLen(&tp->queue);
	tp->nextId++;


	return 0;
}
int tpAddLine(TP_STRUCT * tp, PmJoint end, int type)
{

    TC_STRUCT tc;
    PmLine line_xyz, line_uvw, line_abc;
    PmPose start_xyz, end_xyz;
    PmPose start_uvw, end_uvw;
    PmPose start_abc, end_abc;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
    double joint_pos[8];
    int i;
    RobotPose world;
    PmRotationMatrix Rtmp1,Rtmp2,Rtmp3,Rtmp4,Rm1,Rm2;
    PmCartesian Pm,tmp1,tmp2;

    if (!tp) {
        diagnostics( "TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        diagnostics("TP is aborting\n");
	return -1;
    }

	tc.vMax = tp->vMax;
	tc.aMax = tp->aMax;
	tc.vLimit = tp->vLimit;
	tc.wMax = tp->wMax;
	tc.wDotMax = tp->wDotMax;
	tc.wLimit = tp->wLimit;
     
	tc.canon_motion_type = 1;

	joint_pos[0] = tp->goalJPos.j0;
	joint_pos[1] = tp->goalJPos.j1;
	joint_pos[2] = tp->goalJPos.j2;
	joint_pos[3] = tp->goalJPos.j3;
	joint_pos[4] = tp->goalJPos.j4;
	joint_pos[5] = tp->goalJPos.j5;

	kinematicsForward(joint_pos,&world,&Rtmp1,&fflags,&iflags);
//	Rtmp1=tp->goalFrame.rot;
	tc.robot_R0= Rtmp1;
	//	diagnostics("tc.robot_R0=\n%d,%d,%d,\n%d,%d,%d,\n%d,%d,%d,\n",(int)(1000*tc.robot_R0.x.x),(int)(1000*tc.robot_R0.x.y),(int)(1000*tc.robot_R0.x.z),(int)(1000*tc.robot_R0.y.x),(int)(1000*tc.robot_R0.y.y),(int)(1000*tc.robot_R0.y.z),(int)(1000*tc.robot_R0.z.x),(int)(1000*tc.robot_R0.z.y),(int)(1000*tc.robot_R0.z.z));


 //diagnostics("tp->goalJPos end joint %d,%d,%d,%d,%d,%d\n",
 //		(int)(1000*tp->goalJPos.j0),(int)(1000*tp->goalJPos.j1),(int)(1000*tp->goalJPos.j2),
 //		(int)(1000*tp->goalJPos.j3),(int)(1000*tp->goalJPos.j4),(int)(1000*tp->goalJPos.j5));
 //diagnostics("tpAddLine end joint %d,%d,%d,%d,%d,%d\n",
 //		(int)(1000*end.j0),(int)(1000*end.j1),(int)(1000*end.j2),
 //		(int)(1000*end.j3),(int)(1000*end.j4),(int)(1000*end.j5));
	start_xyz.tran.x=world.tran.x;
	start_xyz.tran.y=world.tran.y;
	start_xyz.tran.z=world.tran.z;

	joint_pos[0]=end.j0;
	joint_pos[1]=end.j1;
	joint_pos[2]=end.j2;
	joint_pos[3]=end.j3;
	joint_pos[4]=end.j4;
	joint_pos[5]=end.j5;
	kinematicsForward(joint_pos,&world,&Rtmp2,&fflags,&iflags);
	end_xyz.tran.x=world.tran.x;
	end_xyz.tran.y=world.tran.y;
	end_xyz.tran.z=world.tran.z;

	pmMatInv(Rtmp1, &Rm1);
	pmMatMatMult(Rm1, Rtmp2,&Rm2);
	pmMatRotConvert(Rm2, &Vector);
	Pm.x=Vector.x;
	Pm.y=Vector.y;
	Pm.z=Vector.z;
	pmMatCartMult(Rtmp1, Pm, &tmp1);
	Vector.x=tmp1.x;
	Vector.y=tmp1.y;
	Vector.z=tmp1.z;

	start_abc.tran.x =0.0;
	start_abc.tran.y =0.0;
	start_abc.tran.z =0.0;

	end_abc.tran.x=Vector.s*180/PM_PI;
	end_abc.tran.y=0.0;
	end_abc.tran.z=0.0;

    

    start_uvw.tran.x = tp->goalPos.u;
    start_uvw.tran.y = tp->goalPos.v;
    start_uvw.tran.z = tp->goalPos.w;
    end_uvw.tran.x = tp->goalPos.u;
    end_uvw.tran.y = tp->goalPos.v;
    end_uvw.tran.z = tp->goalPos.w;

    start_xyz.rot = identity_quat;
    end_xyz.rot = identity_quat;
    start_uvw.rot = identity_quat;
    end_uvw.rot = identity_quat;
    start_abc.rot = identity_quat;
    end_abc.rot = identity_quat;


    pmLineInit(&line_xyz, start_xyz, end_xyz);
    pmLineInit(&line_uvw, start_uvw, end_uvw);
    pmLineInit(&line_abc, start_abc, end_abc);
    tcInit(&tc);
    tc.Vector= Vector;
    tc.cycle_time = tp->cycleTime;
   if (!line_xyz.tmag_zero){ 
	if(line_abc.tmag*tc.vMax > line_xyz.tmag*tc.wMax)
	{
	tc.reqvel =  tp->wMax;
	tc.maxaccel = tp->wDotMax;
	tc.maxvel= tp->wLimit;
        tc.target = line_abc.tmag;
	}
	else
	{
	tc.target = line_xyz.tmag;
	tc.reqvel = tp->vMax;
	tc.maxaccel = tp->aMax;
	tc.maxvel = tp->vLimit;
	}
   }else if (!line_uvw.tmag_zero){
	tc.target = line_uvw.tmag;
   }else{
	tc.reqvel =  tp->wMax;
	tc.maxaccel = tp->wDotMax;
	tc.maxvel= tp->wLimit;
        tc.target = line_abc.tmag;
   }

    tc.progress = 0.0;
    tc.accScale=tp->accScale;
    tc.PL=tp->PL;
    tc.stopcondition=tp->stopcondition;
    tc.ioaddress=tp->ioaddress;
    tc.iotype=tp->iotype;
    tc.addrtype=tp->addrtype;

    tc.id = tp->nextId;
    for (i=0;i<MAX_FILENAME_LENGTH;i++)
        tc.fileName[i]= tp->fileName[i];

    tc.active = 0;
    tc.feed_override = 1.0;
    tc.free=0;
    tc.joint_type=2;
    tc.coords.line.xyz = line_xyz;
    tc.coords.line.uvw = line_uvw;
    tc.coords.line.abc = line_abc;
    tc.motion_type = TC_LINEAR;
   
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;

    tc.synchronized = tp->synchronized;
 //   tc.enables = enables;
    tc.goalFrame.rot=Rtmp2;

	if (tcqPut(&tp->queue, tc) == -1) {
		diagnostics( "tcqPut failed.\n");
		return -1;
	}

	if(1 == type){
		tp->goalJPos=end;}

		tp->goalPos.tran.x = end_xyz.tran.x;
		tp->goalPos.tran.y = end_xyz.tran.y;
		tp->goalPos.tran.z = end_xyz.tran.z;
		//printf("goalPos.x=%f goalPos.y=%f goalPos.z=%f \r\n",end_xyz.tran.x,end_xyz.tran.y,end_xyz.tran.z);
		//printf("tc.vMax=%f \r\n",tc.vMax);
//		tp->goalPos.a = end_abc.tran.x ;
//		tp->goalPos.b = end_abc.tran.y;
//		tp->goalPos.c = end_abc.tran.z;	
		tp->goalFrame.rot= Rtmp2 ;
	
		tp->done = 0;
		tp->depth = tcqLen(&tp->queue);
		tp->nextId++;

    return 0;

}

int tpAddFreeLine(TP_STRUCT * tp, double end,int cord_type,int axis_num)
{
	TC_STRUCT tc;
	PmLine line_xyz, line_uvw, line_abc;
	PmPose start_xyz, end_xyz;
	PmPose start_uvw, end_uvw;
	PmPose start_abc, end_abc;
	PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
	double positions[MAX_JOINTS];
	RobotPose world;
	PmCartesian tmp;
	PmRotationMatrix RS;
	PmCartesian Pm,tmp1;
	PmCartesian usrT;
	JointPoint usrJ;
	printf("tpAddFreeLine: end=%d      \n", (int)(1000*end));
	if (!tp) 
	{
        	diagnostics("TP is null\n");
        	return -1;
    	}
	if (tp->aborting)
	{
       	diagnostics("TP is aborting\n");
		return -1;
	}
	world.a=0.0;
	world.b=0.0;
	world.c=0.0;


	tc.feed_override = emcmotDebug->net_feed_scale[axis_num];
	tc.joint_num= axis_num;
	positions[0] = emcmotStatus->joint_cmd.j0;
	positions[1] = emcmotStatus->joint_cmd.j1;
	positions[2] = emcmotStatus->joint_cmd.j2;
	positions[3] = emcmotStatus->joint_cmd.j3;
	positions[4] = emcmotStatus->joint_cmd.j4;
	positions[5] = emcmotStatus->joint_cmd.j5;
	usrJ.j1=positions[0];
	usrJ.j2=positions[1];
	usrJ.j3=positions[2];
	usrJ.j4=positions[3];
	usrJ.j5=positions[4];
	usrJ.j6=positions[5];
	kinematicsForward(positions,&world,&Rtmp,&fflags,&iflags);	
	emcmotDebug->robot_R0 = Rtmp;
	emcmotDebug->robot_P0 = world.tran;
	start_xyz.tran = world.tran;
	start_abc.tran.x=world.a;	
	start_abc.tran.y=world.b;	
	start_abc.tran.z=world.c;	
	end_xyz.tran=world.tran;
	end_abc.tran=start_abc.tran;
	switch(axis_num)
	{
		case 0:
		//printf("j0=%f  j1=%f j2=%f j3=%f j4=%f  j5=%f\n",positions[0],positions[1],positions[2],positions[3],positions[4],positions[5]);			
			if(1==cord_type)
			{
				end_xyz.tran.x=end;
			}
			else if(2==cord_type)
			{
				
				CTRL_GetTCPInUserSpace(&motionFb->usrCmd,&usrJ,&motionFb->user,&motionFb->tool);
				end=end-motionFb->usrCmd.x;
				pmCartScalMult(emcmotDebug->Xvector, end, &tmp);
				pmCartCartAdd(tmp,start_xyz.tran,&end_xyz.tran);
			}
			else
			{
				pmMatMatMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcf,&RS);
				emcmotDebug->robot_RT =RS;				
				pmCartScalMult(RS.x, end, &tmp);
				pmCartCartAdd(tmp,start_xyz.tran,&end_xyz.tran);
				Vector.x =RS.x.x;
       			Vector.y =RS.x.y;
       			Vector.z =RS.x.z;	
			}		
			break;
		case 1:
			if(1==cord_type)
			{
				end_xyz.tran.y=end;
			}
			else if(2==cord_type)
			{
				CTRL_GetTCPInUserSpace(&motionFb->usrCmd,&usrJ,&motionFb->user,&motionFb->tool);
				end=end-motionFb->usrCmd.y;
				pmCartScalMult(emcmotDebug->Yvector, end, &tmp);
				pmCartCartAdd(tmp,start_xyz.tran,&end_xyz.tran);
			}
			else
			{
				pmMatMatMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcf,&RS);
				emcmotDebug->robot_RT =RS;				
				pmCartScalMult(RS.y, end, &tmp);
				pmCartCartAdd(tmp,start_xyz.tran,&end_xyz.tran);	
				Vector.x =RS.y.x;
       			Vector.y =RS.y.y;
       			Vector.z =RS.y.z;	
			}
			break;
		case 2:
			if(1==cord_type)
			{
				end_xyz.tran.z=end;
			}
			else if(2==cord_type)
			{
				CTRL_GetTCPInUserSpace(&motionFb->usrCmd,&usrJ,&motionFb->user,&motionFb->tool);
				end=end-motionFb->usrCmd.z;
				pmCartScalMult(emcmotDebug->Zvector, end, &tmp);
				pmCartCartAdd(tmp,start_xyz.tran,&end_xyz.tran);
			}
			else
			{
				pmMatMatMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcf,&RS);
				emcmotDebug->robot_RT =RS;				
				pmCartScalMult(RS.z, end, &tmp);
				pmCartCartAdd(tmp,start_xyz.tran,&end_xyz.tran);
				Vector.x =RS.z.x;
       			Vector.y =RS.z.y;
       			Vector.z =RS.z.z;	
			}
			break;
		case 3:
			CTRL_GetTCPInUserSpace(&motionFb->usrCmd,&usrJ,&motionFb->user,&motionFb->tool);
			end_abc.tran.x=end-motionFb->usrCmd.a;
			if(1==cord_type)
			{
				Vector.x=1.0;
			      	Vector.y=0.0;
			      	Vector.z=0.0;
			}
			else if(2==cord_type)
			{
				Vector.x =emcmotDebug->Xvector.x;
       			Vector.y =emcmotDebug->Xvector.y;
       			Vector.z =emcmotDebug->Xvector.z;
			}
			else
			{
				pmMatMatMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcf,&RS);
				emcmotDebug->robot_RT =RS;	
				Vector.x =RS.x.x;
       			Vector.y =RS.x.y;
       			Vector.z =RS.x.z;
			}
			break;
		case 4:
			CTRL_GetTCPInUserSpace(&motionFb->usrCmd,&usrJ,&motionFb->user,&motionFb->tool);
			end_abc.tran.x=end-motionFb->usrCmd.b;
			if(1==cord_type)
			{
				Vector.x=0.0;
			      	Vector.y=1.0;
			      	Vector.z=0.0;
			}
			else if(2==cord_type)
			{
				Vector.x =emcmotDebug->Yvector.x;
       			Vector.y =emcmotDebug->Yvector.y;
       			Vector.z =emcmotDebug->Yvector.z;
			}
			else
			{
				pmMatMatMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcf,&RS);
				emcmotDebug->robot_RT =RS;	
				Vector.x =RS.y.x;
       			Vector.y =RS.y.y;
       			Vector.z =RS.y.z;	
			}
			break;
		case 5:
			CTRL_GetTCPInUserSpace(&motionFb->usrCmd,&usrJ,&motionFb->user,&motionFb->tool);
			end_abc.tran.x=end-motionFb->usrCmd.c;
			if(1==cord_type)
			{
				Vector.x=0.0;
			      	Vector.y=0.0;
			      	Vector.z=1.0;
			}
			else if(2==cord_type)
			{
				Vector.x =emcmotDebug->Zvector.x;
       			Vector.y =emcmotDebug->Zvector.y;
       			Vector.z =emcmotDebug->Zvector.z;
			}
			else
			{
				pmMatMatMult(emcmotDebug->robot_R0,emcmotDebug->robot_tcf,&RS);
				emcmotDebug->robot_RT =RS;	
				Vector.x =RS.z.x;
       			Vector.y =RS.z.y;
       			Vector.z =RS.z.z;		
			}
			break;
		default:
			break;
	}
	start_abc.rot = identity_quat;
	end_abc.rot = identity_quat;
	pmLineInit(&line_xyz, start_xyz, end_xyz);
	//pmLineInit(&line_uvw, start_uvw, end_uvw);
	pmLineInit(&line_abc, start_abc, end_abc);	
	if(axis_num<3)
	{
		tc.target = line_xyz.tmag;	
	}
	else
	{
		line_xyz.tmag=0;
		tc.target = line_abc.tmag;
	}
			
	start_uvw.tran.x = tp->goalPos.u;
	start_uvw.tran.y = tp->goalPos.v;
	start_uvw.tran.z = tp->goalPos.w;
	end_uvw.tran.x = tp->goalPos.u;
	end_uvw.tran.y = tp->goalPos.v;
	end_uvw.tran.z = tp->goalPos.w;

    	start_uvw.rot = identity_quat;
    	end_uvw.rot = identity_quat;

	pmLineInit(&line_uvw, start_uvw, end_uvw);

//! \todo FIXEME - calculate the unit vector from start to end( as: line->uVec), and it calculate other
//   values, such as line->start, line->end, line->tmag, line->tmag_zero etc,which will be used by other
//    functions later. 
//// 2010.05.18 byf
      tcInit(&tc);
      tc.cycle_time = tp->cycleTime;
	tc.progress = 0.0;
	if (!line_xyz.tmag_zero)
	{
		tc.reqvel = tp->vMax;
		tc.maxaccel = tp->aMax;
		tc.maxvel = tp->vLimit;
	}
	else
	{
		tc.reqvel = tp->wMax;
		tc.maxaccel = tp->wDotMax;
		tc.maxvel = tp->wLimit;
    	}
	tc.active = 0;
	tc.accScale=tp->accScale;
//    tc.feed_override = 0.0;
	tc.free=1;
	tc.joint_type=2;
    	tc.coords.line.xyz = line_xyz;
    	tc.coords.line.uvw = line_uvw;
    	tc.coords.line.abc = line_abc;
    	tc.motion_type = TC_LINEAR;
    	tc.blend_with_next = 0;            // 2011.07.27
   
//    tc.enables = enables;

	if (tcqPut(&tp->queue, tc) == -1)
	{
       	diagnostics( "tcqPut failed.\n");
		return -1;
	}
	tp->done = 0;
	tp->depth = tcqLen(&tp->queue);
	tp->nextId++;

	return 0;
}
static double DETA (PmRotationMatrix  *A) //行列式计算
{
return (A->x.x*A->y.y*A->z.z+A->y.x*A->z.y*A->x.z+A->z.x*A->x.y*A->y.z
	-A->z.x*A->y.y*A->x.z-A->x.x*A->z.y*A->y.z-A->y.x*A->x.y*A->z.z); 
}

static void swap (PmRotationMatrix *A,double *B,int k)//常数项与k列系数交换
{
	double  t;
	if(0==k){
		t = B[0];B[0] = A->x.x;A->x.x=t;
		t = B[1];B[1] = A->y.x;A->y.x=t;
		t = B[2];B[2] = A->z.x;A->z.x=t;
	}else if(1==k){
		t = B[0];B[0] = A->x.y;A->x.y=t;
		t = B[1];B[1] = A->y.y;A->y.y=t;
		t = B[2];B[2] = A->z.y;A->z.y=t;
	}else if(2==k){
		t = B[0];B[0] = A->x.z;A->x.z=t;
		t = B[1];B[1] = A->y.z;A->y.z=t;
		t = B[2];B[2] = A->z.z;A->z.z=t;
	}
} 
/*
int tpAdd3DCircleDesc(TP_STRUCT * tp, PmHomogeneous end,PmHomogeneous end2,PmHomogeneous end3)
{
TC_STRUCT tc;
Pm3DCircle circle;
PmLine line_uvw, line_abc;
PmPose start_xyz, middle_xyz, end_xyz;
PmPose start_uvw, end_uvw;
PmPose start_abc, middle_abc, end_abc;

PmCartesian center; 
PmCartesian normal;
int turn = 0;
int i;
double positions[6];
RobotPose world;
PmCartesian v1, v2, uVec;
double delta1, delta2, delta;
PmRotationMatrix A;
double B[3];
double d;
double tempA1,tempA2,tempB1,tempB2,tempC1,tempC2,temp;
PmRotationMatrix Rtmp1,Rtmp2,Rtmp3,Rm1,Rm2;
PmCartesian Pm,tmp1;
if (!tp) 
	{
        printf("TP is null\n");
        return -1;
	}
if (tp->aborting)
	{
	printf( "TP is aborting\n");
	return -1;
	}
tc.vMax = tp->vMax;
tc.aMax = tp->aMax*tp->accScale;
tc.vLimit = tp->vLimit;
tc.wMax = tp->wMax;
tc.wDotMax = tp->wDotMax*tp->accScale;
tc.wLimit = tp->wLimit;
tc.robot_R0=end.rot;
Rtmp1=end.rot;
start_xyz.tran = end.tran;
start_abc.tran.x = 0.0;
start_abc.tran.y = 0.0;
start_abc.tran.z = 0.0;
middle_xyz.tran = end2.tran;
middle_abc.tran.x = 0.0;
middle_abc.tran.y = 0.0;
middle_abc.tran.z = 0.0;
end_xyz.tran=end3.tran;
Rtmp2=end3.rot;
	pmMatInv(Rtmp1, &Rm1);
	pmMatMatMult(Rm1, Rtmp2,&Rm2);
	pmMatRotConvert(Rm2, &Vector);
	Pm.x=Vector.x;
	Pm.y=Vector.y;
	Pm.z=Vector.z;
	pmMatCartMult(Rtmp1, Pm, &tmp1);
	Vector.x=tmp1.x;
	Vector.y=tmp1.y;
	Vector.z=tmp1.z;

	start_abc.tran.x =0.0;
	start_abc.tran.y =0.0;
	start_abc.tran.z =0.0;


	end_abc.tran.x=Vector.s*180/PM_PI;
	end_abc.tran.y=0.0;
	end_abc.tran.z=0.0;

 //     world.a = newEnd(start_abc.tran.x,world.a);
 //     world.b = newEnd(start_abc.tran.y,world.b);
 //     world.c = newEnd(start_abc.tran.z,world.c);


//	if(world.a-start_abc.tran.x>180.0)
	//	world.a-=360.0;
//	else if(world.a-start_abc.tran.x<=-180.0)
	//	world.a+=360.0;
//	if(world.b-start_abc.tran.y>180.0)
//		world.b-=360.0;
//	else if(world.b-start_abc.tran.y<=-180.0)
//		world.b+=360.0;
//	if(world.c-start_abc.tran.z>180.0)
//		world.c-=360.0;
//	else if(world.c-start_abc.tran.z<=-180.0)
//		world.c+=360.0;


//	end_abc.tran.x=world.a;
//	end_abc.tran.y=world.b;
//	end_abc.tran.z=world.c;

	pmCartCartSub(middle_xyz.tran, start_xyz.tran, &v1);
	pmCartMag(v1, &delta1);
	pmCartCartSub(end_xyz.tran, middle_xyz.tran, &v2);
	pmCartMag(v2, &delta2);

	if (delta1 < 0.000001 || delta2 < 0.000001) {
		return tpAddLine(tp, end3, 1);
	}

	pmCartUnit(v2, &uVec);
	pmCartCartDot(v1, uVec, &delta);
	if (delta1 - fabs(delta) < 0.000001) {
		return tpAddLine(tp, end3, 1);
	}

      pmCartCartCross(v1, v2, &normal);
      pmCartUnit(normal, &normal);

	if (fabs(normal.z) > 0.999999) {
//		diagnostics("dddddddd\n");

	tempA1=start_xyz.tran.x-middle_xyz.tran.x;
	tempB1=start_xyz.tran.y-middle_xyz.tran.y;
	tempC1=(pow(start_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(start_xyz.tran.y,2)-pow(middle_xyz.tran.y,2))/2;

	tempA2=end_xyz.tran.x -middle_xyz.tran.x;
	tempB2=end_xyz.tran.y -middle_xyz.tran.y;
	tempC2=(pow(end_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(end_xyz.tran.y,2)-pow(middle_xyz.tran.y,2))/2;

	temp=tempA1*tempB2-tempA2*tempB1;

	center.x =(tempC1*tempB2-tempC2*tempB1)/temp;
	center.y =(tempA1*tempC2-tempA2*tempC1)/temp;
	center.z = start_xyz.tran.z;
	}
	else if (fabs(normal.x) > 0.999999) {
//		diagnostics("cccccccccc\n");

	tempA1=start_xyz.tran.y-middle_xyz.tran.y;
	tempB1=start_xyz.tran.z-middle_xyz.tran.z;
	tempC1=(pow(start_xyz.tran.y,2)-pow(middle_xyz.tran.y,2)+pow(start_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	tempA2=end_xyz.tran.y -middle_xyz.tran.y;
	tempB2=end_xyz.tran.z -middle_xyz.tran.z;
	tempC2=(pow(end_xyz.tran.y,2)-pow(middle_xyz.tran.y,2)+pow(end_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	temp=tempA1*tempB2-tempA2*tempB1;

	center.y =(tempC1*tempB2-tempC2*tempB1)/temp;
	center.z =(tempA1*tempC2-tempA2*tempC1)/temp;
	center.x = start_xyz.tran.x;

	}
	else if (fabs(normal.y) > 0.999999) {
	//	diagnostics("aaaaaaa\n");

	tempA1=start_xyz.tran.x-middle_xyz.tran.x;
	tempB1=start_xyz.tran.z-middle_xyz.tran.z;
	tempC1=(pow(start_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(start_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	tempA2=end_xyz.tran.x -middle_xyz.tran.x;
	tempB2=end_xyz.tran.z -middle_xyz.tran.z;
	tempC2=(pow(end_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(end_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	temp=tempA1*tempB2-tempA2*tempB1;

	center.x =(tempC1*tempB2-tempC2*tempB1)/temp;
	center.z =(tempA1*tempC2-tempA2*tempC1)/temp;
	center.y = start_xyz.tran.y;

	}
	else {
//		diagnostics("bbbbbbbbb\n");
	A.x.x = 2*(start_xyz.tran.x-middle_xyz.tran.x);
	A.x.y = 2*(start_xyz.tran.y-middle_xyz.tran.y );
	A.x.z = 2*( start_xyz.tran.z-middle_xyz.tran.z );
	A.y.x = 2*(middle_xyz.tran.x- end_xyz.tran.x );
	A.y.y = 2* ( middle_xyz.tran.y - end_xyz.tran.y );
	A.y.z = 2 *( middle_xyz.tran.z - end_xyz.tran.z );
	A.z.x =A.x.z* (A.x.z*A.y.y-A.x.y*A.y.z)/8.0;
	A.z.y =A.x.z* (A.x.x *A.y.z-A.x.z*A.y.x)/8.0;
	A.z.z=-(A.x.x* (A.x.z*A.y.y-A.x.y*A.y.z)+A.x.y*(A.x.x*A.y.z-A.x.z*A.y.x))/8.0;

	B[0]=start_xyz.tran.x*start_xyz.tran.x+start_xyz.tran.y*start_xyz.tran.y+start_xyz.tran.z*start_xyz.tran.z-middle_xyz.tran.x*middle_xyz.tran.x-middle_xyz.tran.y*middle_xyz.tran.y-middle_xyz.tran.z*middle_xyz.tran.z;
	B[1]=middle_xyz.tran.x*middle_xyz.tran.x+middle_xyz.tran.y*middle_xyz.tran.y+middle_xyz.tran.z*middle_xyz.tran.z-end_xyz.tran.x*end_xyz.tran.x-end_xyz.tran.y*end_xyz.tran.y-end_xyz.tran.z*end_xyz.tran.z;
	B[2]= A.z.x*start_xyz.tran.x+ A.z.y*start_xyz.tran.y + A.z.z*start_xyz.tran.z;

	d=DETA(&A); 
	swap (&A,&B[0],0) ; 
	center.x=DETA(&A)/d; 
	swap (&A,&B[0],0) ;
	swap (&A,&B[0],1) ; 
	center.y=DETA(&A)/d; 
	swap (&A,&B[0],1) ; 
	swap (&A,&B[0],2) ; 
	center.z=DETA(&A)/d;
	swap (&A,&B[0],2) ;
//diagnostics("(int)(1000*center.x)=%d,%d,%d    \n",(int)(1000*center.x),(int)(1000*center.y),(int)(1000*center.z));
	}
 	
 //diagnostics("tpAddCircle start %d,%d,%d,%d,%d,%d\n",
 //	(int)(start_xyz.tran.x),(int)(start_xyz.tran.y),(int)(start_xyz.tran.z),
 //	(int)(start_abc.tran.x),(int)(start_abc.tran.y),(int)(start_abc.tran.z));
 //diagnostics("tpAddCircle start joint %d,%d,%d,%d,%d,%d\n",
 	//	(int)(end.j0),(int)(end.j1),(int)(end.j2),
 	//	(int)(end.j3),(int)(end.j4),(int)(end.j5));

// diagnostics("tpAddCircle end %d,%d,%d,%d,%d,%d\n",
 	//	(int)(end_xyz.tran.x),(int)(end_xyz.tran.y),(int)(end_xyz.tran.z),
 	//	(int)(end_abc.tran.x),(int)(end_abc.tran.y),(int)(end_abc.tran.z));
// diag//nostics("tpAddCircle end joint %d,%d,%d,%d,%d,%d\n",
 	//	(int)(end3.j0),(int)(end3.j1),(int)(end3.j2),
 	//	(int)(end3.j3),(int)(end3.j4),(int)(end3.j5));

	tcInit(&tc);

	if(-1 == pm3DCircleInit(&circle, start_xyz,middle_xyz, end_xyz, center, normal, turn))
  	{   
    		return -1;
  	} 
    pmLineInit(&line_uvw, start_uvw, end_uvw);
    pmLineInit(&line_abc, start_abc, end_abc);
//diagnostics("line_abc.tmag = %d \n",(int)(1000*line_abc.tmag));

    tc.cycle_time = tp->cycleTime;
    tc.accScale=tp->accScale;
    tc.progress = 0.0;

    tc.id = tp->nextId;
       for (i=0;i<MAX_FILENAME_LENGTH;i++)
        tc.fileName[i]= tp->fileName[i];
    tc.active = 0;
    tc.PL=tp->PL;
    tc.stopcondition=tp->stopcondition;
    tc.ioaddress=tp->ioaddress;
//diagnostics("tc.target = %d \n",(int)(1000*tc.target));
    tc.feed_override = 1.0;
    tc.free=0;
    tc.joint_type=2;
 //   tc.reqvel = tp->vMax; 
 //   tc.maxaccel = tp->aMax*tp->accScale;
 //   tc.maxvel = tp->vLimit;


//    if (!line_abc.tmag_zero) {
	//	if (line_abc.tmag * tc.vMax > tc.target * tc.wMax)
		//	tc.reqvel = tc.target * tc.wMax / line_abc.tmag;
//diagnostics("tc.reqvel = %d \n",(int)(1000*tc.reqvel));

	//	if (tc.wMax * tc.aMax > tc.vMax * tc.wDotMax)
		//	tc.maxaccel = tc.vMax * tc.wDotMax / tc.wMax;
//diagnostics("tc.maxaccel = %d \n",(int)(1000*tc.maxaccel));

//}
	
	if (!circle.tmag_zero){ 
		if(line_abc.tmag*tc.vMax >circle.tmag*tc.wMax)
		{
		tc.reqvel =  tp->wMax;
		tc.maxaccel = tp->wDotMax;
		tc.maxvel= tp->wLimit;
	        tc.target = line_abc.tmag;
		}
		else
		{
		tc.target = circle.tmag;
		tc.reqvel = tp->vMax;
		tc.maxaccel = tp->aMax;
		tc.maxvel = tp->vLimit;
		}
   }else if (!line_uvw.tmag_zero){
		tc.target = line_uvw.tmag;
   }else{
		tc.reqvel =  tp->wMax;
		tc.maxaccel = tp->wDotMax;
		tc.maxvel= tp->wLimit;
		tc.target = line_abc.tmag;
   }


    tc.coords.circle3D.xyz = circle;
    tc.coords.circle3D.uvw = line_uvw;
    tc.coords.circle3D.abc = line_abc;

    tc.motion_type = TC_3DCIRCULAR;
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;

    tc.synchronized = tp->synchronized;
    tc.goalFrame.rot=Rtmp2;

	if (-1 == tcqPut(&tp->queue, tc)) 
	{
		diagnostics("error in putcircle\n");
	    	return -1;
	}


	tp->goalJPos = end3;
	tp->goalPos.tran=end_xyz.tran;
//	tp->goalPos.a=end_abc.tran.x;
//	tp->goalPos.b=end_abc.tran.y;
//	tp->goalPos.c=end_abc.tran.z;

	tp->done = 0;
	tp->depth = tcqLen(&tp->queue);
	tp->nextId++;

	return 0;



return 0;
}
*/
int tpAdd3DUsrCircle(TP_STRUCT * tp, PointPose end,PointPose end2,PointPose end3,UserCoordianteInformation usr,ToolCoordianteInformation tool) 
{
	TC_STRUCT tc;
	Pm3DCircle circle;
	PmLine line_uvw, line_abc;
	PmPose start_xyz, middle_xyz, end_xyz;
	PmPose start_uvw, end_uvw;
	PmPose start_abc, middle_abc, end_abc;
	PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
	start_uvw.rot = identity_quat;
	start_uvw.tran.x = 0;
	start_uvw.tran.y = 0;
	start_uvw.tran.z = 0;

	end_uvw.rot = identity_quat;
	end_uvw.tran.x = 0;
	end_uvw.tran.y = 0;
	end_uvw.tran.z = 0;

	PmCartesian center; 
	PmCartesian normal;
	int turn = 0;
	int i;
	double positions[6];
	RobotPose world;
	PmCartesian v1, v2, uVec;
	double delta1, delta2, delta;
	PmRotationMatrix A;
	double B[3];
	double d;
	double tempA1,tempA2,tempB1,tempB2,tempC1,tempC2,temp;
	PmRotationMatrix Rtmp1,Rtmp2,Rtmp3,Rm1,Rm2;
	PmCartesian Pm,tmp1;
	JointPoint startJoint,endJoint;
	PmHomogeneous start,endHomo;
	PmEulerZyx tempZyx;
	PmJoint tempJoint;
	if (!tp) 
	{
		printf("TP is null\n");
		return -1;
	}
	if (tp->aborting)
	{
		printf( "TP is aborting\n");
		return -1;
	}

	tc.vMax = tp->vMax;
	tc.aMax = tp->aMax*tp->accScale;
	tc.vLimit = tp->vLimit;
	tc.wMax = tp->wMax;
	tc.wDotMax = tp->wDotMax*tp->accScale;
	tc.wLimit = tp->wLimit;
	tc.canon_motion_type = 2;
	startJoint.j1=tp->goalJPos.j0;
	startJoint.j2=tp->goalJPos.j1;
	startJoint.j3=tp->goalJPos.j2;
	startJoint.j4=tp->goalJPos.j3;
	startJoint.j5=tp->goalJPos.j4;
	startJoint.j6=tp->goalJPos.j5;

	endHomo.tran.x=end.x;
	endHomo.tran.y=end.y;
	endHomo.tran.z=end.z;
	tempZyx.x=end.a*PM_PI/180.0;
	tempZyx.y=end.b*PM_PI/180.0;
	tempZyx.z=end.c*PM_PI/180.0;
	pmZyxMatConvert(tempZyx, &(endHomo.rot));
	if(!CTRL_GetTCPInJointInMatrix(&endJoint,&startJoint,&endHomo,&usr,&tool))
	{	
		tempJoint.j0 = endJoint.j1;
		tempJoint.j1 = endJoint.j2;
		tempJoint.j2 = endJoint.j3;
		tempJoint.j3 = endJoint.j4;
		tempJoint.j4 = endJoint.j5;
		tempJoint.j5 = endJoint.j6;
		tpAddLine(tp, tempJoint, 1);
	}
	else
	{
		printf("add line failed \r\n");
		return -1;
	}

	start_xyz.tran=endHomo.tran;
	Rtmp1=endHomo.rot;
	tc.robot_R0= endHomo.rot;
	
	middle_xyz.tran.x=end2.x;
	middle_xyz.tran.y=end2.y;
	middle_xyz.tran.z=end2.z;

	end_xyz.tran.x=end3.x;
	end_xyz.tran.y=end3.y;
	end_xyz.tran.z=end3.z;
	
	tempZyx.x=end2.a*PM_PI/180.0;
	tempZyx.y=end2.b*PM_PI/180.0;
	tempZyx.z=end2.c*PM_PI/180.0;
	pmZyxMatConvert(tempZyx, &Rtmp);

	pmMatInv(Rtmp1, &Rm1);
	pmMatMatMult(Rm1, Rtmp,&Rm2);
	pmMatRotConvert(Rm2, &Vector);
	Pm.x=Vector.x;
	Pm.y=Vector.y;
	Pm.z=Vector.z;
	pmMatCartMult(Rtmp1, Pm, &tmp1);
	Vector.x=tmp1.x;
	Vector.y=tmp1.y;
	Vector.z=tmp1.z;

	start_abc.tran.x =0.0;
	start_abc.tran.y =0.0;
	start_abc.tran.z =0.0;


	end_abc.tran.x=Vector.s*180/PM_PI;
	end_abc.tran.y=0.0;
	end_abc.tran.z=0.0;

	pmCartCartSub(middle_xyz.tran, start_xyz.tran, &v1);
	pmCartMag(v1, &delta1);
	pmCartCartSub(end_xyz.tran, middle_xyz.tran, &v2);
	pmCartMag(v2, &delta2);

	if (delta1 < 0.000001 || delta2 < 0.000001)
	{
		endHomo.tran.x=end3.x;
		endHomo.tran.y=end3.y;
		endHomo.tran.z=end3.z;
		tempZyx.x=end3.a*PM_PI/180.0;
		tempZyx.y=end3.b*PM_PI/180.0;
		tempZyx.z=end3.c*PM_PI/180.0;
		pmZyxMatConvert(tempZyx, &(endHomo.rot));
		CTRL_GetTCPInJointInMatrix(&endJoint,&startJoint,&endHomo,&usr,&tool);
		tempJoint.j0 = endJoint.j1;
		tempJoint.j1 = endJoint.j2;
		tempJoint.j2 = endJoint.j3;
		tempJoint.j3 = endJoint.j4;
		tempJoint.j4 = endJoint.j5;
		tempJoint.j5 = endJoint.j6;		
		return tpAddLine(tp, tempJoint, 1);
	}

	pmCartUnit(v2, &uVec);
	pmCartCartDot(v1, uVec, &delta);
	if (delta1 - fabs(delta) < 0.000001)
	{
		endHomo.tran.x=end3.x;
		endHomo.tran.y=end3.y;
		endHomo.tran.z=end3.z;
		tempZyx.x=end3.a*PM_PI/180.0;
		tempZyx.y=end3.b*PM_PI/180.0;
		tempZyx.z=end3.c*PM_PI/180.0;
		pmZyxMatConvert(tempZyx, &(endHomo.rot));
		CTRL_GetTCPInJointInMatrix(&endJoint,&startJoint,&endHomo,&usr,&tool);
		tempJoint.j0 = endJoint.j1;
		tempJoint.j1 = endJoint.j2;
		tempJoint.j2 = endJoint.j3;
		tempJoint.j3 = endJoint.j4;
		tempJoint.j4 = endJoint.j5;
		tempJoint.j5 = endJoint.j6;		
		return tpAddLine(tp, tempJoint, 1);
	}

      pmCartCartCross(v1, v2, &normal);
      pmCartUnit(normal, &normal);

	if (fabs(normal.z) > 0.999999)
	{
		tempA1=start_xyz.tran.x-middle_xyz.tran.x;
		tempB1=start_xyz.tran.y-middle_xyz.tran.y;
		tempC1=(pow(start_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(start_xyz.tran.y,2)-pow(middle_xyz.tran.y,2))/2;

		tempA2=end_xyz.tran.x -middle_xyz.tran.x;
		tempB2=end_xyz.tran.y -middle_xyz.tran.y;
		tempC2=(pow(end_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(end_xyz.tran.y,2)-pow(middle_xyz.tran.y,2))/2;

		temp=tempA1*tempB2-tempA2*tempB1;

		center.x =(tempC1*tempB2-tempC2*tempB1)/temp;
		center.y =(tempA1*tempC2-tempA2*tempC1)/temp;
		center.z = start_xyz.tran.z;
	}
	else if (fabs(normal.x) > 0.999999)
	{
	//		diagnostics("cccccccccc\n");
		tempA1=start_xyz.tran.y-middle_xyz.tran.y;
		tempB1=start_xyz.tran.z-middle_xyz.tran.z;
		tempC1=(pow(start_xyz.tran.y,2)-pow(middle_xyz.tran.y,2)+pow(start_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

		tempA2=end_xyz.tran.y -middle_xyz.tran.y;
		tempB2=end_xyz.tran.z -middle_xyz.tran.z;
		tempC2=(pow(end_xyz.tran.y,2)-pow(middle_xyz.tran.y,2)+pow(end_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

		temp=tempA1*tempB2-tempA2*tempB1;

		center.y =(tempC1*tempB2-tempC2*tempB1)/temp;
		center.z =(tempA1*tempC2-tempA2*tempC1)/temp;
		center.x = start_xyz.tran.x;
	}
	else if (fabs(normal.y) > 0.999999)
	{
		tempA1=start_xyz.tran.x-middle_xyz.tran.x;
		tempB1=start_xyz.tran.z-middle_xyz.tran.z;
		tempC1=(pow(start_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(start_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

		tempA2=end_xyz.tran.x -middle_xyz.tran.x;
		tempB2=end_xyz.tran.z -middle_xyz.tran.z;
		tempC2=(pow(end_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(end_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

		temp=tempA1*tempB2-tempA2*tempB1;

		center.x =(tempC1*tempB2-tempC2*tempB1)/temp;
		if (center.x < 1e-6)
			center.x = 0.0;
		center.z =(tempA1*tempC2-tempA2*tempC1)/temp;
		if (center.y < 1e-6)
			center.y = 0.0;
		center.y = start_xyz.tran.y;
	}
	else
	{
		A.x.x = 2*(start_xyz.tran.x-middle_xyz.tran.x);
		A.x.y = 2*(start_xyz.tran.y-middle_xyz.tran.y );
		A.x.z = 2*( start_xyz.tran.z-middle_xyz.tran.z );
		A.y.x = 2*(middle_xyz.tran.x- end_xyz.tran.x );
		A.y.y = 2* ( middle_xyz.tran.y - end_xyz.tran.y );
		A.y.z = 2 *( middle_xyz.tran.z - end_xyz.tran.z );
		A.z.x =A.x.z* (A.x.z*A.y.y-A.x.y*A.y.z)/8.0;
		A.z.y =A.x.z* (A.x.x *A.y.z-A.x.z*A.y.x)/8.0;
		A.z.z=-(A.x.x* (A.x.z*A.y.y-A.x.y*A.y.z)+A.x.y*(A.x.x*A.y.z-A.x.z*A.y.x))/8.0;

		B[0]=start_xyz.tran.x*start_xyz.tran.x+start_xyz.tran.y*start_xyz.tran.y+start_xyz.tran.z*start_xyz.tran.z-middle_xyz.tran.x*middle_xyz.tran.x-middle_xyz.tran.y*middle_xyz.tran.y-middle_xyz.tran.z*middle_xyz.tran.z;
		B[1]=middle_xyz.tran.x*middle_xyz.tran.x+middle_xyz.tran.y*middle_xyz.tran.y+middle_xyz.tran.z*middle_xyz.tran.z-end_xyz.tran.x*end_xyz.tran.x-end_xyz.tran.y*end_xyz.tran.y-end_xyz.tran.z*end_xyz.tran.z;
		B[2]= A.z.x*start_xyz.tran.x+ A.z.y*start_xyz.tran.y + A.z.z*start_xyz.tran.z;

		d=DETA(&A); 
		swap (&A,&B[0],0) ; 
		center.x=DETA(&A)/d; 
		swap (&A,&B[0],0) ;
		swap (&A,&B[0],1) ; 
		center.y=DETA(&A)/d; 
		swap (&A,&B[0],1) ; 
		swap (&A,&B[0],2) ; 
		center.z=DETA(&A)/d;
		swap (&A,&B[0],2) ;
	}
 	
	tcInit(&tc);

	if(-1 == pm3DCircleInit(&circle, start_xyz,middle_xyz, end_xyz, center, normal, turn))
  	{   
    		return -1;
  	} 
	pmLineInit(&line_uvw, start_uvw, end_uvw);
	pmLineInit(&line_abc, start_abc, end_abc);

	tc.cycle_time = tp->cycleTime;
	tc.accScale=tp->accScale;
	tc.progress = 0.0;

	tc.id = tp->nextId;
       for (i=0;i<MAX_FILENAME_LENGTH;i++)
	   	tc.fileName[i]= tp->fileName[i];
	tc.active = 0;
	tc.PL=tp->PL;
	tc.stopcondition=tp->stopcondition;
	tc.ioaddress=tp->ioaddress;
	tc.feed_override = 1.0;
	tc.free=0;
	tc.joint_type=2;

	
	if (!circle.tmag_zero)
	{ 
		if(line_abc.tmag*tc.vMax >circle.tmag*tc.wMax)
		{
			tc.reqvel =  tp->wMax;
			tc.maxaccel = tp->wDotMax;
			tc.maxvel= tp->wLimit;
			tc.target = line_abc.tmag;
		}
		else
		{
			tc.target = circle.tmag;
			tc.reqvel = tp->vMax;
			tc.maxaccel = tp->aMax;
			tc.maxvel = tp->vLimit;
		}
	}
	else if (!line_uvw.tmag_zero)
	{
		tc.target = line_uvw.tmag;
	}else
	{
		tc.reqvel =  tp->wMax;
		tc.maxaccel = tp->wDotMax;
		tc.maxvel= tp->wLimit;
		tc.target = line_abc.tmag;
	}

	tc.coords.circle3D.xyz = circle;
	tc.coords.circle3D.uvw = line_uvw;
	tc.coords.circle3D.abc = line_abc;

	tc.motion_type = TC_3DCIRCULAR;
	tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
	tc.tolerance = tp->tolerance;
	tc.synchronized = tp->synchronized;
	tc.goalFrame.rot=Rtmp;
	tc.usr=usr;
	tc.tool=tool;


	if (-1 == tcqPut(&tp->queue, tc)) 
	{
		printf("error in putcircle\n");
	    	return -1;
	}

	endHomo.tran.x = end3.x;
	endHomo.tran.y = end3.y;
	endHomo.tran.z = end3.z;
	tempZyx.x = end3.a*PM_PI / 180.0;
	tempZyx.y = end3.b*PM_PI / 180.0;
	tempZyx.z = end3.c*PM_PI / 180.0;
	pmZyxMatConvert(tempZyx, &(endHomo.rot));
	CTRL_GetTCPInJointInMatrix(&endJoint, &startJoint, &endHomo, &usr, &tool);
	tempJoint.j0 = endJoint.j1;
	tempJoint.j1 = endJoint.j2;
	tempJoint.j2 = endJoint.j3;
	tempJoint.j3 = endJoint.j4;
	tempJoint.j4 = endJoint.j5;
	tempJoint.j5 = endJoint.j6;
	tp->goalJPos = tempJoint;
	tp->goalPos.tran=end_xyz.tran;

	tp->done = 0;
	tp->depth = tcqLen(&tp->queue);
	tp->nextId++;

	return 0;
}

int tpAdd3DCircle(TP_STRUCT * tp, PmJoint end,PmJoint end2,PmJoint end3) 
{
	TC_STRUCT tc;
	Pm3DCircle circle;
	PmLine line_uvw, line_abc;
	PmPose start_xyz, middle_xyz, end_xyz;
	PmPose start_uvw, end_uvw;
	PmPose start_abc, middle_abc, end_abc;
	PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
	start_uvw.rot = identity_quat;
	start_uvw.tran.x = 0;
	start_uvw.tran.y = 0;
	start_uvw.tran.z = 0;

	end_uvw.rot = identity_quat;
	end_uvw.tran.x = 0;
	end_uvw.tran.y = 0;
	end_uvw.tran.z = 0;

	PmCartesian center; 
	PmCartesian normal;
	int turn = 0;
	int i;
	double positions[6];
	RobotPose world;
	PmCartesian v1, v2, uVec;
	double delta1, delta2, delta;
	PmRotationMatrix A;
	double B[3];
	double d;
	double tempA1,tempA2,tempB1,tempB2,tempC1,tempC2,temp;
	PmRotationMatrix Rtmp1,Rtmp2,Rtmp3,Rm1,Rm2;
	PmCartesian Pm,tmp1;

    if (!tp) {
        diagnostics("TP is null\n");
        return -1;
    }
    if (tp->aborting) {
        diagnostics( "TP is aborting\n");
	return -1;
    }

	printf("add 3d circle p1 = %4f %4f %4f %4f %4f %4f\n",end.j0,end.j1,end.j2,end.j3,end.j4,end.j5);
	printf("add 3d circle p2 = %4f %4f %4f %4f %4f %4f\n",end2.j0,end2.j1,end2.j2,end2.j3,end2.j4,end2.j5);
	printf("add 3d circle p3 = %4f %4f %4f %4f %4f %4f\n",end3.j0,end3.j1,end3.j2,end3.j3,end3.j4,end3.j5);

	tc.vMax = tp->vMax;
	tc.aMax = tp->aMax*tp->accScale;
	tc.vLimit = tp->vLimit;
	tc.wMax = tp->wMax;
	tc.wDotMax = tp->wDotMax*tp->accScale;
	tc.wLimit = tp->wLimit;


	
	tpAddLine(tp, end, 1);


	positions[0] =end.j0;
	positions[1] = end.j1;
	positions[2] = end.j2;
	positions[3] = end.j3;
	positions[4] = end.j4;
	positions[5] = end.j5;
	kinematicsForward(positions,&world,&Rtmp1,&fflags,&iflags);
	tc.robot_R0= Rtmp1;

	




	start_xyz.tran = world.tran;
	start_abc.tran.x = 0.0;
	start_abc.tran.y = 0.0;
	start_abc.tran.z = 0.0;
//diagnostics("tp->dispon_flag = %d       \n",tp->dispon_flag);

	positions[0]=end2.j0;
	positions[1]=end2.j1;
	positions[2]=end2.j2;
	positions[3]=end2.j3;
	positions[4]=end2.j4;
	positions[5]=end2.j5;
	kinematicsForward(positions,&world,&Rtmp,&fflags,&iflags);


	middle_xyz.tran = world.tran;
	//if(1 == tp->dispon_flag) 
	//{
	//	pmCartCartAdd(middle_xyz.tran, tp->disp_vector.tran, &(middle_xyz.tran));
	//	world.a += tp->disp_vector.a;
	//	world.b += tp->disp_vector.b;
	//	world.c += tp->disp_vector.c;

//}  

	middle_abc.tran.x = world.a;
	middle_abc.tran.y = world.b;
	middle_abc.tran.z = world.c;

	positions[0]=end3.j0;
	positions[1]=end3.j1;
	positions[2]=end3.j2;
	positions[3]=end3.j3;
	positions[4]=end3.j4;
	positions[5]=end3.j5;
	kinematicsForward(positions,&world,&Rtmp,&fflags,&iflags);


	end_xyz.tran=world.tran;

//      if(1 == tp->dispon_flag) 
//	{
//		pmCartCartAdd(end_xyz.tran, tp->disp_vector.tran, &(end_xyz.tran));
//		world.a += tp->disp_vector.a;
//		world.b += tp->disp_vector.b;
//		world.c += tp->disp_vector.c;
//

//	}

	pmMatInv(Rtmp1, &Rm1);
	pmMatMatMult(Rm1, Rtmp,&Rm2);
	pmMatRotConvert(Rm2, &Vector);
	Pm.x=Vector.x;
	Pm.y=Vector.y;
	Pm.z=Vector.z;
	pmMatCartMult(Rtmp1, Pm, &tmp1);
	Vector.x=tmp1.x;
	Vector.y=tmp1.y;
	Vector.z=tmp1.z;

	start_abc.tran.x =0.0;
	start_abc.tran.y =0.0;
	start_abc.tran.z =0.0;


	end_abc.tran.x=Vector.s*180/PM_PI;
	end_abc.tran.y=0.0;
	end_abc.tran.z=0.0;

 //     world.a = newEnd(start_abc.tran.x,world.a);
 //     world.b = newEnd(start_abc.tran.y,world.b);
 //     world.c = newEnd(start_abc.tran.z,world.c);


//	if(world.a-start_abc.tran.x>180.0)
	//	world.a-=360.0;
//	else if(world.a-start_abc.tran.x<=-180.0)
	//	world.a+=360.0;
//	if(world.b-start_abc.tran.y>180.0)
//		world.b-=360.0;
//	else if(world.b-start_abc.tran.y<=-180.0)
//		world.b+=360.0;
//	if(world.c-start_abc.tran.z>180.0)
//		world.c-=360.0;
//	else if(world.c-start_abc.tran.z<=-180.0)
//		world.c+=360.0;


//	end_abc.tran.x=world.a;
//	end_abc.tran.y=world.b;
//	end_abc.tran.z=world.c;

	pmCartCartSub(middle_xyz.tran, start_xyz.tran, &v1);
	pmCartMag(v1, &delta1);
	pmCartCartSub(end_xyz.tran, middle_xyz.tran, &v2);
	pmCartMag(v2, &delta2);

	if (delta1 < 0.000001 || delta2 < 0.000001) {
		return tpAddLine(tp, end3, 1);
	}

	pmCartUnit(v2, &uVec);
	pmCartCartDot(v1, uVec, &delta);
	if (delta1 - fabs(delta) < 0.000001) {
		return tpAddLine(tp, end3, 1);
	}

      pmCartCartCross(v1, v2, &normal);
      pmCartUnit(normal, &normal);

	if (fabs(normal.z) > 0.999999) {
//		diagnostics("dddddddd\n");

	tempA1=start_xyz.tran.x-middle_xyz.tran.x;
	tempB1=start_xyz.tran.y-middle_xyz.tran.y;
	tempC1=(pow(start_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(start_xyz.tran.y,2)-pow(middle_xyz.tran.y,2))/2;

	tempA2=end_xyz.tran.x -middle_xyz.tran.x;
	tempB2=end_xyz.tran.y -middle_xyz.tran.y;
	tempC2=(pow(end_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(end_xyz.tran.y,2)-pow(middle_xyz.tran.y,2))/2;

	temp=tempA1*tempB2-tempA2*tempB1;

	center.x =(tempC1*tempB2-tempC2*tempB1)/temp;
	center.y =(tempA1*tempC2-tempA2*tempC1)/temp;
	center.z = start_xyz.tran.z;
	}
	else if (fabs(normal.x) > 0.999999) {
//		diagnostics("cccccccccc\n");

	tempA1=start_xyz.tran.y-middle_xyz.tran.y;
	tempB1=start_xyz.tran.z-middle_xyz.tran.z;
	tempC1=(pow(start_xyz.tran.y,2)-pow(middle_xyz.tran.y,2)+pow(start_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	tempA2=end_xyz.tran.y -middle_xyz.tran.y;
	tempB2=end_xyz.tran.z -middle_xyz.tran.z;
	tempC2=(pow(end_xyz.tran.y,2)-pow(middle_xyz.tran.y,2)+pow(end_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	temp=tempA1*tempB2-tempA2*tempB1;

	center.y =(tempC1*tempB2-tempC2*tempB1)/temp;
	center.z =(tempA1*tempC2-tempA2*tempC1)/temp;
	center.x = start_xyz.tran.x;

	}
	else if (fabs(normal.y) > 0.999999) {
	//	diagnostics("aaaaaaa\n");

	tempA1=start_xyz.tran.x-middle_xyz.tran.x;
	tempB1=start_xyz.tran.z-middle_xyz.tran.z;
	tempC1=(pow(start_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(start_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	tempA2=end_xyz.tran.x -middle_xyz.tran.x;
	tempB2=end_xyz.tran.z -middle_xyz.tran.z;
	tempC2=(pow(end_xyz.tran.x,2)-pow(middle_xyz.tran.x,2)+pow(end_xyz.tran.z,2)-pow(middle_xyz.tran.z,2))/2;

	temp=tempA1*tempB2-tempA2*tempB1;

	center.x =(tempC1*tempB2-tempC2*tempB1)/temp;
	if (center.x < 1e-6)
		center.x = 0.0;
	center.z =(tempA1*tempC2-tempA2*tempC1)/temp;
	if (center.y < 1e-6)
		center.y = 0.0;
	center.y = start_xyz.tran.y;

	}
	else {
//		diagnostics("bbbbbbbbb\n");
//	A.x.x = 2*(start_xyz.tran.x-middle_xyz.tran.x);
//	A.x.y = 2*(start_xyz.tran.y-middle_xyz.tran.y );
//	A.x.z = 2*( start_xyz.tran.z-middle_xyz.tran.z );
//	A.y.x = 2*(middle_xyz.tran.x- end_xyz.tran.x );
//	A.y.y = 2* ( middle_xyz.tran.y - end_xyz.tran.y );
//	A.y.z = 2 *( middle_xyz.tran.z - end_xyz.tran.z );
//	A.z.x =A.x.z* (A.x.z*A.y.y-A.x.y*A.y.z)/8.0;
//	A.z.y =A.x.z* (A.x.x *A.y.z-A.x.z*A.y.x)/8.0;
//	A.z.z=-1*(A.x.x* (A.x.z*A.y.y-A.x.y*A.y.z)+A.x.y*(A.x.x*A.y.z-A.x.z*A.y.x))/8.0;

//	B[0]=start_xyz.tran.x*start_xyz.tran.x+start_xyz.tran.y*start_xyz.tran.y+start_xyz.tran.z*start_xyz.tran.z-middle_xyz.tran.x*middle_xyz.tran.x-middle_xyz.tran.y*middle_xyz.tran.y-middle_xyz.tran.z*middle_xyz.tran.z;
//	B[1]=middle_xyz.tran.x*middle_xyz.tran.x+middle_xyz.tran.y*middle_xyz.tran.y+middle_xyz.tran.z*middle_xyz.tran.z-end_xyz.tran.x*end_xyz.tran.x-end_xyz.tran.y*end_xyz.tran.y-end_xyz.tran.z*end_xyz.tran.z;
//	B[2]= A.z.x*start_xyz.tran.x+ A.z.y*start_xyz.tran.y + A.z.z*start_xyz.tran.z;

//	printf("3D circle: A.x = %4f %4f %4f \n",A.x.x,A.x.y,A.x.z);
//	printf("3D circle: A.y = %4f %4f %4f \n",A.y.x,A.y.y,A.y.z);
//	printf("3D circle: A.z = %4f %4f %4f \n",A.z.x,A.z.y,A.z.z);

//	printf("3D circle: B[0] = %4f  \n",B[0]);
//	printf("3D circle: B[1] = %4f  \n",B[1]);
//	printf("3D circle: B[2] = %4f  \n",B[2]);
//


//	d=DETA(&A); 
//	swap (&A,&B[0],0) ; 
//	center.x=DETA(&A)/d; 
//	swap (&A,&B[0],0) ;
//	swap (&A,&B[0],1) ; 
//	center.y=DETA(&A)/d; 
//	swap (&A,&B[0],1) ; 
//	swap (&A,&B[0],2) ; 
//	center.z=DETA(&A)/d;
//	swap (&A,&B[0],2) ;

	center = threePointCircle(start_xyz.tran,middle_xyz.tran,end_xyz.tran);

//diagnostics("(int)(1000*center.x)=%d,%d,%d    \n",(int)(1000*center.x),(int)(1000*center.y),(int)(1000*center.z));
	}
 	
 //diagnostics("tpAddCircle start %d,%d,%d,%d,%d,%d\n",
 //	(int)(start_xyz.tran.x),(int)(start_xyz.tran.y),(int)(start_xyz.tran.z),
 //	(int)(start_abc.tran.x),(int)(start_abc.tran.y),(int)(start_abc.tran.z));
 //diagnostics("tpAddCircle start joint %d,%d,%d,%d,%d,%d\n",
 	//	(int)(end.j0),(int)(end.j1),(int)(end.j2),
 	//	(int)(end.j3),(int)(end.j4),(int)(end.j5));

// diagnostics("tpAddCircle end %d,%d,%d,%d,%d,%d\n",
 	//	(int)(end_xyz.tran.x),(int)(end_xyz.tran.y),(int)(end_xyz.tran.z),
 	//	(int)(end_abc.tran.x),(int)(end_abc.tran.y),(int)(end_abc.tran.z));
// diag//nostics("tpAddCircle end joint %d,%d,%d,%d,%d,%d\n",
 	//	(int)(end3.j0),(int)(end3.j1),(int)(end3.j2),
 	//	(int)(end3.j3),(int)(end3.j4),(int)(end3.j5));

	tcInit(&tc);

	printf(" 3D circle: start_xyz = %4f %4f %4f \n",start_xyz.tran.x,start_xyz.tran.y,start_xyz.tran.z);
	printf(" 3D circle: middle_xyz = %4f %4f %4f \n",middle_xyz.tran.x,middle_xyz.tran.y,middle_xyz.tran.z);
	printf(" 3D circle: end_xyz = %4f %4f %4f \n",end_xyz.tran.x,end_xyz.tran.y,end_xyz.tran.z);
	printf(" 3D circle: center = %4f %4f %4f \n",center.x,center.y,center.z);
	printf(" 3D circle: normal = %4f %4f %4f \n",normal.x,normal.y,normal.z);
	printf(" 3D circle: turn = %d \n",turn);

	if(-1 == pm3DCircleInit(&circle, start_xyz,middle_xyz, end_xyz, center, normal, turn))
  	{   
  			printf("failed to add pm 3D circle\n");
    		return -1;
  	} 
    pmLineInit(&line_uvw, start_uvw, end_uvw);
    pmLineInit(&line_abc, start_abc, end_abc);
//diagnostics("line_abc.tmag = %d \n",(int)(1000*line_abc.tmag));

    tc.cycle_time = tp->cycleTime;
    tc.accScale=tp->accScale;
    tc.progress = 0.0;

    tc.id = tp->nextId;
       for (i=0;i<MAX_FILENAME_LENGTH;i++)
        tc.fileName[i]= tp->fileName[i];
    tc.active = 0;
    tc.PL=tp->PL;
    tc.stopcondition=tp->stopcondition;
    tc.ioaddress=tp->ioaddress;
//diagnostics("tc.target = %d \n",(int)(1000*tc.target));
    tc.feed_override = 1.0;
    tc.free=0;
    tc.joint_type=2;
	tc.Vector = Vector;
 //   tc.reqvel = tp->vMax; 
 //   tc.maxaccel = tp->aMax*tp->accScale;
 //   tc.maxvel = tp->vLimit;


//    if (!line_abc.tmag_zero) {
	//	if (line_abc.tmag * tc.vMax > tc.target * tc.wMax)
		//	tc.reqvel = tc.target * tc.wMax / line_abc.tmag;
//diagnostics("tc.reqvel = %d \n",(int)(1000*tc.reqvel));

	//	if (tc.wMax * tc.aMax > tc.vMax * tc.wDotMax)
		//	tc.maxaccel = tc.vMax * tc.wDotMax / tc.wMax;
//diagnostics("tc.maxaccel = %d \n",(int)(1000*tc.maxaccel));

//}
	
	if (!circle.tmag_zero){ 
		if(line_abc.tmag*tc.vMax >circle.tmag*tc.wMax)
		{
		tc.reqvel =  tp->wMax;
		tc.maxaccel = tp->wDotMax;
		tc.maxvel= tp->wLimit;
	        tc.target = line_abc.tmag;
		}
		else
		{
		tc.target = circle.tmag;
		tc.reqvel = tp->vMax;
		tc.maxaccel = tp->aMax;
		tc.maxvel = tp->vLimit;
		}
   }else if (!line_uvw.tmag_zero){
		tc.target = line_uvw.tmag;
   }else{
		tc.reqvel =  tp->wMax;
		tc.maxaccel = tp->wDotMax;
		tc.maxvel= tp->wLimit;
		tc.target = line_abc.tmag;
   }


    tc.coords.circle3D.xyz = circle;
    tc.coords.circle3D.uvw = line_uvw;
    tc.coords.circle3D.abc = line_abc;

    tc.motion_type = TC_3DCIRCULAR;
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;

    tc.synchronized = tp->synchronized;
    tc.goalFrame.rot=Rtmp;

	if (-1 == tcqPut(&tp->queue, tc)) 
	{
		diagnostics("error in putcircle\n");
	    	return -1;
	}


	tp->goalJPos = end3;
	tp->goalPos.tran=end_xyz.tran;
//	tp->goalPos.a=end_abc.tran.x;
//	tp->goalPos.b=end_abc.tran.y;
//	tp->goalPos.c=end_abc.tran.z;

	tp->done = 0;
	tp->depth = tcqLen(&tp->queue);
	tp->nextId++;

	return 0;
}

int tpAddCircle(TP_STRUCT * tp, RobotPose end,
		PmCartesian center, PmCartesian normal, int turn, int type,
                double vel, double ini_maxvel, double acc, unsigned char enables, int *active_g_codes)
{
    TC_STRUCT tc;
    PmCircle circle;
    PmLine line_uvw, line_abc;
    PmPose start_xyz, end_xyz;
    PmPose start_uvw, end_uvw;
    PmPose start_abc, end_abc;
    double helix_z_component;   // z of the helix's cylindrical coord system
    double helix_length;
    PmQuaternion identity_quat = { 1.0, 0.0, 0.0, 0.0 };
	
    if (!tp || tp->aborting) 
	return -1;

    start_xyz.tran = tp->goalPos.tran;
    end_xyz.tran = end.tran;

    start_abc.tran.x = tp->goalPos.a;
    start_abc.tran.y = tp->goalPos.b;
    start_abc.tran.z = tp->goalPos.c;
    end_abc.tran.x = end.a;
    end_abc.tran.y = end.b;
    end_abc.tran.z = end.c;

    start_uvw.tran.x = tp->goalPos.u;
    start_uvw.tran.y = tp->goalPos.v;
    start_uvw.tran.z = tp->goalPos.w;
    end_uvw.tran.x = end.u;
    end_uvw.tran.y = end.v;
    end_uvw.tran.z = end.w;

    start_xyz.rot = identity_quat;
    end_xyz.rot = identity_quat;
    start_uvw.rot = identity_quat;
    end_uvw.rot = identity_quat;
    start_abc.rot = identity_quat;
    end_abc.rot = identity_quat;

    pmCircleInit(&circle, start_xyz, end_xyz, center, normal, turn);
    pmLineInit(&line_uvw, start_uvw, end_uvw);
    pmLineInit(&line_abc, start_abc, end_abc);

    // find helix length
    pmCartMag(circle.rHelix, &helix_z_component);
    helix_length = pmSqrt(pmSq(circle.angle * circle.radius) +
                          pmSq(helix_z_component));

    tc.cycle_time = tp->cycleTime;
    tc.target = helix_length;
    tc.progress = 0.0;
    tc.reqvel = vel;
    tc.maxaccel = acc;
    tc.maxAccel = acc;           // maxAccel must be initialized
    tc.maxvel = ini_maxvel;
    tc.id = tp->nextId;
    tc.active = 0;

    tc.coords.circle.xyz = circle;
    tc.coords.circle.uvw = line_uvw;
    tc.coords.circle.abc = line_abc;
    tc.motion_type = TC_CIRCULAR;
    tc.canon_motion_type = type;
    tc.blend_with_next = tp->termCond == TC_TERM_COND_BLEND;
    tc.tolerance = tp->tolerance;

    tc.synchronized = tp->synchronized;
    tc.enables = enables;

    if (tcqPut(&tp->queue, tc) == -1) {
	return -1;
    }

    tp->goalPos = end;
    tp->done = 0;
    tp->depth = tcqLen(&tp->queue);
    tp->nextId++;

    return 0;
}

void tcRunCycle(TC_STRUCT *tc, double *v, int *on_final_decel) {
    double discr, maxnewvel, newvel, newaccel=0;
    if(!tc->blending) tc->vel_at_blend_start = tc->currentvel;
    discr = 0.5 * tc->cycle_time * tc->currentvel - (tc->target - tc->progress);
    if(discr > 0.0) {
        // should never happen: means we've overshot the target
		newvel = maxnewvel = 0.0;
    } else {
        discr = 0.25 * pmSq(tc->cycle_time) - 2.0 / tc->maxaccel * discr;
        newvel = maxnewvel = -0.5 * tc->maxaccel * tc->cycle_time + 
            tc->maxaccel * pmSqrt(discr);
 //    diagnostics("bbbbbbbbb1 = %d ,%d,%d,\n",(int)(1000000* tc->maxaccel),(int)(1000000000*discr),(int)(1000000*tc->cycle_time));
    }
    if(newvel<=0.0 ) {
        // also should never happen - if we already finished this tc, it was
        // caught above
        newvel = newaccel = 0.0;
        tc->progress = tc->target;
    } else {
        // constrain velocity
//	 diagnostics(" tc->feed_override =%d\n",(int)( tc->feed_override*1000));

        if(newvel > tc->reqvel * tc->feed_override) 
            newvel = tc->reqvel * tc->feed_override;
        if(newvel > tc->maxvel) newvel = tc->maxvel;
        // get resulting acceleration
        newaccel = (newvel - tc->currentvel) / tc->cycle_time;
        // constrain acceleration and get resulting velocity
        if(newaccel > 0.0 && newaccel > tc->maxaccel){
            newaccel = tc->maxaccel;
            newvel = tc->currentvel + newaccel * tc->cycle_time;
        }
        if(newaccel < 0.0 && newaccel < -tc->maxaccel){
            newaccel = -tc->maxaccel;
            newvel = tc->currentvel + newaccel * tc->cycle_time;
	}
        tc->progress += (newvel + tc->currentvel) * 0.5 * tc->cycle_time;
//     diagnostics("tcrun = %d ,%d,%d,%d,%d ,%d\n",(int)(1000000*tc->currentvel),(int)(1000000*newvel),(int)(1000000*tc->cycle_time),(int)(1000*newaccel),(int)(1000*tc->maxaccel),(int)(1000*tc->feed_override));
 //    diagnostics("tc->progress = %d ,%d,  \n",(int)(1000* tc->progress),(int)(1000*tc->target));
	}
    tc->currentvel = newvel;
    if(v) *v = newvel;
    if(on_final_decel) *on_final_decel = fabs(maxnewvel - newvel) < 0.001;
}

// This is the brains of the operation.  It's called every TRAJ period
// and is expected to set tp->currentPos to the new machine position.
// Lots of other tp fields (depth, done, etc) have to be twiddled to
// communicate the status; I think those are spelled out here correctly
// and I can't clean it up without breaking the API that the TP presents
// to motion.  It's not THAT bad and in the interest of not touching
// stuff outside this directory, I'm going to leave it for now.

int tpRunCycle(TP_STRUCT * tp, long period)
{
    TC_STRUCT *tc, *nexttc;
    double primary_vel;
    int on_final_decel;
    PmJoint primary_before1 = {0.0,0.0,0.0,0.0,0.0,0.0},primary_after1= {0.0,0.0,0.0,0.0,0.0,0.0};
    RobotPose primary_before2, primary_after2 = {{0.0,0.0,0.0},0.0,0.0,0.0,0.0,0.0,0.0};
    PmJoint secondary_before1 = {0.0,0.0,0.0,0.0,0.0,0.0}, secondary_after1 ={0.0,0.0,0.0,0.0,0.0,0.0};
    RobotPose secondary_before2, secondary_after2;
    RobotPose primary_displacement, secondary_displacement;
    PmJoint primary_disJ = {0.0,0.0,0.0,0.0,0.0,0.0}, secondary_disJ = {0.0,0.0,0.0,0.0,0.0,0.0};
    double save_vel;
    int i;
	static int lastPaused ;
	if (emcmotStatus->joint_type!=2)
	{
		i = 0;
	}
 	tc = tcqItem(&tp->queue, 0, period);
	//printf("tc=%d",tc);
    	if(!tc) {
		tcqInit(&tp->queue);
		//       tp->goalPos = tp->currentPos;
		//       tp->goalJPos=tp->currentJPos;
	//    if(emcmotStatus->joint_type == 2){
	//          emcmotStatus->joint_type = 1;
	//     }

		tp->done = 1;
		tp->depth = tp->activeDepth = 0;
		tp->aborting = 0;
//		tp->execId = 0;
		for (i=0;i<MAX_FILENAME_LENGTH;i++)
		tp->execFileName[i]= '\0';
		tp->motionType = 0;
		tpResume(tp);
		emcmotStatus->distance_to_go = 0.0;    // 2010.10.22
		tp->currentJPos = emcmotStatus->joint_cmd;
		tp->goalJPos = emcmotStatus->joint_cmd;
		tp->currentPos = emcmotStatus->carte_pos_cmd;
		tp->goalPos = emcmotStatus->carte_pos_cmd;
		tp->currentFrame.rot= emcmotDebug->currentFrame.rot;
		tp->goalFrame.rot= emcmotDebug->currentFrame.rot;
		emcmotStatus->joint_type=0;

// diagnostics("tpAddLine end joint %d,%d,%d,%d,%d,%d\n",
 //		(int)(1000*emcmotStatus->joint_cmd.j0),(int)(1000*emcmotStatus->joint_cmd.j1),(int)(1000*emcmotStatus->joint_cmd.j2),
 //		(int)(1000*emcmotStatus->joint_cmd.j3),(int)(1000*emcmotStatus->joint_cmd.j4),(int)(1000*emcmotStatus->joint_cmd.j5));

		tp->decelFlag = 0;
		return 0;
	}
	tp->accScale = tc->accScale;
	emcmotStatus->joint_type = tc->joint_type;
	tp->Vector = tc->Vector;
	
if(tc->joint_type==2){
	tp->robot_R0 = tc->robot_R0;

}
	tp->free = tc->free;
	tp->cord_type = tc->cord_type; 
	tp->robot_tcp = tc->robot_tcp;
	tp->robot_tcf  = tc->robot_tcf ;
	tp->robot_invtcf = tc->robot_invtcf;
	tp->robot_RT= tc->robot_RT;
	tp->usr=tc->usr;
	tp->tool=tc->tool;
    if (fabs(tc->target -tc->progress)<CART_FUZZ) {

        tcqRemove(&tp->queue, 1);
        tp->depth = tcqLen(&tp->queue);
        tc = tcqItem(&tp->queue, 0, period);
        if(!tc) {
		tcqInit(&tp->queue);
		//       tp->goalPos = tp->currentPos;
		//       tp->goalJPos=tp->currentJPos;
	//    if(emcmotStatus->joint_type == 2){
	//          emcmotStatus->joint_type = 1;
	//     }
		
		tp->done = 1;
		tp->depth = tp->activeDepth = 0;
		tp->aborting = 0;
//		tp->execId = 0;
		for (i=0;i<MAX_FILENAME_LENGTH;i++)
		tp->execFileName[i]= '\0';
		tp->motionType = 0;
		tpResume(tp);
		emcmotStatus->distance_to_go = 0.0;    // 2010.10.22
		tp->currentJPos = emcmotStatus->joint_cmd;
		tp->goalJPos = emcmotStatus->joint_cmd;
		tp->currentPos = emcmotStatus->carte_pos_cmd;
		tp->goalPos = emcmotStatus->carte_pos_cmd;
		tp->currentFrame.rot= emcmotDebug->currentFrame.rot;
		tp->goalFrame.rot= emcmotDebug->currentFrame.rot;
		emcmotStatus->joint_type=0;

// diagnostics("tpAddLine end joint %d,%d,%d,%d,%d,%d\n",
 //		(int)(1000*emcmotStatus->joint_cmd.j0),(int)(1000*emcmotStatus->joint_cmd.j1),(int)(1000*emcmotStatus->joint_cmd.j2),
 //		(int)(1000*emcmotStatus->joint_cmd.j3),(int)(1000*emcmotStatus->joint_cmd.j4),(int)(1000*emcmotStatus->joint_cmd.j5));

		tp->decelFlag = 0;
		return 0;
	}
		tp->free = tc->free;
		tp->cord_type = tc->cord_type;
		tp->robot_tcp = tc->robot_tcp;
		tp->robot_tcf = tc->robot_tcf;
		tp->robot_invtcf = tc->robot_invtcf;
		tp->robot_RT = tc->robot_RT;
		tp->usr = tc->usr;
		tp->tool = tc->tool;
	  }
/*
	if((NONE !=tc->iotype)&&(extIORead2(tc->addrtype,tc->iotype,tc->ioaddress)==tc->stopcondition)){
	emcmotDebug->adjust = 1;
	tc->feed_override = 0.0;
	if((fabs(tc->currentvel )<CART_FUZZ)){
	emcmotStatus->distance_to_go = 0.0;    // 2010.10.22
	emcmotStatus->distance_was_gone = 0.0;
	tcqRemove(&tp->queue, 1);
	tp->depth = tcqLen(&tp->queue);
	tc = tcqItem(&tp->queue, 0, period);
	if(!tc) {
		tcqInit(&tp->queue);
		//       tp->goalPos = tp->currentPos;
		//       tp->goalJPos=tp->currentJPos;
	//    if(emcmotStatus->joint_type == 2){
	//          emcmotStatus->joint_type = 1;
	//     }

		tp->done = 1;
		tp->depth = tp->activeDepth = 0;
		tp->aborting = 0;
//		tp->execId = 0;
		for (i=0;i<MAX_FILENAME_LENGTH;i++)
		tp->execFileName[i]= '\0';
		tp->motionType = 0;
		tpResume(tp);
		emcmotStatus->distance_to_go = 0.0;    // 2010.10.22
		tp->currentJPos = emcmotStatus->joint_cmd;
		tp->goalJPos = emcmotStatus->joint_cmd;
		tp->currentPos = emcmotStatus->carte_pos_cmd;
		tp->goalPos = emcmotStatus->carte_pos_cmd;
		tp->currentFrame.rot= emcmotDebug->currentFrame.rot;
		tp->goalFrame.rot= emcmotDebug->currentFrame.rot;
		emcmotStatus->joint_type=0;

// diagnostics("tpAddLine end joint %d,%d,%d,%d,%d,%d\n",
 //		(int)(1000*emcmotStatus->joint_cmd.j0),(int)(1000*emcmotStatus->joint_cmd.j1),(int)(1000*emcmotStatus->joint_cmd.j2),
 //		(int)(1000*emcmotStatus->joint_cmd.j3),(int)(1000*emcmotStatus->joint_cmd.j4),(int)(1000*emcmotStatus->joint_cmd.j5));

		tp->decelFlag = 0;
		return 0;
	}
	if(tc->joint_type==1){
	        if(-1 ==tpAdjustJoints(tc)){
		return -1;
	        }
	      emcmotDebug->adjust=0;
	    }else{ 
	        if(-1==tpAdjustLine(tc)){
		return -1;
	        }
	        emcmotDebug->adjust=0;
	    }
	   return 0;
	}
    }
	
	    tc->feed_override = 1.0;
	    if((!emcmotStatus->rapidMode)&&tc->free){
	        tc->feed_override = emcmotDebug->net_feed_scale[tc->joint_num]/100.0;
	    }else{
	        tc->feed_override = emcmotDebug->net_feed_scale[0]/100.0;
	} 
	*/


  if(tc->blend_with_next) {
        nexttc = tcqItem(&tp->queue, 1, period);
	 if (!nexttc || nexttc->joint_type != tc->joint_type||emcmotDebug->adjust == 1)
	 	nexttc = NULL;
  }else {
        nexttc = NULL;
  }
    if(tc->active == 0) {
        tc->currentvel = 0.0;
        tp->activeDepth = 1;
        tp->motionType = tc->canon_motion_type;
        tc->active = 1;
        tc->blending = 0;
//	if(tc->maxvel > tp->vLimit) 
//           tc->maxvel = tp->vLimit;
        if(tc->blend_with_next) 
            tc->maxaccel /= 2.0;
    }
//	 diagnostics("tprun 2 tc->joint_type = %d abort=%d\n",(int)(tc->joint_type),tp->aborting);
    if(nexttc && nexttc->active == 0) {
        nexttc->currentvel = 0.0;
	 tp->activeDepth = 2;                                                  
        nexttc->active = 1;
        nexttc->blending = 0;
        if(nexttc->maxvel > tp->vLimit) 
            nexttc->maxvel = tp->vLimit;
        if(tc->blend_with_next || nexttc->blend_with_next)
            nexttc->maxaccel /= 2.0;
    }

    if(tp->aborting) {
        if(((fabs(tc->currentvel )<CART_FUZZ) && !nexttc) ||
            ((fabs(tc->currentvel )<CART_FUZZ) && nexttc && (fabs(nexttc->currentvel )<CART_FUZZ))) {
            tcqInit(&tp->queue);
//	    if(emcmotStatus->joint_type == 2){
//	          emcmotStatus->joint_type = 1;
//	     }
  	//tc->maxaccel=tp->jaccmax;
  	
	tp->done = 1;
	tp->depth = tp->activeDepth = 0;
	tp->aborting = 0;
//	tp->execId = 0;
        for (i=0;i<MAX_FILENAME_LENGTH;i++)
            tp->execFileName[i]= '\0';
            tp->motionType = 0;
            tp->synchronized = 0;
            emcmotStatus->distance_to_go = 0.0;    // 2010.10.22
            emcmotStatus->distance_was_gone = 0.0;
//	tpClear(tp);
		tp->currentJPos = emcmotStatus->joint_cmd;
		tp->goalJPos = emcmotStatus->joint_cmd;
		tp->currentPos = emcmotStatus->carte_pos_cmd;
		tp->goalPos = emcmotStatus->carte_pos_cmd;
		tp->currentFrame.rot= emcmotDebug->currentFrame.rot;
		tp->goalFrame.rot= emcmotDebug->currentFrame.rot;
		emcmotStatus->joint_type=0;

	tpResume(tp);
            return 0;
        } else {
            tc->reqvel = 0.0;
            tc->maxaccel=tc->maxaccel/tp->accScale;
            if(nexttc) nexttc->reqvel = 0.0;
			
	}
    }
//	 diagnostics("tprun 3 tc->joint_type = %d \n",(int)(tc->joint_type));

    if(nexttc && nexttc->maxaccel > 0.0) {
        tc->blend_vel = nexttc->maxaccel * 
            pmSqrt(nexttc->target / nexttc->maxaccel);
        if(tc->blend_vel > nexttc->reqvel * nexttc->feed_override) {
            tc->blend_vel = nexttc->reqvel * nexttc->feed_override;
        }
        if(tc->maxaccel < nexttc->maxaccel)
            tc->blend_vel *= tc->maxaccel/nexttc->maxaccel;
    }

  if((tp->pausing == 1)&&(lastPaused == 0)){
  	printf("entering pausing \n");
	tc->saved_feed_override = tc->feed_override;
	tc->feed_override = 0.0;
	if(nexttc) {
	nexttc->saved_feed_override = nexttc->feed_override;
	nexttc->feed_override = 0.0;
	}
	
  }
  else if((tp->pausing == 0)&&(lastPaused == 1))
  {
  	printf("entering resume\n");
  	tc->feed_override = tc->saved_feed_override;
	if(nexttc) {
	nexttc->feed_override = nexttc->saved_feed_override;
	}
	
  }
  lastPaused = tp->pausing;

    // calculate the approximate peak velocity the nexttc will hit.
    // we know to start blending it in when the current tc goes below
    // this velocity...
    if(1 == tc->joint_type ){
	emcmotStatus->joint_type = 1;
	primary_before1 = tcGetJPos(tc);
        tcRunCycle(tc, &primary_vel, &on_final_decel);
        primary_after1 = tcGetJPos(tc);

	primary_disJ.j0 = primary_after1.j0 - primary_before1.j0;
	primary_disJ.j1 = primary_after1.j1 - primary_before1.j1;
	primary_disJ.j2 = primary_after1.j2 - primary_before1.j2;
	primary_disJ.j3 = primary_after1.j3 - primary_before1.j3;
	primary_disJ.j4 = primary_after1.j4 - primary_before1.j4;
	primary_disJ.j5 = primary_after1.j5 - primary_before1.j5;
	primary_disJ.j6 = primary_after1.j6 - primary_before1.j6;
	primary_disJ.j7 = primary_after1.j7 - primary_before1.j7;


	
	}else{
	emcmotStatus->joint_type = 2;
	tp->robot_R0 = tc->robot_R0;
	tp->free = tc->free;
	tp->cord_type = tc->cord_type; 
	tp->robot_tcp = tc->robot_tcp;
	tp->robot_tcf  = tc->robot_tcf ;
	tp->robot_invtcf = tc->robot_invtcf;
	tp->robot_RT= tc->robot_RT;
	
        primary_before2 = tcGetPos(tc);
	//printf("primary_before2 x =%f, y=%f ,z=%f \n",primary_before2.tran.x,primary_before2.tran.y,primary_before2.tran.z);
	//printf("maxaccel=%f cycle_time=%f \n",tc->maxaccel,tc->cycle_time);

		tcRunCycle(tc, &primary_vel, &on_final_decel);
        primary_after2 = tcGetPos(tc);
	//printf("primary_after2 x =%f, y=%f ,z=%f \n",primary_after2.tran.x,primary_after2.tran.y,primary_after2.tran.z);
	
//	 diagnostics(" primary_after2 = %d,%d,%d \n",(int)(1000*primary_after2.tran.x),(int)(1000*primary_after2.tran.y),(int)(1000*primary_after2.tran.z));
//	 diagnostics(" abc,primary_after2 = %d,%d,%d \n",(int)(1000*primary_after2.a),(int)(1000*primary_after2.b),(int)(1000*primary_after2.c));

	    pmCartCartSub(primary_after2.tran, primary_before2.tran, 
	            &primary_displacement.tran);
	    primary_displacement.a = primary_after2.a - primary_before2.a;
	    primary_displacement.b = primary_after2.b - primary_before2.b;
	    primary_displacement.c = primary_after2.c - primary_before2.c;

	    primary_displacement.u = primary_after2.u - primary_before2.u;
	    primary_displacement.v = primary_after2.v - primary_before2.v;
	    primary_displacement.w = primary_after2.w - primary_before2.w;
	} 
//	 diagnostics("tprun 4 tc->joint_type = %d \n",(int)(tc->joint_type));


   if((tc->blending && nexttc) || 
            (nexttc && on_final_decel && primary_vel < tc->blend_vel))
   {
        // make sure we continue to blend this segment even when its 
        // accel reaches 0 (at the very end)
        tc->blending = 1;

        // hack to show blends in axis
        // tp->motionType = 0;

        if(tc->currentvel > nexttc->currentvel) {
		tp->motionType = tc->canon_motion_type;
		tp->vMax = tc->reqvel;                                      // 2010.08.26
		tp->ioaddress= tc->ioaddress;
		tp->stopcondition= tc->stopcondition;
		tp->iotype= tc->iotype;
		tp->addrtype= tc->addrtype;
		tp->currentVel = tc->currentvel;                                      // 2010.08.26
		emcmotStatus->distance_to_go = tc->target - tc->progress;
		emcmotStatus->distance_was_gone = tc->progress;
		//  emcmotStatus->enables_queued = tc->enables;
		// report our line number to the guis
		tp->execId = tc->id;
		//  tp->execFileId = tc->fileId;
		for (i=0;i<MAX_FILENAME_LENGTH;i++)
		tp->execFileName[i]= tc->fileName[i];
        } else {
		tp->ioaddress= nexttc->ioaddress;
		tp->stopcondition= nexttc->stopcondition;
		tp->iotype= nexttc->iotype;
		tp->addrtype= nexttc->addrtype;
		tp->currentVel = nexttc->currentvel;                                      // 2010.08.26
		tp->motionType = nexttc->canon_motion_type;
		tp->vMax = nexttc->reqvel;                                      // 2010.08.26
		emcmotStatus->distance_to_go = nexttc->target - nexttc->progress;
		emcmotStatus->distance_was_gone = nexttc->progress;
		//   emcmotStatus->enables_queued = nexttc->enables;
		// report our line number to the guis
		tp->execId = nexttc->id;
		//   tp->execFileId = nexttc->fileId;
		for (i=0;i<MAX_FILENAME_LENGTH;i++)
		tp->execFileName[i]= tc->fileName[i];
        }
	
	tp->decelFlag = on_final_decel;
      //  emcmotStatus->current_vel = tc->currentvel + nexttc->currentvel;

	if(1 == tc->joint_type) {
        secondary_before1 = tcGetJPos(nexttc);
	}
	else {
        secondary_before2 = tcGetPos(nexttc);
	}
        save_vel = nexttc->reqvel;
        nexttc->reqvel = nexttc->feed_override > 0.0 ? 
            ((tc->vel_at_blend_start - primary_vel) / nexttc->feed_override) :
            0.0;
        tcRunCycle(nexttc, NULL, NULL);
        nexttc->reqvel = save_vel;

	if(1 == tc->joint_type) {
		secondary_after1 = tcGetJPos(nexttc);
		secondary_disJ.j0 = secondary_after1.j0 - secondary_before1.j0;
		secondary_disJ.j1 = secondary_after1.j1 - secondary_before1.j1;
		secondary_disJ.j2 = secondary_after1.j2 - secondary_before1.j2;
		secondary_disJ.j3 = secondary_after1.j3 - secondary_before1.j3;
		secondary_disJ.j4 = secondary_after1.j4 - secondary_before1.j4;
		secondary_disJ.j5 = secondary_after1.j5 - secondary_before1.j5;
		secondary_disJ.j6 = secondary_after1.j6 - secondary_before1.j6;
		secondary_disJ.j7 = secondary_after1.j7 - secondary_before1.j7;

		tp->currentJPos.j0 += primary_disJ.j0 + secondary_disJ.j0;
		tp->currentJPos.j1 += primary_disJ.j1 + secondary_disJ.j1;
		tp->currentJPos.j2 += primary_disJ.j2 + secondary_disJ.j2;
		tp->currentJPos.j3 += primary_disJ.j3 + secondary_disJ.j3;
		tp->currentJPos.j4 += primary_disJ.j4 + secondary_disJ.j4;
		tp->currentJPos.j5 += primary_disJ.j5 + secondary_disJ.j5;
		tp->currentJPos.j6 += primary_disJ.j6 + secondary_disJ.j6;
		tp->currentJPos.j7 += primary_disJ.j7 + secondary_disJ.j7;

	}
	else {
	        secondary_after2 = tcGetPos(nexttc);
	        pmCartCartSub(secondary_after2.tran, secondary_before2.tran, 
	                &secondary_displacement.tran);
	        secondary_displacement.a = secondary_after2.a - secondary_before2.a;
	        secondary_displacement.b = secondary_after2.b - secondary_before2.b;
	        secondary_displacement.c = secondary_after2.c - secondary_before2.c;

	        secondary_displacement.u = secondary_after2.u - secondary_before2.u;
	        secondary_displacement.v = secondary_after2.v - secondary_before2.v;
	        secondary_displacement.w = secondary_after2.w - secondary_before2.w;

	        pmCartCartAdd(tp->currentPos.tran, primary_displacement.tran, 
	                &tp->currentPos.tran);
	        pmCartCartAdd(tp->currentPos.tran, secondary_displacement.tran, 
	                &tp->currentPos.tran);
	        tp->currentPos.a += primary_displacement.a + secondary_displacement.a;
	        tp->currentPos.b += primary_displacement.b + secondary_displacement.b;
	        tp->currentPos.c += primary_displacement.c + secondary_displacement.c;

	        tp->currentPos.u += primary_displacement.u + secondary_displacement.u;
	        tp->currentPos.v += primary_displacement.v + secondary_displacement.v;
	        tp->currentPos.w += primary_displacement.w + secondary_displacement.w;
	}

   }
   else{
    tp->motionType = tc->canon_motion_type;
    tp->ioaddress= tc->ioaddress;
    tp->stopcondition= tc->stopcondition;
    tp->iotype= tc->iotype;
    tp->addrtype= tc->addrtype;
    tp->currentVel = tc->currentvel;                                      // 2010.08.26
    tp->vMax = tc->reqvel;                                      // 2010.08.26
    emcmotStatus->distance_to_go = tc->target - tc->progress;
    emcmotStatus->distance_was_gone = tc->progress;
    if(1 == tc->joint_type){ 
        tp->currentJPos=primary_after1;}
    else {
        tp->currentPos = primary_after2;}

        // report our line number to the guis

        tp->execId = tc->id;

	for (i=0;i<MAX_FILENAME_LENGTH;i++)
        tp->execFileName[i]= tc->fileName[i];
   }
    return 0;
}


int tpPause(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }
    tp->pausing = 1;
    return 0;
}

int tpResume(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }
    tp->pausing = 0;
    return 0;
}

int tpAbort(TP_STRUCT * tp)
{
    if (0 == tp) {
	return -1;
    }

    if (!tp->aborting) {
	// to abort, signal a pause and set our abort flag 
	//tpPause(tp);

  // if(emcmotStatus->joint_type==2){
   //       emcmotStatus->joint_type=1;
   //  }
     
	tp->aborting = 1;
//	memset(io_output_shmem, 0, 8*emcmotConfig->io_output_num);
//	memset(io_input_shmem, 0, 8*emcmotConfig->io_input_num);

    }
    return 0;
}

int tpGetMotionType(TP_STRUCT * tp)
{
    return tp->motionType;
}

RobotPose tpGetPos(TP_STRUCT * tp)
{
    RobotPose retval;
    if (0 == tp) {
	retval.tran.x = retval.tran.y = retval.tran.z = 0.0;
	retval.a = retval.b = retval.c = 0.0;
	retval.u = retval.v = retval.w = 0.0;
	return retval;
    }
	//printf("##########tp GetPos########################%f  %f  %f %f  %f  %f\r\n",tp->currentPos.tran.x, tp->currentPos.tran.y, tp->currentPos.tran.z, tp->currentPos.a, tp->currentPos.b, tp->currentPos.c);
    return tp->currentPos;
}
double tpGetCurVel(TP_STRUCT * tp)
{

    if (0 == tp) {

	return -1;
    }

    return tp->currentVel;
}
PmJoint tpGetJPos(TP_STRUCT * tp)
{
    PmJoint retval;

    if (0 == tp) {
	retval.j0 = retval.j1 = retval.j2=retval.j3=retval.j4=retval.j5= retval.j6=retval.j7= 0.0;
	return retval;
    }

    return tp->currentJPos;
}
int tpIsDone(TP_STRUCT * tp)
{
    if (0 == tp) {
	return 0;
    }

    return tp->done;
}



int tpQueueDepth(TP_STRUCT * tp)
{
    if (0 == tp) {
	return 0;
    }

    return tp->depth;
}

int tpActiveDepth(TP_STRUCT * tp)
{
    if (0 == tp) {
	return 0;
    }

    return tp->activeDepth;
}



int tpSetAout(TP_STRUCT *tp, unsigned char index, double start, double end) {
    return 0;
}

int tpSetDout(TP_STRUCT *tp, int index, unsigned char start, unsigned char end) {
    return 0;
}



// vim:sw=4:sts=4:et:
