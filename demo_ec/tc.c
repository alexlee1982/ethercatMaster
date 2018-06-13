 

#include <stdio.h>


#include "posemath.h"
#include "robotpos.h"
#include "tc.h"

PmCartesian tcGetStartingUnitVector(TC_STRUCT *tc) {
    PmCartesian v;

    if(tc->motion_type == TC_LINEAR || tc->motion_type == TC_RIGIDTAP) {
        pmCartCartSub(tc->coords.line.xyz.end.tran, tc->coords.line.xyz.start.tran, &v);
    } else {
        PmPose startpoint;
        PmCartesian radius;

        pmCirclePoint(&tc->coords.circle.xyz, 0.0, &startpoint);
        pmCartCartSub(startpoint.tran, tc->coords.circle.xyz.center, &radius);
        pmCartCartCross(tc->coords.circle.xyz.normal, radius, &v);
    }
    pmCartUnit(v, &v);
    return v;
}

PmCartesian tcGetEndingUnitVector(TC_STRUCT *tc) {
    PmCartesian v;

    if(tc->motion_type == TC_LINEAR) {
        pmCartCartSub(tc->coords.line.xyz.end.tran, tc->coords.line.xyz.start.tran, &v);
    } else if(tc->motion_type == TC_RIGIDTAP) {
        // comes out the other way
        pmCartCartSub(tc->coords.line.xyz.start.tran, tc->coords.line.xyz.end.tran, &v);
    } else {
        PmPose endpoint;
        PmCartesian radius;

        pmCirclePoint(&tc->coords.circle.xyz, tc->coords.circle.xyz.angle, &endpoint);
        pmCartCartSub(endpoint.tran, tc->coords.circle.xyz.center, &radius);
        pmCartCartCross(tc->coords.circle.xyz.normal, radius, &v);
    }
    pmCartUnit(v, &v);
    return v;
}
 
RobotPose tcGetPos(TC_STRUCT * tc)
{
	RobotPose pos;
	PmPose xyz;
	PmPose abc;
	PmPose uvw;
	static int count;
	static double value,value_old;
	PmCartesian  vector;
	double aaaa;
    if (tc->motion_type == TC_RIGIDTAP) {
        if(tc->coords.rigidtap.state > REVERSING) {
            pmLinePoint(&tc->coords.rigidtap.aux_xyz, tc->progress, &xyz);
        } else {
            pmLinePoint(&tc->coords.rigidtap.xyz, tc->progress, &xyz);
        }
        // no rotary move allowed while tapping
        abc.tran = tc->coords.rigidtap.abc;
        uvw.tran = tc->coords.rigidtap.uvw;
    } else if (tc->motion_type == TC_LINEAR) {
        if (tc->coords.line.xyz.tmag > 0.) {
            // progress is along xyz, so uvw and abc move proportionally in order
            // to end at the same time.
	if(tc->coords.line.abc.tmag*tc->vMax > tc->coords.line.xyz.tmag*tc->wMax){
            pmLinePoint(&tc->coords.line.abc, tc->progress, &abc);
            pmLinePoint(&tc->coords.line.xyz, tc->progress * tc->coords.line.xyz.tmag / tc->target, &xyz);
            pmLinePoint(&tc->coords.line.uvw, 0.0, &uvw);
	}else{
            pmLinePoint(&tc->coords.line.xyz, tc->progress, &xyz);
            pmLinePoint(&tc->coords.line.uvw,
                        tc->progress * tc->coords.line.uvw.tmag / tc->target,
                        &uvw);
            pmLinePoint(&tc->coords.line.abc,
                        tc->progress * tc->coords.line.abc.tmag / tc->target,
                        &abc);

		}
//rtapi_print("1111111tcgetpos= %d ,%d,%d  \n",(int)(1000*tc->progress),(int)(1000*tc->target),(int)(1000*tc->coords.line.abc.tmag));
//rtapi_print("abc,tcgetpos= %d ,%d,%d  \n",(int)(1000*abc.tran.x),(int)(1000*abc.tran.y),(int)(1000*abc.tran.z));
        } else {
            // if all else fails, it's along abc only

            pmLinePoint(&tc->coords.line.xyz, 0.0, &xyz);
            pmLinePoint(&tc->coords.line.uvw, 0.0, &uvw);
            pmLinePoint(&tc->coords.line.abc, tc->progress, &abc);
        }
    }else if(tc->motion_type == TC_3DCIRCULAR) {

  if (tc->coords.circle3D.xyz.tmag > 0.) {
      if(tc->coords.circle3D.abc.tmag*tc->vMax > tc->coords.circle3D.xyz.tmag*tc->wMax){

count++;
	     pmLinePoint(&tc->coords.circle3D.abc, tc->progress, &abc);
            pm3DCirclePoint(&tc->coords.circle3D.xyz, tc->progress * tc->coords.circle3D.xyz.angle / tc->target, &xyz);
            pmLinePoint(&tc->coords.circle3D.uvw, 0.0, &uvw);
// rtapi_print("xyz,tcgetpos= %d ,%d,%d \n",(int)(1000*xyz.tran.x),(int)(1000*xyz.tran.y),(int)(1000*xyz.tran.z));
// rtapi_print("abc,tcgetpos= %d ,%d,%d,\n",(int)(1000*abc.tran.x),(int)(1000*abc.tran.y),(int)(1000*abc.tran.z));

value =pmSq(xyz.tran.x-tc->coords.circle3D.xyz.center.x)+pmSq(xyz.tran.y-tc->coords.circle3D.xyz.center.y)+pmSq(xyz.tran.z-tc->coords.circle3D.xyz.center.z);
// rtapi_print("value,tcgetpos= %d ,%d\n",(int)(value),(int)(value_old));

if((value-value_old)>100){
//	rtapi_print(" the R is not right\n");
	value_old=value;
}
 pmCartCartSub(xyz.tran,tc->coords.circle3D.xyz.center,&vector);
 pmCartCartDot(tc->coords.circle3D.xyz.normal,vector,&aaaa   );
if(count<100||0==count%100){
// rtapi_print("1111111tcgetpos= %d ,%d,%d,%d\n",(int)(1000*aaaa),(int)(1000*tc->coords.circle3D.xyz.center.x),(int)(1000*tc->coords.circle3D.xyz.center.y),(int)(1000*tc->coords.circle3D.xyz.center.z));

  	}
	    }else{

		pm3DCirclePoint(&tc->coords.circle3D.xyz,tc->progress * tc->coords.circle3D.xyz.angle / tc->target, &xyz);
		pmLinePoint(&tc->coords.circle3D.abc,
		tc->progress * tc->coords.circle3D.abc.tmag / tc->target, 
		&abc);
		// same for uvw
		pmLinePoint(&tc->coords.circle3D.uvw,
		tc->progress * tc->coords.circle3D.uvw.tmag / tc->target, 
		&uvw);
	   }

      }else{
            pm3DCirclePoint(&tc->coords.circle3D.xyz, 0.0, &xyz);
            pmLinePoint(&tc->coords.circle3D.uvw, 0.0, &uvw);
            pmLinePoint(&tc->coords.circle3D.abc, tc->progress, &abc);

      }
}else { //we have TC_CIRCULAR
        // progress is always along the xyz circle.  This simplification 
        // is possible since zero-radius arcs are not allowed by the interp.
        pmCirclePoint(&tc->coords.circle.xyz,
		      tc->progress * tc->coords.circle.xyz.angle / tc->target, 
                      &xyz);
        // abc moves proportionally in order to end at the same time as the 
        // circular xyz move.
        pmLinePoint(&tc->coords.circle.abc,
                    tc->progress * tc->coords.circle.abc.tmag / tc->target, 
                    &abc);
        // same for uvw
        pmLinePoint(&tc->coords.circle.uvw,
                    tc->progress * tc->coords.circle.uvw.tmag / tc->target, 
                    &uvw);
    }

    pos.tran = xyz.tran;
    pos.a = abc.tran.x;
    pos.b = abc.tran.y;
    pos.c = abc.tran.z;
    pos.u = uvw.tran.x;
    pos.v = uvw.tran.y;
    pos.w = uvw.tran.z;

    return pos;
}
PmJoint tcGetJPos(TC_STRUCT * tc)
{
    PmJoint pos;
  //  PmPose jpos;

    if (tc->motion_type == TC_JLINEAR) {
       pmJointsPoint(&tc->coords.jline, tc->progress, &pos);
    } 

//   pos=jpos;

   return pos;
}

int tcqCreate(TC_QUEUE_STRUCT * tcq, int _size, TC_STRUCT * tcSpace)
{
    if (_size <= 0 || 0 == tcq) {
	return -1;
    } else {
	tcq->queue = tcSpace;
	tcq->size = _size;
	tcq->_len = 0;
	tcq->start = tcq->end = 0;
	tcq->allFull = 0;

	if (0 == tcq->queue) {
	    return -1;
	}
	return 0;
    }
}
 
int tcqInit(TC_QUEUE_STRUCT * tcq)
{
    if (0 == tcq) {
	return -1;
    }

    tcq->_len = 0;
    tcq->start = tcq->end = 0;
    tcq->allFull = 0;

    return 0;
}
  
int tcqPut(TC_QUEUE_STRUCT * tcq, TC_STRUCT tc)
{
    /* check for initialized */
    if (0 == tcq || 0 == tcq->queue) {
	    return -1;
    }
    /* check for allFull, so we don't overflow the queue */
    if (tcq->allFull) {	
	  return -1;         
    }
    /* add it */

    tcq->queue[tcq->end] = tc;
    tcq->_len++;

    /* update end ptr, modulo size of queue */
    tcq->end = (tcq->end + 1) % tcq->size;

    /* set allFull flag if we're really full */
    if (tcq->end == tcq->start) {
	tcq->allFull = 1;
    }

    return 0;
}
 
int tcqRemove(TC_QUEUE_STRUCT * tcq, int n)
{

    if (n <= 0) {
	    return 0;		/* okay to remove 0 or fewer */
    }

    if ((0 == tcq) || (0 == tcq->queue) ||	/* not initialized */
	((tcq->start == tcq->end) && !tcq->allFull) ||	/* empty queue */
	(n > tcq->_len)) {	/* too many requested */
	    return -1;
    }

    /* update start ptr and reset allFull flag and len */
    tcq->start = (tcq->start + n) % tcq->size;
    tcq->allFull = 0;
    tcq->_len -= n;

    return 0;
}

 
int tcqLen(TC_QUEUE_STRUCT * tcq)
{
    if (0 == tcq) {
	    return -1;
    }

    return tcq->_len;
}
 
TC_STRUCT *tcqItem(TC_QUEUE_STRUCT * tcq, int n, long period)
{
    TC_STRUCT *t;
    if ((0 == tcq) || (0 == tcq->queue) ||	/* not initialized */
	(n < 0) || (n >= tcq->_len)) {	/* n too large */
	//printf("not initialized tcq=%d tcq->len=%d n=%d",tcq,tcq->_len,n);
	return (TC_STRUCT *) 0;
    }
    t = &(tcq->queue[(tcq->start + n) % tcq->size]);
#ifndef RTAPI
    t->cycle_time = period * 0.000000001;;
#endif
    return t;
}
 
#define TC_QUEUE_MARGIN 10
 
int tcqFull(TC_QUEUE_STRUCT * tcq)
{
    if (0 == tcq) {
	   return 1;		/* null queue is full, for safety */
    }
 
    if (tcq->size <= TC_QUEUE_MARGIN) {
 	    return tcq->allFull;
    }

    if (tcq->_len >= tcq->size - TC_QUEUE_MARGIN) {
 	    return 1;
    }
 
    return 0;
}

