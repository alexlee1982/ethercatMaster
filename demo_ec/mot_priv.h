#ifndef MOT_PRIV_H
#define MOT_PRIV_H

/***********************************************************************
*                       TYPEDEFS, ENUMS, ETC.                          *
************************************************************************/

/***********************************************************************
*                   GLOBAL VARIABLE DECLARATIONS                       *
************************************************************************/
#include "motionInterfaceBuffer.h"
#include "kinematics.h"
#include "motionConfig.h"
#include "motionFeedback.h"
#include "rtos.h"
// #include <pthread.h>

#define diagnostics printf

#ifndef NULL
#define NULL 0
#endif


extern int home_flag_modi;
extern int num_joints;
extern int num_axis;
extern MotionJointParameter *joints;
extern int first_pass;
extern int DEBUG_MOTION;
extern KINEMATICS_FORWARD_FLAGS fflags;
extern KINEMATICS_INVERSE_FLAGS iflags;

/* Struct pointers */
//extern struct emcmot_struct_t *emcmotStruct;
extern  MotionStatus *emcmotStatus;
extern  MotionConfig *emcmotConfig;
extern  MotionDebug *emcmotDebug;
//extern struct emcmot_error_t *emcmotError;

extern MotionCommandBuffer *commandShmem;
extern VRobotMotionCommand  *motCmd;
extern MotionCommandQueue *interpList;
extern VRobotMotionCommand *emcmotCommand;
//extern unsigned char *io_output_shmem;
//extern unsigned char *io_input_shmem;
extern  int over_vel;
//extern volatile LC_NC *M_Mem;
extern PmRotationMatrix Rtmp;
extern PmRotationVector Vector;

extern double PUMA_A1;
extern double PUMA_A2;
extern double PUMA_A3;
extern double PUMA_D4;
extern double PUMA_D6;

extern t_lock Mutex;
extern MotionFeedback *motionFb;
//extern t_servo* g_servo[6];

extern int errorCode;
/***********************************************************************
*                    PUBLIC FUNCTION PROTOTYPES                        *
************************************************************************/

/* function definitions */
extern void emcmotCommandHandler(void *arg, long period);
extern void emcmotController(void *arg, long period);
extern void ReadInput(void);
extern void WriteOutput(void);
extern int extIORead2(unsigned short addrtype,unsigned short IOtype,unsigned short ioaddr);

extern void refresh_jog_limits(MotionJointParameter *joint);
extern void clearHomes(int joint_num);
extern int checkAllHomed(void); 
extern void emcmot_config_change(void);

//#ifndef A8_PLAT
//extern void reportError(unsigned short index, unsigned short word_val, int long_val);	/* Use the rtapi_print call */
//#else
//extern int ReportError(unsigned short index, char type, unsigned short word_val, int long_val, int cancel);
//#endif
/* rtapi_get_time() returns a nanosecond value. In time, we should use a u64
    value for all calcs and only do the conversion to seconds when it is
    really needed. */
//#define etime() (((double) rtapi_get_time()) / 1.0e9)

/* motion flags */

#define GET_MOTION_ERROR_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_ERROR_BIT ? 1 : 0)

#define SET_MOTION_ERROR_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_ERROR_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_ERROR_BIT;

#define GET_MOTION_PLAY_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_PLAY_BIT ? 1 : 0)

#define SET_MOTION_PLAY_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_PLAY_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_PLAY_BIT;

#define GET_MOTION_INPOS_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_INPOS_BIT ? 1 : 0)

#define SET_MOTION_INPOS_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_INPOS_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_INPOS_BIT;

#define GET_MOTION_ENABLE_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_ENABLE_BIT? 1 : 0)

#define SET_MOTION_ENABLE_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_ENABLE_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_ENABLE_BIT;

#define GET_MOTION_DIE_FLAG() (emcmotStatus->motionFlag & EMCMOT_MOTION_DIE_BIT ? 1 : 0)

#define SET_MOTION_DIE_FLAG(fl) if (fl) emcmotStatus->motionFlag |= EMCMOT_MOTION_DIE_BIT; else emcmotStatus->motionFlag &= ~EMCMOT_MOTION_DIE_BIT;


#define GET_JOINT_ENABLE_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_ENABLE_BIT ? 1 : 0)

#define SET_JOINT_ENABLE_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_ENABLE_BIT; else (joint)->flag &= ~EMCMOT_JOINT_ENABLE_BIT;

#define GET_JOINT_ERROR_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_ERROR_BIT ? 1 : 0)

#define SET_JOINT_ERROR_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_ERROR_BIT; else (joint)->flag &= ~EMCMOT_JOINT_ERROR_BIT;

#define GET_JOINT_FERROR_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_FERROR_BIT ? 1 : 0)

#define SET_JOINT_FERROR_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_FERROR_BIT; else (joint)->flag &= ~EMCMOT_JOINT_FERROR_BIT;

#define GET_JOINT_HOMED_FLAG(joint) ((joint)->flag & EMCMOT_JOINT_HOMED_BIT ? 1 : 0)

#define SET_JOINT_HOMED_FLAG(joint,fl) if (fl) (joint)->flag |= EMCMOT_JOINT_HOMED_BIT; else (joint)->flag &= ~EMCMOT_JOINT_HOMED_BIT;

#define SET_JOINT_ERROR_FAULT_FLAG(joint,fl) if (fl) (joint)->errorFlag |=EMCMOT_JOINT_ERROR_FAULT_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_FAULT_BIT;

#define GET_JOINT_ERROR_FAULT_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_FAULT_BIT ? 1 : 0)

#define SET_JOINT_ERROR_PHL_FLAG(joint, fl) if (fl) (joint)->errorFlag |= EMCMOT_JOINT_ERROR_MAX_HARD_LIMIT_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_MAX_HARD_LIMIT_BIT;

#define GET_JOINT_ERROR_PHL_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_MAX_HARD_LIMIT_BIT ? 1 : 0)

#define SET_JOINT_ERROR_NHL_FLAG(joint, fl) if (fl) (joint)->errorFlag |= EMCMOT_JOINT_ERROR_MIN_HARD_LIMIT_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_MIN_HARD_LIMIT_BIT;

#define GET_JOINT_ERROR_NHL_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_MIN_HARD_LIMIT_BIT ? 1 : 0)

#define SET_JOINT_ERROR_PSL_FLAG(joint,fl) if (fl) (joint)->errorFlag |= EMCMOT_JOINT_ERROR_MAX_SOFT_LIMIT_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_MAX_SOFT_LIMIT_BIT;

#define GET_JOINT_ERROR_PSL_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_MAX_SOFT_LIMIT_BIT ? 1 : 0)

#define SET_JOINT_ERROR_NSL_FLAG(joint,fl) if (fl) (joint)->errorFlag |= EMCMOT_JOINT_ERROR_MIN_SOFT_LIMIT_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_MIN_SOFT_LIMIT_BIT;

#define GET_JOINT_ERROR_NSL_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_MIN_SOFT_LIMIT_BIT ? 1 : 0)

#define GET_JOINT_ERROR_OUTCHN_FAULT_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_OUTCHN_FAULT_BIT ? 1 : 0)
#define SET_JOINT_ERROR_OUTCHN_FAULT_FLAG(joint,fl) if (fl) (joint)->errorFlag |= EMCMOT_JOINT_ERROR_OUTCHN_FAULT_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_OUTCHN_FAULT_BIT;

#define GET_JOINT_ERROR_OUTCHN_SAME_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_OUTCHN_SAME_BIT ? 1 : 0)
#define SET_JOINT_ERROR_OUTCHN_SAME_FLAG(joint,fl) if (fl) (joint)->errorFlag |= EMCMOT_JOINT_ERROR_OUTCHN_SAME_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_OUTCHN_SAME_BIT;

#define GET_JOINT_ERROR_ENCCHN_FAULT_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_ENCCHN_FAULT_BIT ? 1 : 0)
#define SET_JOINT_ERROR_ENCCHN_FAULT_FLAG(joint,fl) if (fl) (joint)->errorFlag |= EMCMOT_JOINT_ERROR_ENCCHN_FAULT_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_ENCCHN_FAULT_BIT;

#define GET_JOINT_ERROR_ENCCHN_SAME_FLAG(joint) ((joint)->errorFlag & EMCMOT_JOINT_ERROR_ENCCHN_SAME_BIT ? 1 : 0)
#define SET_JOINT_ERROR_ENCCHN_SAME_FLAG(joint,fl) if (fl) (joint)->errorFlag |=EMCMOT_JOINT_ERROR_ENCCHN_SAME_BIT; else (joint)->errorFlag &= ~EMCMOT_JOINT_ERROR_ENCCHN_SAME_BIT;

#define GET_JOINT_ENC_FAULT_FLAG(joint) (joint->errorFlag & EMCMOT_JOINT_ENC_FAULT_BIT ? 1 : 0)
#define SET_JOINT_ENC_FAULT_FLAG(joint,fl) if (fl) joint->errorFlag |= EMCMOT_JOINT_ENC_FAULT_BIT; else joint->errorFlag &= ~EMCMOT_JOINT_ENC_FAULT_BIT;

#define GET_JOINT_SV_ALARM_FLAG(joint) (joint->errorFlag & EMCMOT_JOINT_SV_ALARM_BIT ? 1 : 0)
#define SET_JOINT_SV_ALARM_FLAG(joint,fl) if (fl) joint->errorFlag |= EMCMOT_JOINT_SV_ALARM_BIT; else joint->errorFlag &=EMCMOT_JOINT_SV_ALARM_BIT;




//#if defined(LINUX_VERSION_CODE) && !defined(SIM)
//#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
//#define HAVE_CPU_KHZ
//#endif
//#endif

#endif /* MOT_PRIV_H */
