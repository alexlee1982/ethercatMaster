#ifndef MOTION_INER_H
#define MOTION_INER_H
#include "tp.h"


#define DOUT_NUM    6
#define DEFAULT_TC_QUEUE_SIZE  2000
#ifndef MAX_JOINTS
#define MAX_JOINTS 9
#endif
#define MAX_FILENAME_LENGTH       255
#define EMCMOT_ERROR_NUM 32	/* how many errors we can queue */
#define EMCMOT_ERROR_LEN 256	/* how long error string can be */
#define EMCMOT_ALARM_SIZE 32

#define CALCULATECYCLETIME 500000000	//nsec

 typedef unsigned long EMCMOT_MOTION_FLAG;
 
//IOtype
#define   NONE	 0
//#define   IN	        1
//#define   OUT	        2
/* bit masks */
#define EMCMOT_MOTION_ENABLE_BIT      0x00000001      //使能
#define EMCMOT_MOTION_INPOS_BIT       0x00000002       //到位
#define EMCMOT_MOTION_PLAY_BIT       0x00000004           //再现摸式
#define EMCMOT_MOTION_ERROR_BIT       0x00000008         //错误
#define EMCMOT_MOTION_EMERG_BIT      0x00000010          //急停
#define EMCMOT_MOTION_PAUSE_BIT      0x00000020          //暂停
#define EMCMOT_MOTION_SVALM_BIT     0x00000040
#define EMCMOT_MOTION_DIE_BIT           0x00000080

typedef unsigned long EMCMOT_MOTION_ERROR_FLAG;


typedef unsigned long EMCMOT_AXIS_ERROR_FLAG;


//add by swtang 20130121
typedef unsigned long EMCMOT_JOINT_FLAG;
 
#define EMCMOT_JOINT_ENABLE_BIT         0x00000001
#define EMCMOT_JOINT_INPOS_BIT          0x00000004
#define EMCMOT_JOINT_ERROR_BIT          0x00000008
#define EMCMOT_JOINT_FERROR_BIT         0x00001000
#define EMCMOT_JOINT_FAULT_BIT          0x00002000
#define EMCMOT_JOINT_HOMED_BIT          0x00000002


//add by swtang 20130121
typedef unsigned long EMCMOT_JOINT_ERROR_FLAG;

/* macros definition -- */
#define EMCMOT_JOINT_ERROR_FAULT_BIT                                 0x00000001
#define EMCMOT_JOINT_ERROR_MAX_SOFT_LIMIT_BIT             0x00000010
#define EMCMOT_JOINT_ERROR_MIN_SOFT_LIMIT_BIT             0x00000020
#define EMCMOT_JOINT_ERROR_MAX_HARD_LIMIT_BIT            0x00000004
#define EMCMOT_JOINT_ERROR_MIN_HARD_LIMIT_BIT            0x00000008
#define EMCMOT_JOINT_ERROR_OUTCHN_FAULT_BIT             0x00000010
#define EMCMOT_JOINT_ERROR_OUTCHN_SAME_BIT              0x00000020
#define EMCMOT_JOINT_ERROR_ENCCHN_FAULT_BIT             0x00000040
#define EMCMOT_JOINT_ERROR_ENCCHN_SAME_BIT              0x00000080
#define EMCMOT_JOINT_ENC_FAULT_BIT       0x00000100
#define EMCMOT_JOINT_SV_ALARM_BIT        0x00000200
/* motion controller states */

    typedef enum {
	EMCMOT_MOTION_DISABLED = 0,
	EMCMOT_MOTION_TEACH,
	EMCMOT_MOTION_PLAY
    } motion_state_t;

    typedef struct {
//add by swtang 20130121
EMCMOT_JOINT_ERROR_FLAG errorFlag;      /* 关节错误标志 */   
EMCMOT_JOINT_FLAG flag;	                           /* 关节状态标志 */
double pos_cmd;		                     /* 关节命令位置 */
double jog_vel;         // 轴jog 速度
double pos_fb;		                     /* 关节反馈位置*/
double ferror;		                            /*实际随动误差 */	
double ferror_high_mark;	             /* 最大随动误差*/
double max_ferror;	                   /* 最大随动误差上限 */
double min_ferror;	                      /* 零速随动误差*/
double max_pos_limit;	                  /* 关节正向位置极限*/
double min_pos_limit;  	                 /* 关节负向位置极限 */
} robot_joint_status_t;

  typedef struct {
//add by swtang 20130121
EMCMOT_AXIS_ERROR_FLAG errorFlag;      /* 关节错误标志 */   
double pos_cmd;		                     /* 关节命令位置 */
double jog_vel;         // 轴jog 速度
double pos_fb;		                     /* 关节反馈位置*/
double ferror;		                            /*实际随动误差 */	
double ferror_high_mark;	             /* 最大随动误差*/
double max_ferror;	                   /* 最大随动误差上限 */
double min_ferror;	                      /* 零速随动误差*/
double max_pos_limit;	                  /* 关节正向位置极限*/
double min_pos_limit;  	                 /* 关节负向位置极限 */
} robot_axis_status_t;
//add by swtang 20130121

typedef enum {
	EMCMOT_COMMAND_OK = 0,	/* cmd honored */
	EMCMOT_COMMAND_UNKNOWN_COMMAND,	/* cmd not understood */
	EMCMOT_COMMAND_INVALID_COMMAND,	/* cmd can't be handled now */
	EMCMOT_COMMAND_INVALID_PARAMS,	/* bad cmd params */
	EMCMOT_COMMAND_BAD_EXEC	/* error trying to initiate */
    } cmd_status_t;

typedef enum {
	JOINT_COOR = 0,         //关节坐标系
	CAR_COOR,                    //直角坐标系
	CYL_COOR,                    //圆柱坐标系
	USER_COOR ,                  //用户坐标系
	TOOL_COOR,                   //工具坐标系
	EXTA_CTL                   //
    } robot_coordinate_t;



    typedef struct {
	unsigned char head;	/* flag count for mutex detect */
	int split;		/* number of split command reads */
	TP_STRUCT queue;	/* coordinated mode planner */
	TP_STRUCT qtmp;	/* coordinated mode planner */
	TC_STRUCT queueTcSpace[DEFAULT_TC_QUEUE_SIZE + 10];
       int enabling;		/* starts up disabled */
	int teaching;	/* starts up in free mode */
       int playing;
	int linkSimTest;
        int allHomed;
        int svon_count;
	double net_feed_scale[MAX_JOINTS];                           //示教修调
	double rawOutput[MAX_JOINTS];
	int inputRot[MAX_JOINTS];
	int inputInit[MAX_JOINTS];
	int inputInitCount[MAX_JOINTS]; 
	unsigned char doutBuffer[DOUT_NUM];
	int    addrtype;
	int     ioaddr ;
	int     iostate; 
	int adjust;
	PmCartesian Xvector,Yvector,Zvector,ORG;
       PmCartesian   robot_tcp;
       PmRotationMatrix   robot_tcf;
       PmRotationMatrix   robot_invtcf;

	PmRotationMatrix   robot_R0;
	PmCartesian   robot_P0;
	PmRotationMatrix   robot_RT;
	PmCartesian   robot_PT;
	PmCartesian Porg;
	PmHomogeneous  goalFrame;	
	PmHomogeneous  currentFrame;	
	int has_extaxis;
	int home_free;
	int speed_level;

	unsigned char tail;	/* flag count for mutex detect */
    } MotionDebug;

typedef struct
{
	unsigned char head;	
	int commandEcho;   // 对命令类型的响应
	int commandNumEcho;	
	cmd_status_t commandStatus;	
	motion_state_t motion_state; 
	int IObyte;
       int rapidMode;
	int joint_type;                            //关节运动类型
	int config_num;
	unsigned int heartbeat;
	double net_feed_scale;                          
	int linkErr;                     
	double distance_was_gone;
	double distance_to_go;                       
	EMCMOT_MOTION_FLAG motionFlag;	
	EMCMOT_MOTION_ERROR_FLAG motErrorFlag;           

	int id;			      
	char  fileName[MAX_FILENAME_LENGTH];                  
	robot_coordinate_t coordinate_type;                  
	int on_soft_limit;	            
	int on_soft_limit2;	            
	int estop;                      
	int pause;                
        int playing;    	
	int enble;
	int running;
	int queueFull;	
	robot_joint_status_t  joint_status[MAX_JOINTS];      //关节状态数组
	robot_axis_status_t  axis_status[MAX_JOINTS];      //关节状态数组
	unsigned long linkAlmFlag;           
	unsigned long linkAlmReport;       
//	ALARM_MSG autoAlm[EMCMOT_ALARM_SIZE];           // 自动报警(伺服报警形式修改后新增的变量)
	//ALARM_MSG manuAlm[EMCMOT_ALARM_SIZE];          // 手动报警
      // Param_Modi param_modi;

	PmJoint joint_fb;                            //关节反馈值
	PmJoint joint_cmd;                        //关节命令值
	RobotPose carte_pos_cmd;	                        //直角坐标命令位置	
	RobotPose carte_pos_fb;	                           //直角坐标反馈位置
       double speed_scale;
//int speed_level;
       RobotPose tool_tcp;       //反馈工具信息
       
	unsigned char tail;	                              

   }MotionStatus;
 typedef struct {
        int  not_first_input;
	double max_pos_limit;	//关节正软限
	double min_pos_limit;	        //关节负软限
	double max_jog_limit;	       //jog正向最大距离
	double min_jog_limit;	       //jog负向最大距离
	int linkSimTest;
	double vel_limit;         // 轴进给最大速度         -未使用
	double jog_vel;           // 轴jog 速度                         -未使用
	double acc_limit;	      // -未使用
	double min_ferror;	/* zero speed following error limit */
	double max_ferror;	/* max speed following error limit */
       unsigned long  errorFlag;      /* 关节错误标志 */   
	unsigned short  flag;	                           /* 关节状态标志 */
	double coarse_pos;	       //   -未使用
	double pos_cmd;		   //当前周期命令位置
        double old_pos_cmd;     //前一周期命令位置
	double cur_vel;               //超速处理时当前关节速度
	double vel_cmd;		  //关节命令速度
	double pos_fb;		   // 关节反馈位置
	double ferror;		
	double ferror_limit;	/* limit depends on speed */
	double ferror_high_mark;	/* max following error */

	int motorInputOffset;
	int motorInputOffset2;
	int svNum;             // 轴对应的伺服驱动器通道号
        int svtype;
        double pulse_value;
	double inv_pulse_value;
       int offsetInit;         //-未使用
//	PID_STRUCT pid;
       int     servoEncoderPolarity;  //编码器极性
       int encoder_counts;              //编码器线数
       double pitch;                          //丝杠导程
	int tranRatioNumer;             //传动比分子
	int tranRatioDenom;            //传动比分母
	double svMotorSafFac;        //计算Jvel_limit最大安全系数
       double motor_max_vel;      //电机最大转速
       int servoOutputPolarity;      //输出极性
       double max_volt;                  //最大电压
	double motorOutputScale;   //电机输出比例
	double servo_zero_offset;   // -未使用
       int rawMotorInput;                 //编码器位置
	int oldMotorInput;                 //未使用
	double motorInputScale;       //电机输入比例
	double motorInverseInputScale; //电机输入比例倒数
//	CUBIC_STRUCT cubic;	    //未使用
	int on_pos_limit;	           //未使用
	int on_neg_limit;	           //未使用

	double teachJVel;     //关节示教速度
	double JVel;              //关节再现速度
	double Jvel_limit;     //关节最大速度

	double Jteach_acc;   //关节示教加速度
	double Jplay_acc;    //关节再现加速度

	double high_speed_scale;	  //高速修调比例	
	double mid_speed_scale;	 //中速修调比例		
	double low_speed_scale;	//低速修调比例			
	double tiny_speed_scale;	//微速修调比例			

//	home_state_t home_state;	/* state machine for homing */
	int pg_chn;
	int enc_position;
	int enc_input_chn;
	int enc_line_detect;

	double big_vel;		// - 未使用
    } MotionJointParameter;


#endif
