#ifndef MOTIONPARAMETER_H
#define MOTIONPARAMETER_H
#define PARAMETER_SET_CLEAR		-1
#define PARAMETER_SET_OK			0
#define PARAMETER_SET_FAILED 		1
#define  MOTION_PARAMETER_KEY		200	
#define MOTION_COMMAND_KEY			201


typedef struct
{
	int head;
	int type; 	//参数类型宏
	union{
		double doubleValue;
		int intValue;
		unsigned int uIntValue;
		}value;
	unsigned int axis;
	int setFlag;		//参数设置标志PARAMETER_SET_OK	PARAMETER_SET_FAILED
	int tail;
}MotionParameter;

#endif
