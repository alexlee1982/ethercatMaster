#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "Common.h"
//#include "MotionApi.h"
typedef struct
{
	f64 PUMA_A[5];
	f64 teachLinearV;
	f64 playLinearV;
	f64 linearMaxV;
	f64 linearMoveAcc;
	f64 rotateMoveV;
	f64 rotateMoveAcc;
	f64 linearAccTime;
	f64 linearMaxAcc;
	f64 linearPosLimit[6];
	f64 linearNegaLimit[6];
	f64 jointPosLimit[6];
	f64 jointNegaLimit[6];
	f64 jointMoveVel[6];
	f64 jointMoveAcc[6];
	f64 jointMoveMaxVel[6];
	f64 mechTransRatio[6];
	s64 mechTransDenominator[6];
	s64 mechTransNumerator[6];

	unsigned long servoPulsePerRound[6];
	f64 ServoMaxVelocity[6];
	 s64 encodeOriginOffset[6];
	int	encoderType[6];	// 编码器类型
	int	servoPolarity[6];	// 极性

	s64 coupleRate;
	
	char socketIP[255];
	int socketPortNumber;
	int cycleTime;


}MotionConfig;



#endif 
