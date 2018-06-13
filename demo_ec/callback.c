
//#include "contrller.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motionInterfaceBuffer.h"
#include "motionParameterType.h"
#include "motionParameter.h"
#include "motionIner.h"
#include "motionConfig.h"
//#include "pumakins.h"

#include "mot_priv.h"
int temporaryTestFlag=0;
extern int analyzePostion();
extern int getCommand();
 int CTRL_Callback(long us)
{
	int ret = 0;
	//errorCode=0;
	ret=getCommand();
	ret=analyzePostion();
	return ret;
}

