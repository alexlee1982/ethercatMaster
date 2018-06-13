#ifndef MOTIONINTERFACEBUFFER_H
#define MOTIONINTERFACEBUFFER_H

#define MOTION_COMMAD_QUEUE_SIZE 2000
#define MAX_FILE_NAME_NUMBER 255

#define COMMAND_DONE		1 
#define COMMAND_EXEC		2
#define COMMAND_ERROR		3

#define COORDINATE_TYPE_JOINT 	1
#define COORDINATE_TYPE_LINEAR 	2
#define COORDINATE_TYPE_USER		3
#define COORDINATE_TYPE_TOOL		4


#define MOVE_DIR_POSITIVE 		1
#define MOVE_DIR_NEGATIVE 		-1
#include "Common.h"
//关节位置点

//__pragma(pack(push, 4))
#pragma pack(push, 1)

typedef struct
{
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;
	double j6;
}JointPoint;

typedef struct
{
	JointPoint org;
	JointPoint ox;
	JointPoint oy;
}ThreeJointPoints;


//笛卡尔坐标系下的点位信息
typedef struct
{
double x;
double y;
double z;
}DescartesPoint;
//笛卡尔位置下的姿态信息(rpy角)
typedef struct
{
double a;			//绕x轴旋转的角度(单位为度)
double b;			//绕y轴旋转的角度(单位为度)
double c;			//绕z轴旋转的角度(单位为度)
}Pose;
//笛卡尔坐标系下的位姿信息
typedef struct
{
double x;
double y;
double z;
double a;
double b;
double c;
}PointPose;


/*用户坐标系信息
x/y/z表示用户坐标系原点对机器人坐标系原点的偏移
a/b/c表示将机器人坐标系绕z轴旋转c度，
再绕y轴旋转b度，再绕x轴旋转a度可以得到用户坐标系。
*/
typedef struct
{
	double x,y,z,a,b,c;
}UserCoordianteInformation;


//旋转矩阵信息，x,y,z分别表示新姿态X/Y/Z轴在原坐标系下的向量。
typedef struct
{
DescartesPoint x,y,z;
}RotationMatrix;

//伺服使能参数，0为下使能，1为上使能。
typedef struct
{
int enable;
}ServoEnable;


//用户坐标系需要输入的坐标参数。
typedef struct
{	
	JointPoint originPoint;	//原点位置信息。
	JointPoint AxisXPoint;	// X轴上一点位置信息。
	JointPoint PlaneXYPoint;	//XY平面上一点位置信息。
}CalUserCoordianteInformation;

//工具坐标系需要输入的坐标参数。
typedef struct
{
	double x,y,z,rx,ry,rz;	//工具坐标系的偏移量。

}ToolCoordianteInformation;

//坐标信息结构体，暂时只支持关节与直角。
typedef struct
{
	u32 coordinateType;	//坐标系类型使用宏: 
								//COORDIANTE_TYPE_JOINT,COORDINATE_TYPE_LINEAR,
								//COORDINATE_TYPE_TOOL
	//将来加入坐标系的参数实现用户坐标系与工具坐标系
	
		union{
			
		UserCoordianteInformation user;
		ToolCoordianteInformation tool;
		}CoordinateInformation;

															
}CoordinatInformation;



//单轴运动结构，当下发该命令时，
//将VRobotMotionCommand用的type指定为SINGLE_AXIS_MOVE_TYPE
//并对联合体中的singleMove赋值。
typedef struct
{
	int axis;		//轴号:1-6	//或写成宏。
	 int direction;	//运动方向:正向: MOVE_DIR_POSITIVE 负向MOVE_DIR_NEGATIVE 
	 double vel;			//单轴运动速度百分比，0.0-100.0
	double endPoint;	// 终点位置，暂时没实现该功能。
	CoordinatInformation coordinate;	//坐标系信息。
}SingleAxisMove;
//关节运动信息，现在只包括位置点与速度两个参数，后续加入其它信息。
typedef struct
{
	JointPoint endPoint;		//终点位置信息。
	double vJ;				//速度信息。取值范围根据具体设计改变。
	double acc;				//加速度
	double dec;				//减速度
	int id;					//命令号码，用来区分指令的身份标识。

}JointMoveInformation;

//笛卡尔直线运动信息，现在只包括位置点与速度两个参数，后续加入其它信息。
typedef struct
{
	DescartesPoint endPoint;		//终点位置信息。
	//RotationMatrix rot;				//终点姿态信息
	Pose zyx;					//终点姿态ZYX欧拉角
	//PmEulerZyx zyx;				//终点姿态ZYX欧拉角
	int free;						//为0时选用rot作为终点姿态，为1时使用起点姿态
	double vL;				//速度信息。取值范围根据具体设计改变。单位mm/s?
	double vR;				//姿态速度，单位为度/秒
	double acc;				//加速度
	double dec;				//减速度
	int id;					//命令号码，用来区分指令的身份标识。
	UserCoordianteInformation user;			//用户坐标系信息
	ToolCoordianteInformation tool;			//工具坐标系信息
}LinearDescartesMoveInformation;





//直线运动信息，现在只包括位置点与速度两个参数，后续加入其它信息。
typedef struct
{
	JointPoint endPoint;		//终点位置信息。
	double vL;				//速度信息。取值范围根据具体设计改变。单位mm/s?
	double vR;				//姿态速度，单位为度/秒
	double acc;				//加速度
	double dec;				//减速度
	int id;					//命令号码，用来区分指令的身份标识。
}LinearMoveInformation;
//圆弧运动信息，现在只包括位置点与速度两个参数，后续加入其它信息。
typedef struct
{
	JointPoint endPoint[3];		//圆弧插补的3个点位信息。
	double vL;				//速度信息。
	double vR;
	double acc;				//加速度
	double dec;				//减速度
	int id ;					//命令号码。
}CircularMoveInformation;

typedef struct
{
	DescartesPoint endPoint[3];		//终点位置信息，0-2依次对应圆弧起点/中间点/终点
	//RotationMatrix rot[3];			//终点姿态信息，0-2依次对应起点/中间点/终点姿态
								//暂时rot[1]即中间点姿态可以不填，没处理。
	Pose zyx[3];					//ZYX 欧拉角表示的姿态，a-X ,b-y,c-z
	int free;						//是否选用上一段运动段的终点姿态作为圆弧运行的姿态，
								//0为使用rot中的信息，free为使用前一点的姿态
	double vL;				//速度信息。取值范围根据具体设计改变。单位mm/s?
	double vR;
	double acc;				//加速度
	double dec;				//减速度
	int id;					//命令号码，用来区分指令的身份标识。
	UserCoordianteInformation user;			//用户坐标系信息
	ToolCoordianteInformation tool;			//工具坐标系信息
}CircularDescartesMoveInformation;

//运动命令参数
typedef union
{
	ServoEnable svEable;
	SingleAxisMove singleMove;			//单轴运动开始，VRobotMotionCommand用的type
									//单轴运动停止。只需要一个type没有参数。
									//下发一个单轴停止命令所有轴都停止。规避风险。			
	JointMoveInformation movJ;				//关节插补运动movj
	LinearMoveInformation movL;				//直线插补运动movl
	CircularMoveInformation movC;				//圆弧插补运动movc
	LinearDescartesMoveInformation movDL;		//笛卡尔坐标输入直线插补
	CircularDescartesMoveInformation movDC;		//笛卡尔坐标输入圆弧插补
}MotionCommandParameter;


//运动命令结构体，包含了运动模块可以执行的各项命令
typedef struct
{
	u32 head; //写入命令时将head+1，
					//写完时将head和tail写为相同的值。
					//在head 和tail 为不同的值时，运动模块不会读取其中的值。
	int type;		//命令类型，宏:包含在motionCommandType.h文件中

	int serialNumber;	//命令串号，每次下发的指令应该与上一个指令串号+1。
	u32 currentLineNumber; //该指令对应的命令行号。
	u32 currentFileNumber; //该指令所在的文件号
	MotionCommandParameter motionCmdParameter;	//运动指令参数
	u32 tail;

}VRobotMotionCommand;



//命令队列，环形队列。
typedef struct
{
	int start,end; // 队列的首与尾的位置。
	int motionState;	//运动模块的执行状态。
					//分为运动执行完成完成:DONE 正在执行:EXECUTE 与运行出错ERROR
	u32 currentLineNumber;	//当前运行指令的行号
	u32 currentFileName[MAX_FILE_NAME_NUMBER]; //当前执行指令的文件名
	VRobotMotionCommand motionCmd[MOTION_COMMAD_QUEUE_SIZE]; //运动命令数组
}MotionCommandQueue;
//运动命令缓存，其中包括立即命令与队列命令
typedef struct
{
	MotionCommandQueue motionCmdQ;	//队列命令
	VRobotMotionCommand motionCmd;	//运动立即命令

}MotionCommandBuffer;
typedef VRobotMotionCommand * Command_PTR;
#pragma pack(pop)
//__pragma(pack(pop))
#endif
