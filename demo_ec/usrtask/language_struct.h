#ifndef LANGUAGE_STRUCT_H
#define LANGUAGE_STRUCT_H

#define ABNORMAL_LARGE 100000

enum wordType
{
	W_MOVJ,
	W_MOVL,
	W_MOVC,
	W_VJ,
	W_VL,
	W_VR,
	W_J1,
	W_J2,
	W_J3,
	W_J4,
	W_J5,
	W_J6,
	W_X,
	W_Y,
	W_Z,
	W_A,
	W_B,
	W_C
};
enum commandType
{
	C_MOVJ,
	C_MOVL,
	C_MOVC
};
typedef struct 
{
	double joint[6];
}JointPosition;
typedef struct
{
	double x;
	double y;
	double z;
	double a;
	double b;
	double c;
}DescartPosition;

typedef struct
{
	JointPosition jp;
	DescartPosition p;
}Position;

typedef struct
{
	double jointVelocity;
	double linearVelocity;
	double angularVelocity;
}Velocity;

typedef struct
{
	enum commandType cmdType;
	Position pos;
	Velocity vel;
}Line;

#endif
