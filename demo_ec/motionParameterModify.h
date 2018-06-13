#ifndef MOTIONPARAMETERMODIFY_H
#define MOTIONPARAMETERMODIFY_H
/*设置机器人臂长参数
	value中为臂长参数，
	number 为机械臂号，取值范围为0-5。
*/
extern int SET_PUMA_A(double value,int number);
/*设置示教直线单轴运动的最大速度
	value 中为速度之，单位为mm/s
*/
extern int SET_TEACH_LINEAR_MOVE_MAX_VELOCITY(double value);
/*
	设置直线插补指令movl与movc的最大速度
	value 中为速度之，单位为mm/s	
*/
extern int SET_PLAY_LINEAR_MOVE_MAX_VELOCITY(double value);
/*
	设置默认直线加速时间
	time中加速时间值，单位为s
*/
extern int SET_LINEAR_MOVE_ACC_TIME(double time);
/*
	设置默认旋转运动加速时间。
	time中为加速时间值，单位为s。
*/
extern int SET_ROTATE_MOVE_ACC_TIME(double time);

/*
	设置直线轴正向最大行程
	value中为行程值，单位为mm，并且为正值。
	axis中为轴号，取值范围为宏AXIS_X/AXIS_Y/AXIS_Z
*/
extern int SET_LINEAR_POSITIVE_LIMIT(double value,int axis);

/*
	设置直线轴负向最大行程
	value中为行程值，单位为mm,并且为负值。
	axis中为轴号，取值范围为宏AXIS_X/AXIS_Y/AXIS_Z
*/
extern int SET_LINEAR_NEGATIVE_LIMIT(double value,int axis);
/*
	设置关节轴正向限制
	value 中关节的角度值输入1-180的角度值。
	axis中为轴号，取值范围为0-5
*/
extern int SET_JOINT_POSITIVE_LIMIT(double value,int axis);
/*
	设置关节轴负向限制
	value中为关节的角度之输入-180-0的角度值。
	axis中为轴号，取值范围为0-5。

*/
extern int SET_JOINT_NEGATIVE_LIMIT(double value,int axis);
/*
	设置关节轴的最大速度
	value中为关机的速度，单位角度/s。?
	axis为轴号，取值范围为0-5。
*/
extern int SET_JOINTMOVE_MAX_VELOCITY(double value, int axis);
/*
	设置关节轴的减速比。
	value中为减速比的值。输入正值。
	axis为轴号，取值范围为0-5。
*/
extern int SET_MECHANICAL_TRANSMISSION_RATIO(double value,int axis);

/*
	设置电机编码器每转的脉冲数。
	value中为脉冲的值。输入正整数值。
	axis为轴号，取值范围为0-5。
*/
extern int SET_SERVO_PULSE_PER_ROUND(unsigned int value,int axis);

/*
	设置电机编器的反馈极性。
	polarity中填写+1或-1 。或宏SERVO_POSITIVE或SERVO_NEGATIVE
	axis 为轴号取值0-5。

extern SET_SERVO_ENCODER_POLARITY(int polarity,int axis);*/

/*
	设置电机的最大转速。
	velocity中写入最大转速。正整数。单位r/min.
	axis中写入轴号0-5。
*/
extern int SET_SERVO_MAX_VELOCITY(double velocity,int axis);

/*
	设置电机决定编码器的零点偏移量。
	offset中写入编码器的偏移量，非负整数。
	axis中写入轴号0-5。
*/
extern int SET_ENCODER_ORIGIN_OFFSET(unsigned int offset, int axis);






#endif
