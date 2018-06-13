#ifndef EC_STATUS_HH
#define EC_STATUS_HH
#include <unistd.h>
typedef struct
{
	uint16_t	status_word;			//伺服从站状态字
	uint8_t 	enabled;		 		//伺服是否上使能 0: 下使能状态，1:上使能状态。
	uint8_t		fault;				//伺服错误状态。
	uint8_t 	error_code;			//伺服错误码，0:无错误，其他 : 错误码
	int32_t		target_position;	//目标位置		
	int32_t		actual_position;	//实际位置值
}ec_user_statust_t;

typedef struct
{
	uint8_t 	enable;			//伺服上使能指令，1为上使能，当伺服状态变为使能时，清零。
	uint8_t 	disable;		//伺服下使能指令，1为下使能，当伺服为下使能状态时，清零。
	uint8_t		reset;			//伺服复位指令，1表示复位。当伺服状态重新变为231(shutdown)时，该值清零。
	ec_user_statust_t status;		//伺服驱动器当前状态信息
}ec_user_slave_t;

#endif