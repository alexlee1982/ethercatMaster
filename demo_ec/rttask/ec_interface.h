#ifndef EC_INTERFACE_H
#define EC_INTERFACE_H
#include <stdint.h>
#include "ec_status.h"

//period dc刷新周期，应该和插补周期一致，单位ns
extern int ec_master_init(uint32_t period);
//从站上使能，下使能。joint为从站号码:0-5, enable:0-1 0:下使能，1:上使能。
extern void ec_slave_enable(uint8_t joint,uint8_t enable);
//从站复位 ，joint从站号码:0-5
extern void ec_slave_reset(uint8_t joint);

//CSP模式下，更新目标位置 joint为从站号码:0-5, target_position: 目标位置 ，单位:脉冲
extern void ec_slave_csp_target_position(uint8_t joint,int32_t target_position);

//读取伺服驱动器状态反馈
extern void ec_slave_status(uint8_t joint,ec_user_statust_t *status);


extern void ec_call_back(void);

#endif
