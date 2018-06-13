#ifndef INC_RTOS_H
#define	INC_RTOS_H

#include "Common.h"


#ifdef WIN32
#include <windows.h>
#include <iostream>
// 锁相关操作
typedef CRITICAL_SECTION t_lock;
#define	INIT_LOCK(lock)		InitializeCriticalSection(&(lock))
#define	LOCK(lock)			EnterCriticalSection(&(lock))
#define	UNLOCK(lock)		LeaveCriticalSection(&(lock))
#else
#include <pthread.h>
typedef pthread_mutex_t t_lock;
#define	INIT_LOCK(lock)		pthread_mutex_init(&(lock),NULL)
#define	LOCK(lock)			pthread_mutex_lock(&(lock))
#define	UNLOCK(lock)		pthread_mutex_unlock(&(lock))
#endif

#ifdef __cplusplus
extern "C"{
#endif

// 初始化，只需要调用一次
void RTOS_Init();
// 获取系统时间，单位是纳秒
u64 RTOS_GetTimeNs();
// 以us为单位休眠
void RTOS_SleepUs(u32 us);
// 以ns为单位休眠
void RTOS_SleepNs(u64 ns);





#ifdef __cplusplus
}
#endif



#endif

