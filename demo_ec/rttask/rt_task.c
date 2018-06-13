#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include "contrller.h"
#include "ec_interface.h"



#define ONE_BILLION		1000000000
#define TEN_MILLIONS	10000000
#define ONE_MILLON		1000000

#define HIPRIO 99
#define LOPRIO 0


pthread_t rt_thread;
int done;
int exitMain(int sig)
{
	done=0;
	CTRL_Exit();
	return 0;
}

static void* rt_task(void *arg)
{
	ec_call_back();
	/*struct timespec ts;
	ts.tv_sec=0;
	ts.tv_nsec=TEN_MILLIONS;		// 1ms
	printf("entre the rt_task !\r\n");
	while(done)
	{
		
		clock_nanosleep(CLOCK_REALTIME,0,&ts,NULL);
		CTRL_Callback(10000);
		
	}*/
	
	printf("the rt_task exit for now!\r\n");
	return NULL;
}




int main(int argc, char ** argv)
{
	struct sched_param rtParam={.sched_priority=HIPRIO};
	sigset_t set;
	pthread_attr_t attr;
	int sig,errno;
	done=1;
	sigemptyset(&set);
	sigaddset(&set,SIGINT);
	sigaddset(&set,SIGTERM);


	//signal(SIGINT,exitMain);
	
	pthread_sigmask(SIG_BLOCK,&set,NULL);
	

	CTRL_Init(10000);
	ec_master_init(1000000);	


	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE);
	pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr,SCHED_RR);
	pthread_attr_setschedparam(&attr,&rtParam);
	errno=pthread_create(&rt_thread,&attr,&rt_task,NULL);

	
	sigwait(&set,&sig);
	

	printf("the thread and the main process join here and exit the process\r\n");
	pthread_cancel(rt_thread);
	pthread_join(rt_thread,NULL);
	

	return 0;
}
