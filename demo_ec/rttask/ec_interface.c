
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>

/****************************************************************************/
#include "ecrt.h"
#include "ec_status.h"
/****************************************************************************/

#include "contrller.h"


 // Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_REALTIME
#define MEASURE_TIMING
 
 /****************************************************************************/
 
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
 
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
	 (B).tv_nsec - (A).tv_nsec)
 
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
 
 /****************************************************************************/
 
 // EtherCAT
 static ec_master_t *master = NULL;
 static ec_master_state_t master_state = {};
 
 static ec_domain_t *domain1 = NULL;
 static ec_domain_state_t domain1_state = {};
 static ec_slave_config_t *ec_slave[6];
 static ec_slave_config_state_t ec_slave_state[6];

 ec_user_slave_t ec_slave_data[6];

 // process data
 static uint8_t *domain1_pd = NULL;


#define Panasonic_Slave_0 0x0000066f, 0x60380007 
#define Panasonic_Slave_1 0x0000066f, 0x60380007 
#define Panasonic_Slave_2 0x0000066f, 0x60380006 
#define Panasonic_Slave_3 0x0000066f, 0x60380004
#define Panasonic_Slave_4 0x0000066f, 0x60380004 
#define Panasonic_Slave_5 0x0000066f, 0x60380004 


 static unsigned int counter = 0;
 static unsigned int blink = 0;
 static unsigned int sync_ref_counter = 0;
 const struct timespec cycletime = {0, PERIOD_NS};


 static unsigned int off_Controlword[6];
 static unsigned int off_operation_mode[6];
 static unsigned int off_target_position[6];
 static unsigned int off_Statusword[6];
 static unsigned int off_Modes[6];
 static unsigned int off_position_actual_value[6];

 const static ec_pdo_entry_reg_t domain1_regs[] = {
	 {0,0,	Panasonic_Slave_0, 0x6040, 0, &off_Controlword[0]},
	 {0,0,	Panasonic_Slave_0, 0x6060, 0, &off_operation_mode[0]},
	 {0,0,	Panasonic_Slave_0, 0x607a, 0, &off_target_position[0]},
	 {0,0,	Panasonic_Slave_0, 0x6041, 0, &off_Statusword[0]},
	 {0,0,	Panasonic_Slave_0, 0x6061, 0, &off_Modes[0]},
	 {0,0,	Panasonic_Slave_0, 0x6064, 0, &off_position_actual_value[0]},

	 {0,1,	Panasonic_Slave_1, 0x6040, 0, &off_Controlword[1]},
	 {0,1,	Panasonic_Slave_1, 0x6060, 0, &off_operation_mode[1]},
	 {0,1,	Panasonic_Slave_1, 0x607a, 0, &off_target_position[1]},
	 {0,1,	Panasonic_Slave_1, 0x6041, 0, &off_Statusword[1]},
	 {0,1,	Panasonic_Slave_1, 0x6061, 0, &off_Modes[1]},
	 {0,1,	Panasonic_Slave_1, 0x6064, 0, &off_position_actual_value[1]},

	 {0,2,	Panasonic_Slave_2, 0x6040, 0, &off_Controlword[2]},
	 {0,2,	Panasonic_Slave_2, 0x6060, 0, &off_operation_mode[2]},
	 {0,2,	Panasonic_Slave_2, 0x607a, 0, &off_target_position[2]},
	 {0,2,	Panasonic_Slave_2, 0x6041, 0, &off_Statusword[2]},
	 {0,2,	Panasonic_Slave_2, 0x6061, 0, &off_Modes[2]},
	 {0,2,	Panasonic_Slave_2, 0x6064, 0, &off_position_actual_value[2]},



	 {0,3,	Panasonic_Slave_3, 0x6040, 0, &off_Controlword[3]},
	 {0,3,	Panasonic_Slave_3, 0x6060, 0, &off_operation_mode[3]},
	 {0,3,	Panasonic_Slave_3, 0x607a, 0, &off_target_position[3]},
	 {0,3,	Panasonic_Slave_3, 0x6041, 0, &off_Statusword[3]},
	 {0,3,	Panasonic_Slave_3, 0x6061, 0, &off_Modes[3]},
	 {0,3,	Panasonic_Slave_3, 0x6064, 0, &off_position_actual_value[3]},


	 {0,4,	Panasonic_Slave_4, 0x6040, 0, &off_Controlword[4]},
	 {0,4,	Panasonic_Slave_4, 0x6060, 0, &off_operation_mode[4]},
	 {0,4,	Panasonic_Slave_4, 0x607a, 0, &off_target_position[4]},
	 {0,4,	Panasonic_Slave_4, 0x6041, 0, &off_Statusword[4]},
	 {0,4,	Panasonic_Slave_4, 0x6061, 0, &off_Modes[4]},
	 {0,4,	Panasonic_Slave_4, 0x6064, 0, &off_position_actual_value[4]},


	 {0,5,	Panasonic_Slave_5, 0x6040, 0, &off_Controlword[5]},
	 {0,5,	Panasonic_Slave_5, 0x6060, 0, &off_operation_mode[5]},
	 {0,5,	Panasonic_Slave_5, 0x607a, 0, &off_target_position[5]},
	 {0,5,	Panasonic_Slave_5, 0x6041, 0, &off_Statusword[5]},
	 {0,5,	Panasonic_Slave_5, 0x6061, 0, &off_Modes[5]},
	 {0,5,	Panasonic_Slave_5, 0x6064, 0, &off_position_actual_value[5]},
	 {}
 };

 static ec_pdo_entry_info_t slave_pdo_entries[] = {
	 {0x6040, 0x00, 16}, /* Controlword */
	 {0x6060, 0x00, 8}, /* Modes of operation */
	 {0x607a, 0x00, 32}, /* Target position */
	 {0x60b8, 0x00, 16}, /* Touch probe function */
	 {0x603f, 0x00, 16}, /* Error code */
	 {0x6041, 0x00, 16}, /* Statusword */
	 {0x6061, 0x00, 8}, /* Modes of operation display */
	 {0x6064, 0x00, 32}, /* Position actual value */
	 {0x60b9, 0x00, 16}, /* Touch probe status */
	 {0x60ba, 0x00, 32}, /* Touch probe pos1 pos value */
	 {0x60f4, 0x00, 32}, /* Following error actual value */
	 {0x60fd, 0x00, 32}, /* Digital inputs */
 };
 
 static ec_pdo_info_t slave_pdos[] = {
	 {0x1600, 4, slave_pdo_entries + 0}, /* Receive PDO mapping 1 */
	 {0x1a00, 8, slave_pdo_entries + 4}, /* Transmit PDO mapping 1 */
 };
 
 static ec_sync_info_t slave_syncs[] = {
	 {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
	 {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
	 {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
	 {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
	 {0xff}
 };
 /*****************************************************************************/
 
 struct timespec timespec_add(struct timespec time1, struct timespec time2)
 {
	 struct timespec result;
 
	 if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
		 result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		 result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
	 } else {
		 result.tv_sec = time1.tv_sec + time2.tv_sec;
		 result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
	 }
 
	 return result;
 }
 
 /*****************************************************************************/

 void check_domain1_state(void)
 {
	 ec_domain_state_t ds;
 
	 ecrt_domain_state(domain1, &ds);
 
	 if (ds.working_counter != domain1_state.working_counter)
		 printf("Domain1: WC %u.\n", ds.working_counter);
	 if (ds.wc_state != domain1_state.wc_state)
		 printf("Domain1: State %u.\n", ds.wc_state);
 
	 domain1_state = ds;
 }
 
 /*****************************************************************************/
 
 void check_master_state(void)
 {
	 ec_master_state_t ms;
 
	 ecrt_master_state(master, &ms);
 
	 if (ms.slaves_responding != master_state.slaves_responding)
		 printf("%u slave(s).\n", ms.slaves_responding);
	 if (ms.al_states != master_state.al_states)
	 {
		 printf("AL states: 0x%02X.\n", ms.al_states);
		 if(8 == ms.al_states)
		 {
		 	int i=0;
			for(i=0;i<6;i++)
			{
				EC_WRITE_S32(domain1_pd+off_target_position[i],EC_READ_S32(domain1_pd+off_position_actual_value[i]));
			}				
		 }
	 }
		 
	 if (ms.link_up != master_state.link_up)
		 printf("Link is %s.\n", ms.link_up ? "up" : "down");
 
	 master_state = ms;
 }

//伺服上使能状态机
void enalbe_state_machine()
{
	int i = 0;
	uint16_t statusword ;
	for(i = 0;i<6;i++)
	{
		//读取对应从站状态字
		statusword = EC_READ_U16(domain1_pd+off_Statusword[i]);
		//判断上使能或者是下使能命令
		if(1 == ec_slave_data[i].enable)
		{
			//上使能
			//通过判断对应从站的PDO状态字来判断从站状态。
		
			if(0x0027 == (statusword&0x006f)) //状态为operation enabled
			{
				//已经为上使能状态，将上使能状态清空
				ec_slave_data[i].enable = 0;
			}
			//为 switched on 或者是ready to switched on
			else if((0x0023 == (statusword&0x006f))||(0x0021 == (statusword&0x006f)))
			{
				//写控制字
				EC_WRITE_U16(domain1_pd + off_Controlword[i], 0x0f);
			}
			//其他情况，如果是出错了，那就不再执行上使能。
			else if(0x0008 == (statusword&0x004f))
			{
				ec_slave_data[i].enable = 0;
			}
			//其他情况都执行shutdown
			else
			{
				EC_WRITE_U16(domain1_pd + off_Controlword[i], 0x06);
			}
		}
		else if(1 == ec_slave_data[i].disable)
		{
			//下使能
			//当前如果不为使能状态，不需要进行下使能操作。
			if((statusword&0x6f) != 0x0027)
			{
				ec_slave_data[i].disable = 0;
			}
			//如果为上使能状态，进入到switch on 状态
			else
			{
				EC_WRITE_U16(domain1_pd + off_Controlword[i], 0x07);			
			}
		}
		else
		{
			//本周期没有指令，什么都不用做。
		}
	}
}

//伺服复位状态机
void reset_state_machine()
{
	//先将状态字中的fault reset 位置为1，再置为0 ，
	int i = 0;
	static uint8_t fault_reset[6] = {0,0,0,0,0,0};
	uint16_t control_word ;
	uint16_t statusword ;
	
	for(i = 0;i < 6;i++)
	{
		if(1 == ec_slave_data[i].reset)
		{
			statusword = EC_READ_U16(domain1_pd+off_Statusword[i]);
			
			if(0x0008 != (statusword&0x004f))
			{
				//如果当前没有错误，直接返回，不执行复位。
				ec_slave_data[i].reset = 0;
				continue;
			}
			if(0 == fault_reset[i])
			{
				control_word = EC_READ_U16(domain1_pd+off_Controlword[i]);
				control_word |= 0x0080;
				EC_WRITE_U16(domain1_pd + off_Controlword[i], control_word);
			}
			else if(1 == fault_reset[i])
			{
				control_word = EC_READ_U16(domain1_pd+off_Controlword[i]);
				control_word &= 0xff7f;
				EC_WRITE_U16(domain1_pd + off_Controlword[i], control_word);
			}
		}
		
	}
	
	
}



 //init the usr space interface to master , create domain and register all the pdo entries

 int ec_master_init(uint32_t period)
{
	int i=0;
	//
	master = ecrt_request_master(0);
    if (!master)
    {
    	printf("ec_init: failed to request the master \n");
    	return -1;
    }
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
    	printf("ec_init:failed to create domain \n");
        return -1;
    }
    // Create configuration for bus coupler  
    printf("ec_init:Configuring Slaves...\n");
    if (!(ec_slave[0] = ecrt_master_slave_config(
                    master, 0,0, Panasonic_Slave_0))) 
    {
        printf("Failed to get slave[0] configuration.\n");
        return -1;
    }
    if (!(ec_slave[1] = ecrt_master_slave_config(
                    master, 0,1, Panasonic_Slave_1))) 
    {
        printf("Failed to get slave[1] configuration.\n");
        return -1;
    }
    if (!(ec_slave[2] = ecrt_master_slave_config(
                    master, 0,2, Panasonic_Slave_2))) 
    {
        printf("Failed to get slave[2] configuration.\n");
        return -1;
    }
    if (!(ec_slave[3] = ecrt_master_slave_config(
                    master, 0,3, Panasonic_Slave_3))) 
    {
        printf("Failed to get slave[3] configuration.\n");
        return -1;
    }
    if (!(ec_slave[4] = ecrt_master_slave_config(
                    master, 0,4, Panasonic_Slave_4))) 
    {
        printf("Failed to get slave[4] configuration.\n");
        return -1;
    }
    if (!(ec_slave[5] = ecrt_master_slave_config(
                    master, 0,5, Panasonic_Slave_5))) 
    {
        printf("Failed to get slave[5] configuration.\n");
        return -1;
    }
	printf("ec_init:Configuring PDOs...\n");
	for(i=0;i<6;i++)
	{
		if (ecrt_slave_config_pdos(ec_slave[i], EC_END, slave_syncs)) 
		{
	        printf("ec_init:Failed to configure PDOs. for slave[%d]\n",i);
	        return -1;
	    }
	}
	
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) 
	{
        printf("ec_init:PDO entry registration failed!\n");
        return -1;
    }
    // configure SYNC signals for this slave
    printf("ec_init:configuring dcs \n");
	for(i = 0;i < 6;i++)
	{
		ecrt_slave_config_dc(ec_slave[i], 0x0300, PERIOD_NS, 4400000, 0, 0);
	}
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
    {
    	printf("ec_init:failed to active the master \n");
		return -1;
    }
    if (!(domain1_pd = ecrt_domain_data(domain1))) 
	{
        printf("domain1= %x domain_pd =%x \n",domain1,domain1_pd);
		return -1;
    }

	return 0;
}
void ec_slave_enable(uint8_t joint,uint8_t enable)
{
	if(enable==ec_slave_data[joint].status.enabled)
	{
		//请求状态和当前状态相同。
		return;
	}
	else if(1 == enable)
	{
		ec_slave_data[joint].enable = 1;
		ec_slave_data[joint].disable= 0;
	}
	else if(0 == enable)
	{
		ec_slave_data[joint].enable = 0;
		ec_slave_data[joint].disable= 1;
	}
	return;
}
void ec_slave_reset(uint8_t joint)
{
	ec_slave_data[joint].reset = 1;
}
void ec_slave_csp_target_position(uint8_t joint,int32_t target_position)
{
	if(joint >5 || joint<0)
	{
		printf("ec_slave_csp: input joint error %d \n",joint);
		return ;
	}
	ec_slave_data[joint].status.target_position = target_position;
	EC_WRITE_S32(domain1_pd+off_target_position[joint],target_position);
	return ;
}

void ec_slave_status(uint8_t joint,ec_user_statust_t *status)
{
	//从伺服从站收集状态信息。
	uint16_t statusword ;
	int32_t target_position;
	int32_t position_actual_value;		
	statusword = EC_READ_U16(domain1_pd+off_Statusword[joint]);
	if(0x0027 == (statusword&0x6f))
	{
		ec_slave_data[joint].status.enabled = 1;
	}
	else
	{
		ec_slave_data[joint].status.enabled = 0;
	}
	if(0x0008 == (statusword&0x004f))
	{
		ec_slave_data[joint].status.fault = 1;
	}
	else
	{
		ec_slave_data[joint].status.fault = 0;
	}
	
	ec_slave_data[joint].status.target_position = EC_READ_S32(domain1_pd+off_target_position[joint]);
	ec_slave_data[joint].status.actual_position = EC_READ_S32(domain1_pd+off_position_actual_value[joint]);
	ec_slave_data[joint].status.status_word = statusword;
	*status = ec_slave_data[joint].status;
	return;
}

extern void ec_call_back(void)
{
struct timespec wakeupTime, time;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
	
	int i=0;
	for(i=0;i<6;i++)
	{
		//初始化操作模式为csp模式	
		EC_WRITE_U8(domain1_pd + off_operation_mode[i], 0x08);
		//初始化目标为值为当前位置
	}

	while(1) {
		wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif

		// receive process data
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
	

		// check process data state (optional)
		check_domain1_state();



		if (counter) {
			counter--;
		} else { // do this at 1 Hz
			counter = FREQUENCY;

			// check for master state (optional)
			check_master_state();

#ifdef MEASURE_TIMING
            // output timing stats
		for(i=0;i<6;i++)
		{
			printf("joint[%d].actual_value = %d \n",i,EC_READ_S32(domain1_pd+off_position_actual_value[i]));	
			printf("joint[%d].target_position = %d \n",i,EC_READ_S32(domain1_pd+off_target_position[i]));	
		}
		
		//    printf("period     %10u ... %10u\n",
        //           period_min_ns, period_max_ns);
        //    printf("exec       %10u ... %10u\n",
        //            exec_min_ns, exec_max_ns);
        //    printf("latency    %10u ... %10u\n",
        //            latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif

		}

		// write process data




		CTRL_Callback(10000);
		enalbe_state_machine();
		reset_state_machine();
    
		// write application time to master
		clock_gettime(CLOCK_TO_USE, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));

		if (sync_ref_counter) {
			sync_ref_counter--;
		} else {
			sync_ref_counter = 1; // sync every cycle
			ecrt_master_sync_reference_clock(master);
		}
		ecrt_master_sync_slave_clocks(master);

		// send process data



		ecrt_domain_queue(domain1);
		ecrt_master_send(master);
		

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
	}
}


