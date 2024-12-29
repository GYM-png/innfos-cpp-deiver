#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "main.h"
//#include "rtc.h"
#include "system.h"



#ifdef __cplusplus
extern "C" {
#endif



typedef struct
{
	uint16_t Hours;
	uint16_t Minutes;
	uint16_t Seconds;
}Rtc_Time_t;

typedef struct
{
	uint16_t Year;
	uint16_t Month;
	uint16_t Date;
}Rtc_Date_t;



#define MOTOR_NUM  2
//Sca_Motor motor_A;
//Sca_Motor motor_B;

void start_task_init(void);


extern uint32_t system_run_time;
extern uint16_t rtc_ms;

extern Rtc_Time_t rtc_time;
extern Rtc_Date_t rtc_date;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif
