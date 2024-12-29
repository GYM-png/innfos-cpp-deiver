#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "elog.h"
//#include "cmd.h"
//#include "mymath.h"

#ifdef __cplusplus
extern "C" {
#endif
void debug_init(void);
void system_reset_soft(const char* tag, ...);
#ifdef __cplusplus
}
#endif

#endif
