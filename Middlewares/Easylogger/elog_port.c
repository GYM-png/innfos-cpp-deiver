/*
 * @Author: GYM-png 480609450@qq.com
 * @Date: 2024-10-20 17:59:27
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-10-26 22:23:17
 * @FilePath: \MDK-ARMd:\warehouse\CmdDebug\CmdDebug\UserCode\EasyLogger\elog_port.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */
 
#include <elog.h>
#include <stdio.h>
#include "myusart.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "global.h"

extern uart_t debug_uart;      //调试串口

static SemaphoreHandle_t log_mutex = NULL;//
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    uart_transmit(&debug_uart, (uint8_t *)&ch, 1);
    return ch;
}

/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    /* add your code here */
    log_mutex = xSemaphoreCreateMutex();

    return result;
}



/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void) {

    /* add your code here */

}

#include <string.h>
char test_buffer[200] = {0};

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size) {
    
    /* add your code here */
    uart_transmit(&debug_uart, log, size);
}

/**
 * output lock
 */
void elog_port_output_lock(void) {
    
    /* add your code here */
    if( NULL != log_mutex )
    {
        xSemaphoreTake(log_mutex,portMAX_DELAY);
    }
}

/**
 * output unlock
 */
void elog_port_output_unlock(void) {
    
    /* add your code here */
    if( NULL != log_mutex )
    {
        xSemaphoreGive(log_mutex);
    }
}

// extern RTC_TimeTypeDef rtc_time;
// extern RTC_DateTypeDef rtc_date;
// extern uint32_t systern_run_time;
// extern uint16_t rtc_ms;//系统时间ms rtc只能提供精确到s


/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void) {
    
    /* add your code here */
    static char cur_system_time[16] = { 0 };

#if (INCLUDE_xTaskGetSchedulerState  == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
    TickType_t tick = xTaskGetTickCount();
    snprintf(cur_system_time, 16, "%02d:%02d:%02d:%03d",rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, rtc_ms);
    return cur_system_time;
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
    }
#endif
	return "";

}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) {
    
    /* add your code here */
    return "";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void) {
    
    /* add your code here */
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
    return pcTaskGetName(xTaskGetCurrentTaskHandle());
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
    }
#endif
    return "";
}

/*******************************************************************************
* Function Name  : static_set_output_log_format
* Description    : 静态设置日志打印格式
* Input          : None
* Output         : None
* Return         : None
* example        ; log_a("Hello EasyLogger!");		//断言
				   log_e("Hello EasyLogger!");		//错误
				   log_w("Hello EasyLogger!");		//警告
				   log_i("Hello EasyLogger!");		//信息
				   log_d("Hello EasyLogger!");		//调试
				   log_v("Hello EasyLogger!");		//详细
*******************************************************************************/

void log_init(void)
{
    /* 初始化 EasyLogger */
    elog_init();

    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TIME | ELOG_FMT_T_INFO);
    elog_set_fmt(ELOG_LVL_WARN, ~ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_DIR));

    elog_set_text_color_enabled(true);
    elog_output_lock_enabled(true);

    /* start EasyLogger */
    elog_start();
}
