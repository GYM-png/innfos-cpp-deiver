#include "global.h"
#include "system.h"
#include "ne30_task.h"




#ifdef __cplusplus
extern "C" {
#endif

Rtc_Time_t rtc_time =  {0, 0, 0};
Rtc_Date_t rtc_date =  {0, 0, 0};
uint32_t system_run_time = 0;
uint16_t rtc_ms = 0;//系统时间ms rtc只能提供精确到s


/**
 * @defgroup start task相关参数
 */
#define START_TASK_PRIO 3  //任务优先级
#define START_TASK_SIZE 200 //任务堆栈大小
TaskHandle_t START_TASK_Handler;//任务句柄


void start_task(void *pvparameters);
void start_task_init(void)
{
    xTaskCreate((TaskFunction_t )start_task,    //任务函数
                (const char *   )"start",   //任务名称
                (uint16_t       )START_TASK_SIZE, //任务堆栈大小
                (void *         )NULL,                //任务参数
                (UBaseType_t    )START_TASK_PRIO, //任务优先级
                (TaskHandle_t * )&START_TASK_Handler); //任务句句柄
}


void start_task(void * pvparameters)
{
    /*获取系统初始时间*/
    log_i("系统开机成功 ");
    sca_task_init();
    for(;;)
    {
        if(rtc_ms >= 1000)
        {
            rtc_ms = 0;
            rtc_time.Seconds++;
        }
        if(rtc_time.Seconds >= 60)
        {
            rtc_time.Seconds = 0;
            rtc_time.Minutes++;
        }
        if(rtc_time.Minutes >= 60)
        {
            rtc_time.Minutes = 0;
            rtc_time.Hours++;
        }

        // log_i("你好\r\n");
        // log_w("你好\r\n");
        // log_e("你好\r\n");
        vTaskDelay(1002);
    }
}

#ifdef __cplusplus
}
#endif
