#include "ne30_task.h"
#include "innfos_ne30.h"
#include "global.h"
#include "system.h"
#include "my_fdcan.h"
#include "filter.h"

#define CURRENT_BUFFER_SIZE 100
Weight_Filter_t current_filter[MOTOR_NUM] = {0};
float *current_buffer[MOTOR_NUM] = {NULL, NULL};
float current_weight[CURRENT_BUFFER_SIZE] = {0};

Sca_Motor motor_A;
Sca_Motor motor_B;
Can fdcan1;

static void current_weight_init(void)
{
    for (uint32_t i = 0; i < CURRENT_BUFFER_SIZE; i++)
    {
        current_weight[i] = i *0.1;
    }
}


static void motor_init(void)
{
    static uint8_t err_t[MOTOR_NUM] = {0};

    /* 电机1初始化 */
    while (motor_A.Power_State != 1)
    {
        err_t[0]++;
        motor_A.Enable();
        motor_A.GetState();
        vTaskDelay(500);
        if (err_t[0] >= 5){//3s后报警一次
            log_e("sca %d is not online", motor_A.id);
            return;
        }
    }
    log_i("sca%d power on and work in %s", motor_A.id, find_name_of_mode(motor_A.Mode));
    motor_A.SetMode(SCA_Profile_Velocity_Mode);
    motor_A.GetAllparameters();

    /* 电机2初始化 */
    while (motor_B.Power_State != 1)
    {
        err_t[1]++;
        motor_B.Enable();
        motor_B.GetState();
        vTaskDelay(500);
        if (err_t[1] >= 5){//3s后报警一次
            log_e("sca %d is not online", motor_B.id);
            return;
        }
    }
    log_i("sca%d power on and work in %s", motor_B.id, find_name_of_mode(motor_B.Mode));
    motor_B.GetCVP();
    vTaskDelay(10);
    motor_B.SetPPMaxcVelocity(2000);
    motor_B.SetMode(SCA_Profile_Position_Mode);
    motor_B.SetPosition(0.00);
    vTaskDelay(100);
    motor_B.SetMode(SCA_Profile_Velocity_Mode);
    motor_B.SetPPMaxcVelocity(30);
    motor_B.GetAllparameters();
}

/**
 * @brief 切换到期望模式
 * @param sca 
 */
static void motor_mode_update(Sca_Motor &sca)
{
    if(sca.Mode_Target != sca.Mode)
    {
        sca.SetMode(sca.Mode_Target);
    }
}

/**
 * @brief 根据当前模式进行控制
 * @param sca 
 */
static void motor_control_update(Sca_Motor &sca)
{
    switch ( sca.Mode)
    {
    case SCA_Current_Mode:
        sca.SetCurrent();
        break;
    case SCA_Profile_Position_Mode:
        sca.SetPosition();
        break;
    case SCA_Profile_Velocity_Mode:
        sca.SetVelocity();
        break;
    default:
        break;
    }
}

/**
 * @brief 单个电机停止
 * @param sca 
 */
static void motor_stop(Sca_Motor &sca)
{
    sca.Disable();
}

/**
 * @brief 所有电机停止
 */
static void motor_stop_all(void)
{
    motor_A.Disable();
    motor_B.Disable();
}

/**
 * @brief 电机掉线检查
 * @param  
 */
static void motor_online_check(void)
{
    static uint8_t err_t = 0;
    static Sca_Motor* instance = NULL;
    MOTOR_TRAVERSE_BEGIN(instance)
    {
        if (instance->Online_State == 0) {
            err_t++;
        } else {
            err_t = 0;
            instance->Online_State = 0;//重置心跳
            instance->HeartBeat();//获取心跳
        }

        if (err_t >= 3) {
            log_e("sca%d is not online", instance->id);
            motor_stop_all();
            system_reset_soft("motor %d is not online", instance->id);
        }
    }
    MOTOR_TRAVERSE_END
}

static void motor_error_check(void)
{
    static Sca_Motor* instance = NULL;
    MOTOR_TRAVERSE_BEGIN(instance)
    {
        instance->GetErrorCode();
    }
    MOTOR_TRAVERSE_END
}
extern "C" {  // 防止名字修饰，按 C 风格链接
/**
 * @brief sca control task function
 * @param pvparameters 
 */
static void sca_control_task(void * pvparameters)
{
    motor_A.Init(Lite_NE30_36, 0X15, SCA_Profile_Position_Mode, &fdcan1);//sca 句柄初始化
    motor_B.Init(NE30,         0x02, SCA_Current_Mode, &fdcan1);
    motor_init();//电机初始化
    current_buffer[0] = (float *)pvPortMalloc(sizeof(float) * CURRENT_BUFFER_SIZE);
    current_buffer[1] = (float *)pvPortMalloc(sizeof(float) * CURRENT_BUFFER_SIZE);
    current_weight_init();
//    wfilter_init(&current_filter[0], current_buffer[0], current_weight, CURRENT_BUFFER_SIZE);
//    wfilter_init(&current_filter[1], current_buffer[1], current_weight, CURRENT_BUFFER_SIZE);
    float current_cal[MOTOR_NUM] = {0, 0};//滤波后的电流值
    log_i("SCA Task Work! ");  
    for(;;)
    {       
        motor_mode_update(motor_A);//先切换到正确模式
        motor_mode_update(motor_B);
        motor_control_update(motor_A);//根据模式执行控制
        motor_control_update(motor_B);
        motor_A.GetCVP();
        motor_B.GetCVP();
        if (motor_B.Position_Real > 3.00)
        {
            motor_B.SetCurrent(3.00);
        }
//        wfilter_add_data(&current_filter[0], motor_A.Current_Real);//添加采集数据
//        wfilter_add_data(&current_filter[1], motor_B.Current_Real);
//        current_cal[0] = wfilter_cal(&current_filter[0]);//计算滤波值
//        current_cal[1] = wfilter_cal(&current_filter[1]);
//        usb_cdc_println("%.2f, %.2f, %.2f, %.2f", current_cal[0], current_cal[0], motor_A.Position_Real, motor_B.Position_Real);
//        motor_A.Position_Target = motor_B.Position_Real;
//        motor_B.Current_Target = motor_A.Current_Real * 0.7;
        vTaskDelay(10);
    }
}

/**
 * @brief sca data process task function
 * @param pvparameters 
 */
void sca_data_process_task(void *pvparameters)
{
    for(;;)
    {
        xSemaphoreTake(*(fdcan1.rx_sema), portMAX_DELAY);
        fdcan1.receive();
        static Sca_Motor* instance = NULL;
        MOTOR_TRAVERSE_BEGIN(instance)
        {
          if (fdcan1.rx_header_t->Identifier == instance->id) {
                instance->DataProcess();
                break;
            }
        }
        MOTOR_TRAVERSE_END
    }
}

/**
 * @brief 1s callback
 * @param xTimer 
 */
void sca_tim_callback(TimerHandle_t xTimer)
{
//    motor_online_check();
    // motor_error_check();
}


/**
 * @defgroup sca control task相关参数
 */
#define SCA_CONTROL_TASK_PRIO 3  //任务优先级
#define SCA_CONTROL_TASK_SIZE 256 //任务堆栈大小
TaskHandle_t SCA_CONTROL_TASK_Handler;//任务句柄
void sca_control_task(void *pvparameters);

/**
 * @defgroup sca data process task相关参数
 */
#define SCA_DATA_PROCESS_TASK_PRIO 5
#define SCA_DATA_PROCESS_TASK_SIZE 256
TaskHandle_t SCA_DATA_PROCESS_TASK_Handler;
void sca_data_process_task(void *pvparameters);

/**
 * @defgroup sca timer 
 */
TimerHandle_t SCA_TIMER;
void sca_tim_callback(TimerHandle_t xTimer);

/**
 * @brief sca task initialization
 */


void sca_task_init(void)
{
    fdcan1.init(&hfdcan1);
    xTaskCreate((TaskFunction_t )sca_control_task,           //任务函数
                (const char *   )"sca control",              //任务名称
                (uint16_t       )SCA_CONTROL_TASK_SIZE,      //任务堆栈大小
                (void *         )NULL,                       //任务参数
                (UBaseType_t    )SCA_CONTROL_TASK_PRIO,      //任务优先级
                (TaskHandle_t * )&SCA_CONTROL_TASK_Handler); //任务句句柄

    xTaskCreate((TaskFunction_t )sca_data_process_task,      //任务函数
                (const char *   )"sca data process",         //任务名称
                (uint16_t       )SCA_DATA_PROCESS_TASK_SIZE, //任务堆栈大小
                (void *         )NULL,                       //任务参数
                (UBaseType_t    )SCA_DATA_PROCESS_TASK_PRIO, //任务优先级
                (TaskHandle_t * )&SCA_CONTROL_TASK_Handler); //任务句句柄

    SCA_TIMER = xTimerCreate((const char* const)"heartbeat",
                             (TickType_t       )1000,
                             (const UBaseType_t)pdTRUE,
                             (void * const     )NULL,
                             (TimerCallbackFunction_t)sca_tim_callback);
    xTimerStart(SCA_TIMER, 0);
}
}