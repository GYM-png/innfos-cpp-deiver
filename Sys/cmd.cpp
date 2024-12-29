#include "cmd.h"
#include <stdio.h>
#include <stdarg.h> // 引入可变参数头文件
#include <string.h>
#include <stdlib.h>
#include "global.h"
#include "system.h"
#include "FreeRTOS_CLI.h"
SemaphoreHandle_t write_mutex = NULL;
static char write_buffer[200] = {0};
static void myprintf(const char*__format, ...);
static uint8_t find_parameters(const char *pcParameter, const uint8_t n, char **parameterArr);
#define TAG "cmd"

/*************************************************以下是用户代码******************************************************************** */
#include "innfos_ne30.h"
#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @defgroup 命令函数声明 
 * {
 */
static BaseType_t system_reset(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t get_command(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t set_command( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/**
 * }
 */

/**
 * @defgroup FreeRTOS CLI命令组件 
 * {
 */
static const CLI_Command_Definition_t user_command[] = {
    {
    .pcCommand = "system",\
    .pcHelpString = "system:\t\t-t:get task; -r:reset mcu\r\n", \
    .pxCommandInterpreter = system_reset, \
    .cExpectedNumberOfParameters = 1}, 
    {
    .pcCommand = "get",\
    .pcHelpString = "get:\t\t-t:get time; -vcp:get sca parameters\r\n", \
    .pxCommandInterpreter = get_command, \
    .cExpectedNumberOfParameters = 2},
    {
    .pcCommand = "set",\
    .pcHelpString = "set:\t\t-v:set velocity; -p:set position; -c:set current; -m:set mode\r\n", \
    .pxCommandInterpreter = set_command, \
    .cExpectedNumberOfParameters = 2},
};
/**
 * }
 */


static BaseType_t system_reset(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
static UBaseType_t uxParameterNumber = 1;   //总参数量
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    pcWriteBuffer[0] = '\0';        //清除上次输出残留
    char **parameterArray = (char **)pvPortMalloc(sizeof(char *) * uxParameterNumber);
    if (parameterArray == NULL)
    {
        log_e("内存分配失败 \r\n");
        vPortFree(parameterArray);
        return pdFALSE;
    }
    if (find_parameters(pcCommandString, uxParameterNumber, parameterArray) == 0)
    {
        log_i("没有找到参数 \r\n");
        return pdFALSE;
    }
    if (strstr(parameterArray[0], "t"))
    {
        const char *const pcHeader = "    状态   优先级   堆栈     #\r\n************************************************\r\n";
        BaseType_t xSpacePadding;
        ( void ) pcCommandString;
        ( void ) xWriteBufferLen;
        configASSERT( pcWriteBuffer );
        strcpy( pcWriteBuffer, "任务" );
        pcWriteBuffer += strlen( pcWriteBuffer );
        configASSERT( configMAX_TASK_NAME_LEN > 3 );
        for( xSpacePadding = strlen( "任务" ); xSpacePadding < ( configMAX_TASK_NAME_LEN - 3 ); xSpacePadding++ )
        {
            *pcWriteBuffer = ' ';
            pcWriteBuffer++;
            *pcWriteBuffer = 0x00;
        }
        strcpy( pcWriteBuffer, pcHeader );
        vTaskList( pcWriteBuffer + strlen( pcHeader ) );
        return pdFALSE;
    }
    else if(strstr(parameterArray[0], "r"))
    {
        vTaskDelay(1000); // 1s后重启
        system_reset_soft(TAG);
    }
	return pdFALSE;
}




static BaseType_t get_command( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
static UBaseType_t uxParameterNumber = 2;   //总参数量
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    pcWriteBuffer[0] = '\0';        //清除上次输出残留
    char **parameterArray = (char **)pvPortMalloc(sizeof(char *) * uxParameterNumber);
    if (parameterArray == NULL)
    {
        log_e("内存分配失败 \r\n");
        vPortFree(parameterArray);
        return pdFALSE;
    }
    if (find_parameters(pcCommandString, uxParameterNumber, parameterArray) == 0)
    {
        log_i("没有找到参数 \r\n");
        return pdFALSE;
    }
    
    if (strstr(parameterArray[0], "t"))
    {
        log_v("当前系统时间:\r\n");
        log_v("%04d年%02d月%02d日 %02d:%02d:%02d", rtc_date.Year, rtc_date.Month, rtc_date.Date, rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);
    }
    else if (strstr(parameterArray[0], "vcp"))
    {
        int index = atoi(parameterArray[1]);//第二个参数转换为下标
        if(index >  MOTOR_NUM || index <= 0)
        {
            log_w("参数超出范围 \r\n");
            return pdFALSE;
        }
        log_v("SCA参数:");
        static Sca_Motor* instance = NULL;
        instance = Sca_Motor::get_instance(index - 1);
        log_v("velocity:\t%.2frpm", instance->Velocity_Real);
        log_v("position:\t%.2fR ",  instance->Position_Real);
        log_v("angle:\t\t%.2f° ",   instance->Angle_Real);
        log_v("current:\t%.2f",     instance->Current_Real);
    }
    else if (strstr(parameterArray[0], "s"))
    {
        int index = atoi(parameterArray[1]);//第二个参数转换为下标
        if(index >  MOTOR_NUM || index <= 0)
        {
            log_w("参数超出范围 \r\n");
            return pdFALSE;
        }
        static Sca_Motor* instance = NULL;
        instance = Sca_Motor::get_instance(index - 1);
        instance->show_sca_t();
    }
    
    
	return pdFALSE;
}


static BaseType_t set_command( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
static UBaseType_t uxParameterNumber = 2;   //总参数量
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
    pcWriteBuffer[0] = '\0';        //清除上次输出残留
    char **parameterArray = (char **)pvPortMalloc(sizeof(char *) * uxParameterNumber);
    if (parameterArray == NULL)
    {
        log_e("内存分配失败 \r\n");
        vPortFree(parameterArray);
        return pdFALSE;
    }
    if (find_parameters(pcCommandString, uxParameterNumber, parameterArray) == 0)
    {
        log_i("没有找到参数 \r\n");
        return pdFALSE;
    }
    static Sca_Motor* instance1 = Sca_Motor::get_instance(0);
    static Sca_Motor* instance2 = Sca_Motor::get_instance(0);

    if (strstr(parameterArray[0], "v1")){
        if (instance1->Mode != SCA_Profile_Velocity_Mode){
            log_w("请先切换到速度模式 ");
            return pdFALSE;
        }
        instance1->Velocity_Target = atof(parameterArray[1]);
    }
    else if (strstr(parameterArray[0], "v2")){
        if (instance2->Mode != SCA_Profile_Velocity_Mode){
            log_w("请先切换到速度模式 ");
            return pdFALSE;
        }
        instance2->Velocity_Target = atof(parameterArray[1]);
    }
    else if (strstr(parameterArray[0], "p1")){
        if (instance1->Mode != SCA_Profile_Position_Mode){
            log_w("请先切换到位置模式 ");
            return pdFALSE;
        }
        instance1->Position_Target = atof(parameterArray[1]);
    }
    else if (strstr(parameterArray[0], "p2")){
        if(instance2->Mode!= SCA_Profile_Position_Mode){
            log_w("请先切换到位置模式 ");
            return pdFALSE;
        }
        instance2->Position_Target = atof(parameterArray[1]);
    }
    else if (strstr(parameterArray[0], "c1")){
        if(instance1->Mode!= SCA_Current_Mode){
            log_w("请先切换到电流模式 ");
            return pdFALSE;
        }
        instance1->Current_Target = atof(parameterArray[1]);
    }
    else if (strstr(parameterArray[0], "c2")){
        if(instance2->Mode!= SCA_Current_Mode){
            log_w("请先切换到电流模式 ");
            return pdFALSE;
        }
        instance2->Current_Target = atof(parameterArray[1]);
    }
    else if (strstr(parameterArray[0], "m1")){
        if (strstr(parameterArray[1], "v"))
            instance1->Mode_Target =  SCA_Profile_Velocity_Mode;
        else if (strstr(parameterArray[1], "p"))
            instance1->Mode_Target =  SCA_Profile_Position_Mode;
        else if(strstr(parameterArray[1], "c"))
            instance1->Mode_Target =  SCA_Current_Mode;
    }
    else if (strstr(parameterArray[0], "m2")){
        if (strstr(parameterArray[1], "v"))
            instance2->Mode_Target =  SCA_Profile_Velocity_Mode;
        else if (strstr(parameterArray[1], "p"))
            instance2->Mode_Target =  SCA_Profile_Position_Mode;
        else if(strstr(parameterArray[1], "c"))
            instance2->Mode_Target =  SCA_Current_Mode;
    }
	return pdFALSE;
}


/*************************************************以下是内部调用函数******************************************************************** */

/**
 * @brief 用户命令初始化
 *        在任务调度开始之前使用
 * @param  
 */
void cmd_init(void)
{
    write_mutex = xSemaphoreCreateMutex();
    for (uint8_t i = 0; i < sizeof(user_command)/sizeof(user_command[0]); i++)
    {
        FreeRTOS_CLIRegisterCommand( &user_command[i] );	
    }
}

#ifdef __cplusplus
}
#endif
/**
 * @brief 仅作为命令回调函数内使用的打印函数
 * @param __format 
 * @param  
 */
static void myprintf(const char*__format, ...)
{
    va_list args; // 创建一个va_list类型的变量，用来存储可变参数
    va_start(args, __format); // 使用va_start宏初始化args，使之指向第一个可选参数
    vsnprintf(write_buffer + strlen(write_buffer), sizeof(write_buffer), __format, args); // 使用vsnprintf代替sprintf，因为它可以处理可变参数列表
    va_end(args); // 使args不再指向可变参数列表中的任何参数
    xSemaphoreGive(write_mutex);
}

/**
 * @brief 提取参数放入字符串数组
 * @param pcParameter 整个参数字符串
 * @param n 需要寻找的参数个数
 * @param parameterArr 字符串数组，存放提取出的参数
 * @return 找到的参数个数
 */
static uint8_t find_parameters(const char *pcParameter, const uint8_t n, char **parameterArr)
{
    uint8_t len = strlen(pcParameter);
    uint8_t found_num = 0;
    uint8_t *index = (uint8_t*) pvPortMalloc(sizeof(uint8_t) * (n + 1));  // 增加一个元素用于最后一个参数
    if (index == NULL)
    {
        return 0;
    }

    // 找到参数位置
    for (uint8_t i = 0; i < len - 1; i++)
    {
        if (pcParameter[i] == ' ' && pcParameter[i + 1] == '-')
        {
            index[found_num++] = i + 2;
            if (found_num == n)
                break;
        }
    }

    // 提取处理参数
    for (uint8_t i = 0; i < found_num; i++)
    {
        uint8_t start = index[i];
        uint8_t end = (i + 1 < found_num) ? index[i + 1] - 2: len;  // 确保最后一个参数处理正确
        uint8_t param_len = end - start;

        parameterArr[i] = (char *)pvPortMalloc(param_len + 1);  // 分配内存
        if (parameterArr[i] == NULL)//错误处理
        {
            for (uint8_t j = 0; j < i; j++)
            {
                vPortFree(parameterArr[j]);
            }
            vPortFree(index);
            return 0;
        }
        memcpy(parameterArr[i], pcParameter + start, param_len);
        parameterArr[i][param_len] = '\0';  // 添加字符串结束符 避免乱码
    }

    vPortFree(index);
    return found_num;
}
