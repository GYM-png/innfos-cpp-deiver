/*
 * @Author: GYM 480609450@qq.com
 * @Date: 2024-08-04 22:17:44
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-09-04 15:45:40
 * @Description: SCA通信协议层
 */


#include "sca_protocol.h"
#include "string.h"


/**
  * @功	能	识别错误代码中的具体错误信息
  * @参	数	pSCA：要操作的执行器句柄地址或指针
  * @返	回	无
  */
// static void warnBitAnaly(sca_errcode_t* ptErr)
// {
// 	if(ptErr->Error_Code & 0x0001)
// 		ptErr->WARN_OVER_VOLT = Actr_Enable;
// 	else
// 		ptErr->WARN_OVER_VOLT = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0002)
// 		ptErr->WARN_UNDER_VOLT = Actr_Enable;
// 	else
// 		ptErr->WARN_UNDER_VOLT = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0004)
// 		ptErr->WARN_LOCK_ROTOR = Actr_Enable;
// 	else
// 		ptErr->WARN_LOCK_ROTOR = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0008)
// 		ptErr->WARN_OVER_TEMP = Actr_Enable;
// 	else
// 		ptErr->WARN_OVER_TEMP = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0010)
// 		ptErr->WARN_RW_PARA = Actr_Enable;
// 	else
// 		ptErr->WARN_RW_PARA = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0020)
// 		ptErr->WARN_MUL_CIRCLE = Actr_Enable;
// 	else
// 		ptErr->WARN_MUL_CIRCLE = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0040)
// 		ptErr->WARN_TEMP_SENSOR_INV = Actr_Enable;
// 	else
// 		ptErr->WARN_TEMP_SENSOR_INV = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0080)
// 		ptErr->WARN_CAN_BUS = Actr_Enable;
// 	else
// 		ptErr->WARN_CAN_BUS = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0100)
// 		ptErr->WARN_TEMP_SENSOR_MTR= Actr_Enable;
// 	else
// 		ptErr->WARN_TEMP_SENSOR_MTR = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0200)
// 		ptErr->WARN_OVER_STEP= Actr_Enable;
// 	else
// 		ptErr->WARN_OVER_STEP = Actr_Disable;

// 	if(ptErr->Error_Code & 0x0400)
// 		ptErr->WARN_DRV_PROTEC= Actr_Enable;
// 	else
// 		ptErr->WARN_DRV_PROTEC = Actr_Disable;

// 	if(ptErr->Error_Code & 0xF800)
// 		ptErr->WARN_DEVICE= Actr_Enable;
// 	else
// 		ptErr->WARN_DEVICE = Actr_Disable;
// }
