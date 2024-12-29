#ifndef INNFOS_NE30_INNFOS_NE30_H
#define INNFOS_NE30_INNFOS_NE30_H

#include "my_fdcan.h"



/**
 * @brief SCA报警信息
 */
typedef struct
{
    uint16_t Error_Code;			//错误代码

    /* 具体报警信息，0：正常，1：出错 */
    uint8_t WARN_OVER_VOLT;  		//过压异常
    uint8_t WARN_UNDER_VOLT;  		//欠压异常
    uint8_t WARN_LOCK_ROTOR;  		//堵转异常
    uint8_t WARN_OVER_TEMP;  		//过热异常
    uint8_t WARN_RW_PARA;  			//读写参数异常
    uint8_t WARN_MUL_CIRCLE;  		//多圈计数异常
    uint8_t WARN_TEMP_SENSOR_INV; 	//逆变器温度传感器异常
    uint8_t WARN_CAN_BUS;  			//CAN通讯异常
    uint8_t WARN_TEMP_SENSOR_MTR;	//电机温度传感器异常
    uint8_t WARN_OVER_STEP;			//位置模式阶跃大于1
    uint8_t WARN_DRV_PROTEC;  		//DRV保护
    uint8_t WARN_DEVICE;  			//设备异常

}sca_errcode_t;

/**
 * @defgroup ne30WorkMode
 */
#define  SCA_Current_Mode			    0x01
#define  SCA_Velocity_Mode			    0x02
#define  SCA_Position_Mode			    0x03
#define  SCA_Profile_Position_Mode	    0X06
#define  SCA_Profile_Velocity_Mode	    0X07
#define  SCA_Homing_Mode				0X08

/**
 * @brief SCA 电机型号
 */
typedef enum
{
	NE30 = 0, 			//
	Lite_NE30_36 = 1, 	//
}sca_model_e;

/**
 * @brief 执行器的各种参数信息
 */
class Sca_Motor
{
public:
    Sca_Motor();
private:
    static uint8_t motor_num;
private:
	/* 通信相关信息 */
    uint8_t Serial_Num[6];			//序列号
    Can * can_obj;                    //can驱动
	/* 不同电机的参数 */
	sca_model_e model;				//电机型号
	float ReductionRatio; 			//减速比					
	float Velocity_MaxRange; 		//转速满量程 (RPM) 速度环使用(初始化赋值，只和电机有关，不能改变)
public:
	/* 目标值 */
	float Current_Target;			//目标电流 (单位：A)
	float Velocity_Target;			//目标速度 (单位：RPM）
	float Position_Target;			//目标位置 (单位：R）
    float Angle_Target;	            //目标角度 (单位：度)
	float Mode_Target;				//目标工作模式

	/* 第三类数据变量 */
	float Current_Real;				//当前电流 (单位：A）
	float Velocity_Real;			//当前速度 (单位：RPM）
	float Position_Real;			//当前位置 (单位：R）
    float Angle_Real;	            //当前角度 (单位：度)
	/* 第一类数据变量 */
	uint8_t Mode;					//当前操作模式 @brief ne30WorkMode
	uint8_t Power_State;			//开关机状态
	uint8_t Online_State;			//在线状态

	uint32_t id;                    //id

private:
	/* 第一类数据变量 */
	uint8_t Position_Limit_State;	//位置限位状态

	/* 第二类数据变量 */
	float Voltage;					//当前电压（单位：V）
	float Motor_Temp;				//电机温度
	float Motor_Protect_Temp;		//电机保护温度
	float Motor_Recover_Temp;		//电机恢复温度
	float Current_MaxRange;			//电流满量程 (A) 电流环使用(初始化获取，只和电机有关，不能改变)
	// sca_errcode_t * ptErrcode;	    //电机报警信息
    uint16_t Error_Code;			//错误代码
	
	/* 第三类数据变量 */
	float PP_Max_Velocity;			//位置梯形速度最大值
	float PP_Max_Acceleration;		//位置梯形加速度最大值
	float PP_Max_Deceleration;		//位置梯形减速度最大值
	float PV_Max_Velocity;			//速度梯形速度最大值
	float PV_Max_Acceleration;		//速度梯形加速度最大值
	float PV_Max_Deceleration;		//速度梯形减速度最大值
	float Position_Filter_Limit_L;	//位置环输出下限
	float Position_Filter_Limit_H;	//位置环输出上限
	float Position_Limit_H;			//执行器的位置上限
	float Position_Limit_L;			//执行器的位置下限
	float Current_Limit;			//电流输入限幅
	float Velocity_Limit;			//速度输入限幅
	float Homing_Value;				//执行器的Homing值
	float Position_Offset;			//执行器的位置偏置
	float Blocked_Energy;			//堵转锁定能量
private:
	uint8_t Protocol_Write_1(uint8_t cmd, uint8_t data);
	uint8_t Protocol_Write_3(uint8_t cmd, float data);
	uint8_t Protocol_Write_4(uint8_t cmd);
	uint8_t Protocol_Write_5(uint8_t cmd, uint8_t data, uint8_t * serial_num);
	uint8_t Protocol_Read(uint8_t cmd);
	void R1dataProcess(void);
	void R2dataProcess(void);
	void R3dataProcess(void);
	void R4dataProcess(void);
	void R5dataProcess(void);
	void InitModelParameters(sca_model_e model);
public:
    static Sca_Motor* get_instance(uint8_t index);
    static uint8_t get_num(void);
	uint8_t Init(sca_model_e model, uint8_t id, uint8_t workmode, Can* can);
	uint8_t Enable(void);
	uint8_t Disable(void);
	uint8_t GetState(void);
	uint8_t HeartBeat(void);
	uint8_t SetMode(uint8_t mod);
	int8_t GetMode(void);
	uint8_t GetSerialNumbe(void);
	uint8_t SetID(uint8_t id);


	uint8_t SetPosition(void);
    uint8_t SetPosition(float position);
	uint8_t SetAngle(void);
	uint8_t GetPosition(void);
	uint8_t SetZeroAngle(float angle);
	uint8_t GetPPAcceleration(void);
	uint8_t GetPPDeceleration(void);
	uint8_t SetPPAcceleration(float acce);
	uint8_t SetPPDeceleration(float dece);
	uint8_t GetPPMaxVelocity(void);
	uint8_t SetPPMaxcVelocity(float max_velocity);


	uint8_t SetVelocity(void);
    uint8_t SetVelocity(float velocity);
	uint8_t GetVelocity(void);
	uint8_t GetPVAcceleration(void);
	uint8_t GetPVDeceleration(void);
	uint8_t SetPVAcceleration(float acceleration);
	uint8_t SetPVDeceleration(float deceleration);
	uint8_t GetVelocityLimit(void);
	uint8_t GetPVMaxVelocity(void);
	uint8_t SetPVMaxVelocity(float maxVelocity);


	uint8_t SetCurrent(void);
	uint8_t SetCurrent(float current);

	uint8_t GetCurrentMaxRange(void);
	uint8_t GetCurrentLimit(void);


	uint8_t GetErrorCode(void);
	uint8_t GetCVP(void);
	uint8_t GetVoltage(void);
	uint8_t GetTemperature(void);
	uint8_t GetAllparameters(void);
	uint8_t SaveAllParamters(void);


	uint8_t DataProcess(void);
	/* UserCode to Show Data */
	void show_sca_t(void);
};

#define MOTOR_TRAVERSE_BEGIN(instance) \
for (uint8_t i = 0; i < Sca_Motor::get_num(); i++) {\
    instance = Sca_Motor::get_instance(i);
#define MOTOR_TRAVERSE_END }


/************************************user code************************************************************* */
extern "C" {
char *find_name_of_mode(uint8_t index);
char *find_name_of_error(uint8_t index);
}

#endif //_INNFOS_NE30_H
