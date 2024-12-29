#include "innfos_ne30.h"
#include "sca_protocol.h"
#include "global.h"
#include "string.h"

/* ！！！以下宏定义信息参数请勿修改！！！ */

//INNFOS CAN 通信协议指令
//第一类读取指令
#define R1_Heartbeat			0x00        //握手        
#define R1_Mode					0x55        //查询模式
#define R1_PositionLimitState	0x8B        //查询位置限位使能/使能
#define R1_PowerState			0x2B        //查询执行器使能/使能

//第二类读取指令
#define R2_Voltage				0x45        //查询执行器电压(数据为真实值的2^10倍)
#define R2_Current_Max			0x53        //查询最大电流量程
#define R2_MotorTemp			0x5F        //查询电机温度
#define R2_MotorProtectTemp		0x6C        //查询电机保护温度
#define R2_MotorRecoverTemp		0x6E        //查询电机恢复温度
#define R2_Error				0xFF        //查询执行器的报警信息

//第三类读取指令
#define R3_Current				0x04        //查询执行器电流值(数据乘以满量程为真实电流值)
#define R3_Velocity				0x05        //查询执行器速度值(单位：RPM)
#define R3_Position				0x06        //查询执行器位置值(单位：R)
#define R3_PPMaxVelocity		0x1C        //查询位置梯形曲线最大速度
#define R3_PPMaxAcceleration	0x1D        //查询位置梯形曲线最大加速度
#define R3_PPMaxDeceleration	0x1E        //查询位置梯形曲线最大减速度
#define R3_PVMaxVelocity		0x22        //查询速度梯形曲线的最大速度
#define R3_PVMaxAcceleration	0x23        //查询速度梯形曲线的最加大速度
#define R3_PVMaxDeceleration	0x24        //查询速度梯形曲线的最减大速度
#define R3_CurrentLimit			0x59        //查询电流限制值(数据乘以满量程为真实电流值)
#define R3_VelocityLimit		0x5B        //查询查询速度限制值
#define R3_PositionLimitH		0x85        //查询位置的上限值
#define R3_PositionLimitL		0x86        //查询位置的下限值
#define R3_PositionOffset		0x8A        //查询查询位置偏置值
#define R3_BlockEngy			0x7F        //查询堵转能量

//第四类读取指令
#define R4_CVP					0x94        //读取执行器电路速度位置

//第五类读取指令
#define R5_ShakeHands			0x02        //查询执行器的序列号


//第一类写入命令
#define W1_Mode					0x07
#define W1_CurrentFilterState	0X70
#define W1_VelocityFilterState	0x74
#define W1_PositionFilterState	0x78
#define W1_PositionLimitState	0x8C
#define W1_PowerState			0x2A




//第三类写入命令
#define W3_Current				0x08
#define W3_Velocity				0x09
#define W3_Position				0x0A
#define W3_CurrentFilterP		0x0E
#define W3_CurrentFilterI		0x0F
#define W3_VelocityFilterP		0x10
#define W3_VelocityFilterI		0x11
#define W3_PositionFilterP		0x12
#define W3_PositionFilterI		0x13
#define W3_PositionFilterD		0X14
#define W3_PPMaxVelocity		0x1F
#define W3_PPMaxAcceleration	0x20
#define W3_PPMaxDeceleration	0x21
#define W3_PVMaxVelocity		0x25
#define W3_PVMaxAcceleration	0x26
#define W3_PVMaxDeceleration	0x27
#define W3_CurrentFilterLimitL	0x2E
#define W3_CurrentFilterLimitH	0x2F
#define W3_VelocityFilterLimitL	0x30
#define W3_VelocityFilterLimitH	0x31
#define W3_PositionFilterLimitL	0x32
#define W3_PositionFilterLimitH	0x33
#define W3_CurrentLimit			0x58
#define W3_VelocityLimit		0x5A
#define W3_PositionLimitH		0x83
#define W3_PositionLimitL		0x84
#define W3_HomingValue			0x87
#define W3_PositionOffset		0x89
#define W3_HomingCurrentLimitL	0x90
#define W3_HomingCurrentLimitH	0x91
#define W3_BlockEngy			0x7E

//第四类写入命令
#define W4_ClearError			0xFE
#define W4_ClearHome			0x88
#define W4_Save					0x0D

//第五类写入命令
#define W5_ChangeID				0x3D


//变量缩放值定义
// #define Velocity_Max	6000.0f			//速度最大值，固定为6000RPM（仅作为换算用）
#define BlkEngy_Scal	75.225f			//堵转能量缩放值
#define Profile_Scal	960.0f			//梯形参数缩放值
#define IQ8				256.0f			//2^8
#define IQ10			1024.0f			//2^10
#define IQ24			16777216.0f		//2^24
#define IQ30			1073741824.0f	//2^30

uint8_t Sca_Motor::motor_num = 0;
static Sca_Motor* motor_list[6];

Sca_Motor::Sca_Motor()
{
    motor_list[motor_num] = this;
    motor_num++;
}

/**
 * 获取当前列表中存放的地址
 * @return
 */
Sca_Motor *Sca_Motor::get_instance(uint8_t index)
{
    return motor_list[index];
}

/**
 * 返回当前电机总数
 * @return
 */
uint8_t Sca_Motor::get_num()
{
    return motor_num;
}

uint8_t Sca_Motor::Protocol_Write_1(uint8_t cmd, uint8_t data)
{
    static uint8_t tx_buf[2];

    /* 数据打包格式：
    tx_buf[0]-操作命令 	tx_buf[1]-数据（高位）至 tx_buf[7]-数据（低位） */
    tx_buf[0] = cmd;
    tx_buf[1] = data;

    /* 调用底层通信函数传输数据，若出现通信错误则返回错误值 */
    return can_obj->send(id, tx_buf, 2);
}

uint8_t Sca_Motor::Protocol_Write_3(uint8_t cmd, float data)
{
    static uint8_t tx_buf[5];
    int32_t temp;

    /*	速度与电流在设定时，要采用标值，
        即设定值除以该参数的最大值，再转换为IQ24格式	*/
    if((cmd == W3_Velocity)||(cmd == W3_VelocityLimit))
        temp = data / Velocity_MaxRange * IQ24;
    else if((cmd == W3_Current)||(cmd == W3_CurrentLimit))
        temp = data / Current_MaxRange * IQ24;
    else if(cmd == W3_BlockEngy)
        temp = data * BlkEngy_Scal;	//堵转能量为真实值的75.225倍
    else
        temp = data * IQ24;

    tx_buf[0] = cmd;
    tx_buf[1] = (uint8_t)(temp>>24);
    tx_buf[2] = (uint8_t)(temp>>16);
    tx_buf[3] = (uint8_t)(temp>>8);
    tx_buf[4] = (uint8_t)(temp>>0);

    return can_obj->send(id, tx_buf, 5);
}

uint8_t Sca_Motor::Protocol_Write_4(uint8_t cmd)
{
	static uint8_t tx_buf[1];

	tx_buf[0] = cmd;

    return can_obj->send(id, tx_buf, 1);
}

uint8_t Sca_Motor::Protocol_Write_5(uint8_t cmd, uint8_t data, uint8_t * serial_num)
{
	static uint8_t tx_buf[8];
	tx_buf[0] = cmd;
	memcpy(tx_buf + 1, serial_num, 6);
	tx_buf[7] = data;
    return can_obj->send(id, tx_buf, 8);
}

uint8_t Sca_Motor::Protocol_Read(uint8_t cmd)
{
	static uint8_t tx_buf[1];
	tx_buf[0] = cmd;
    return can_obj->send(id, tx_buf, 1);
}

void Sca_Motor::R1dataProcess(void)
{
	/* 将读取结果装载到接收地址中 */
	switch(can_obj->rx_data[0])
	{
		case R1_Heartbeat:
			Online_State = Actr_Enable;
			break;
			
		case R1_Mode:
			Mode = can_obj->rx_data[1];
			break;
			
		case R1_PositionLimitState:
			Position_Limit_State = can_obj->rx_data[1];
			break;
		
		case R1_PowerState: 
			Power_State = can_obj->rx_data[1];	
			break;
			
		default:
			break;
	}
}

void Sca_Motor::R2dataProcess(void)
{
	int16_t temp;
	float RxData;
	
	/* 第二类读写命令为IQ8格式 */
	temp  = ((int16_t)can_obj->rx_data[1])<<8;
	temp |= ((int16_t)can_obj->rx_data[2])<<0;
	
	/* 在第二类读写命令中，电压数据为IQ10格式 */
	if(can_obj->rx_data[0] == R2_Voltage)					
		RxData = (uint16_t)temp / IQ10;
	else
		RxData = (float)temp / IQ8;

	switch(can_obj->rx_data[0])
	{
		case R2_Voltage:
			Voltage = RxData;
			break;
				
		case R2_MotorTemp:
			Motor_Temp  = RxData;
			break;
				
		case R2_MotorProtectTemp:
			Motor_Protect_Temp = RxData;
			break;
		
		case R2_MotorRecoverTemp:
			Motor_Recover_Temp = RxData;
			break;
		
		case R2_Current_Max:
		    Current_MaxRange = RxData;
		
		case R2_Error:
			Error_Code = (uint16_t)RxData;
			// ptErrcode->Error_Code = (uint16_t)RxData;
			// warnBitAnaly(ptErrcode);
			break;

		default:
			break;
	}
}

void Sca_Motor::R3dataProcess(void)
{
	int32_t temp;
	float RxData;

	/* 第三类读写命令以IQ24格式传输 */
	temp  = ((int32_t)can_obj->rx_data[1])<<24;
	temp |= ((int32_t)can_obj->rx_data[2])<<16;
	temp |= ((int32_t)can_obj->rx_data[3])<<8;
	temp |= ((int32_t)can_obj->rx_data[4])<<0;

	/* 速度和电流使用标值，需要将转换值乘以该参数的最大值得到实际值 */
	if((can_obj->rx_data[0] == R3_Velocity)||(can_obj->rx_data[0] == R3_VelocityLimit))
		RxData = (float)temp / IQ24 * Velocity_MaxRange; 
	
	else if((can_obj->rx_data[0] == R3_Current)||(can_obj->rx_data[0] == R3_CurrentLimit))
		RxData = (float)temp / IQ24 * Current_MaxRange; 
	
	else if(can_obj->rx_data[0] == R3_BlockEngy)
		RxData = (float)temp / BlkEngy_Scal; 	//堵转能量为真实的75.225倍
		
	else
		RxData = (float)temp / IQ24; 

	switch(can_obj->rx_data[0])
	{
		case R3_Current:
			Current_Real = RxData;
			break;
		
		case R3_Velocity:	
			Velocity_Real = RxData / ReductionRatio;
			break;
		
		case R3_Position:	
			Position_Real = RxData / ReductionRatio;
			break;
				
		case R3_PPMaxVelocity:
			PP_Max_Velocity = RxData * Profile_Scal / ReductionRatio;
			break;
		
		case R3_PPMaxAcceleration:
			PP_Max_Acceleration = RxData * Profile_Scal / ReductionRatio;
			break;
		
		case R3_PPMaxDeceleration:
			PP_Max_Deceleration = RxData * Profile_Scal / ReductionRatio;
			break;
		
		case R3_PVMaxVelocity:
			PV_Max_Velocity = RxData * Profile_Scal / ReductionRatio;
			break;
		
		case R3_PVMaxAcceleration:
			PV_Max_Acceleration = RxData * Profile_Scal / ReductionRatio;
			break;
		
		case R3_PVMaxDeceleration:
			PV_Max_Deceleration = RxData * Profile_Scal / ReductionRatio;
			break;
				
		case R3_CurrentLimit:	
			Current_Limit = RxData;
			break;
		
		case R3_VelocityLimit:
			Velocity_Limit = RxData / ReductionRatio;
			break;
		
		case R3_PositionLimitH:
			Position_Limit_H = RxData / ReductionRatio;
			break;
		
		case R3_PositionLimitL:
			Position_Limit_L = RxData / ReductionRatio;
			break;
		
		case R3_PositionOffset:
			Position_Offset = RxData / ReductionRatio;
			break;
				
		case R3_BlockEngy:
			Blocked_Energy = RxData;
			break;

		default:
			break;
	}
}

void Sca_Motor::R4dataProcess(void)
{
	int32_t temp;	

	/*	在三环读取协议中，为了使速度、电流、位置数据在同一数据帧中表示出
		将电流和速度值以IQ14格式传输，将位置值以IQ16格式传输。为了方便符
		号位的计算，将位置值向左移8位对齐符号位，转而除以IQ24得到真实值；
		同理，将电流和速度值左移16位对齐符号位，转而除以IQ30得到真实值	。	*/
	
	temp  = ((int32_t)can_obj->rx_data[1])<<24;
	temp |= ((int32_t)can_obj->rx_data[2])<<16;
	temp |= ((int32_t)can_obj->rx_data[3])<<8;
	Position_Real = (float)temp / IQ24  / ReductionRatio;

	temp  = ((int32_t)can_obj->rx_data[4])<<24;
	temp |= ((int32_t)can_obj->rx_data[5])<<16;
	Velocity_Real = (float)temp / IQ30 * Velocity_MaxRange  / ReductionRatio;

	temp  = ((int32_t)can_obj->rx_data[6])<<24;
	temp |= ((int32_t)can_obj->rx_data[7])<<16;
	Current_Real  = (float)temp / IQ30 * Current_MaxRange; 
	
}
void Sca_Motor::R5dataProcess(void)
{
	/* 装填序列号 */
	memcpy(Serial_Num, can_obj->rx_data + 1, 6);
}


void Sca_Motor::InitModelParameters(sca_model_e model)
{
    switch (model)
    {
    case NE30:
        ReductionRatio = 1.0f;
        Velocity_MaxRange = 6000.0f;
        break;
    
    case Lite_NE30_36:
        ReductionRatio = 36.0f;
        Velocity_MaxRange = 6000.0f;
        break;

    default:
        break;
    }
}


/**
 * @brief Enables the SCA motor.
 *
 * This function sends a command to the SCA motor to enable its operation.
 * The motor will start responding to control commands after it is enabled.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the enable command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::Enable(void)
{
    return Protocol_Write_1(W1_PowerState, Actr_Enable);
}


/**
 * @brief Disables the SCA motor.
 *
 * This function sends a command to the SCA motor to disable it.
 * The motor will stop responding to control commands and enter a low-power state.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::Disable(void)
{
    return Protocol_Write_1(W1_PowerState, Actr_Disable);
}


/**
 * @brief Retrieves the state of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its power state.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the heartbeat is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetState(void)
{
    return Protocol_Read(R1_PowerState);
}


/**
 * @brief Sends a heartbeat command to the SCA motor to check its status.
 *
 * This function sends a command to shake hands with the motor to check its connection status
 * The motor will reply with a bit 0x00
 * 
 * @param sca Pointer to the t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the heartbeat is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::HeartBeat(void)
{
    return Protocol_Read(R1_Heartbeat);
}


/**
 * @brief Sets the mode of the SCA motor.
 *
 * This function sends a command to the SCA motor to set its mode.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 * @param mode Desired mode of the SCA motor. @ref ne30WorkMode in innfos_ne30.h
 *
 * @return Returns SUCCESS if the mode is successfully set.
 *         Returns ERROR if the sca pointer is NULL or if the mode setting fails.
 *         The function will retry setting the mode up to 3 times with a delay of 1 second between attempts.
 */
uint8_t Sca_Motor::SetMode(uint8_t mode)
{
    if (Mode == mode)
        return SUCCESS;

    uint8_t result = ERROR;
    uint8_t err_t= 0;
    while(Mode != mode)
    {
        err_t++;
        Protocol_Write_1(W1_Mode, mode);
        GetMode();
        if(err_t >= 3)
        {
            log_e("sca%d 模式设置失败 ", id);
            return ERROR;
        }
        vTaskDelay(1000);
    }
    log_i("sca %d is working in %s", id, find_name_of_mode(Mode));
    return SUCCESS;
}



/**
 * @brief Retrieves the mode of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its mode.
 * The mode is represented by a single byte value. @ref ne30WorkMode in innfos_ne30.h
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 *
 * @return Returns the mode value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
int8_t Sca_Motor::GetMode(void)
{
    return Protocol_Read(R1_Mode);
}


/**
 * @brief Retrieves the serial number of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its serial number.
 * The serial number is a 6 bytes value.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetSerialNumbe(void)
{
    return Protocol_Read(R5_ShakeHands);
}



/**
 * @brief Sets the ID of the SCA motor.
 *
 * This function sends a command to the SCA motor to change its ID.
 * Before changing the ID, it checks if the serial number of the SCA motor is available.
 * If the serial number is not available, it logs an error message and returns ERROR.
 * After changing the ID, it updates the ID value in the t structure.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 * @param id The new ID to be set for the SCA motor.
 *
 * @return Returns SUCCESS if the ID is successfully set.
 *         Returns ERROR if the sca pointer is NULL or the serial number is not available.
 *         If the ID setting fails, it logs a warning message.
 */
uint8_t Sca_Motor::SetID(uint8_t id)
{
    /* 判断是否查询到序列号 */
    uint16_t num = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        num += Serial_Num[i];
    }
    if (num == 0)
    {
        log_e("sca%d 序列号为空 ", id);
        return ERROR;
    }
    

    uint8_t result = ERROR;
    uint8_t * serial_num = (uint8_t*)pvPortMalloc(sizeof(uint8_t) * 6);
    if (serial_num == NULL)
    {
        log_e("内存分配失败 ");
        vPortFree(serial_num);
    }
    memcpy(serial_num, Serial_Num, 6);
    result = Protocol_Write_5(W5_ChangeID, id, serial_num);
    vPortFree(serial_num);
    if (result == SUCCESS)
        id = id;
    else
        log_w("sca%d ID 设置失败 ", id);
    return result;
}


/*********************************Position******************************************** */


/**
 * @brief Sets the target position for the SCA motor.
 *
 * This function sends a command to the SCA motor to set its target position.
 * The target position is a float value representing the desired position in
 * encoder counts. The function checks if the target position is within the
 * valid range and logs a warning if it is not. It also checks if the SCA motor
 * is in the correct mode to set the position and logs a warning if the mode is
 * not appropriate.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL or if the mode is not
 *         appropriate.
 */
uint8_t Sca_Motor::SetPosition(void)
{
    if((Position_Target * ReductionRatio) < -127.0f || 
       (Position_Target * ReductionRatio) > 127.0f)
        log_w("sca%d 参数超范围 当前参数%.2fR", id, Position_Target);

    if (Mode == SCA_Position_Mode || Mode == SCA_Profile_Position_Mode)
    {
        return Protocol_Write_3(W3_Position, Position_Target * ReductionRatio);
    }

    log_w("sca%d 模式不匹配 ", id);
    return ERROR;
}


uint8_t Sca_Motor::SetPosition(float position)
{
    if((position * ReductionRatio) < -127.0f ||
       (position * ReductionRatio) > 127.0f)
        log_w("sca%d 参数超范围 当前参数%.2fR", id, position);

    if (Mode == SCA_Position_Mode || Mode == SCA_Profile_Position_Mode)
    {
        return Protocol_Write_3(W3_Position, position * ReductionRatio);
    }

    log_w("sca%d 模式不匹配 ", id);
    return ERROR;
}


/**
 * @brief Sets the target angle for the SCA motor.
 *
 * This function converts the target angle from degrees to revolutions,
 * checks if the angle is within the valid range for the SCA motor,
 * and then sends a command to the SCA motor to set its target position.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL or the angle is out of range.
 *         If the SCA motor is not in position mode, it logs a warning message and returns ERROR.
 */
uint8_t Sca_Motor::SetAngle(void)
{
    float position = Angle_Target / 360.0f * ReductionRatio;

    if(position < -127.0f || position > 127.0f)
    {
        log_w("sca%d 参数超范围 当前参数:%.2f°" ,id, Angle_Target);
    }
    if (Mode == SCA_Position_Mode || Mode == SCA_Profile_Position_Mode)
    {
        return Protocol_Write_3(W3_Position, position);
    }

    log_w("sca%d  模式不匹配 ", id);
    return ERROR;
}


/**
 * @brief Retrieves the position of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its position.
 * The position is a float value representing the current
 * position of the motor in encoder counts.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetPosition(void)
{
    return Protocol_Read(R3_Position);
}


/**
 * @brief Sets the zero angle for the SCA motor.
 *
 * This function is used to set the zero angle for the SCA motor. It writes the homing value to the motor,
 * which is used to determine the zero position. If the angle provided is 0, the homing value is set to 0.
 * Otherwise, the angle is converted to a position value (0 to 1) and written to the motor.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param angle The angle to set as the zero angle for the motor.
 *
 * @return Returns SUCCESS if the homing value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetZeroAngle(float angle)
{
    if (angle == 0)
    {
        return Protocol_Write_3(W3_HomingValue, 0);
    }
    else
    {
        float position = angle / 360.0f * ReductionRatio;
        return Protocol_Write_3(W3_HomingValue, position);
    }
}

/**
 * @brief Gets the acceleration for the Profile Position Mode.
 *
 * This function reads the acceleration value from the SCA motor's Profile Position Mode
 * and returns it.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetPPAcceleration(void)
{
    return Protocol_Read(R3_PPMaxAcceleration);
}



/**
 * @brief Retrieves the deceleration limit for the Profile Position Mode of the SCA motor.
 *
 * This function reads the deceleration limit value from the SCA motor s Profile Position Mode.
 * The deceleration limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetPPDeceleration(void)
{
    return Protocol_Read(R3_PPMaxDeceleration);
}


/**
 * @brief Sets the acceleration for the Profile Position Mode.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param acce Desired acceleration value in the unit of Profile_Scal.
 *
 * @return Returns SUCCESS if the acceleration is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetPPAcceleration(float acce)
{
    
    
    uint8_t result = ERROR;
    acce = acce / Profile_Scal * ReductionRatio;

    result = Protocol_Write_3(W3_PPMaxAcceleration, acce);
    if (result != SUCCESS)   
    {
        log_e("sca%d PPAcceleration 写入失败 ", id);
        return result;
    }

    GetPPAcceleration();
    return SUCCESS;
    
}


/**
 * @brief Sets the deceleration for the Profile Position Mode.
 *
 * This function sets the deceleration value for the Profile Position Mode of the SCA motor.
 * The deceleration value is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param dece Desired deceleration value in the unit of Profile_Scal.
 *
 * @return Returns SUCCESS if the deceleration value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetPPDeceleration(float dece)
{
    uint8_t result = ERROR;
    dece = dece / Profile_Scal * ReductionRatio;
    result = Protocol_Write_3(W3_PPMaxDeceleration, dece);
    if (result != SUCCESS)
    {
        log_e("sca%d PPDeceleration 写入失败 ",id);
        return ERROR;
    }
    GetPPDeceleration();
    return SUCCESS; 
}



/**
 * @brief Gets the maximum velocity for the Profile Position Mode.
 *
 * This function reads the maximum velocity value from the SCA motor's Profile Position Mode
 * and returns it. The value is scaled by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetPPMaxVelocity(void)
{
    return Protocol_Read(R3_PPMaxVelocity);
}


/**
 * @brief Sets the maximum velocity for the Profile Position Mode.
 *
 * This function sets the maximum velocity for the Profile Position Mode of the SCA motor.
 * The maximum velocity is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param max_velocity Desired maximum velocity value in the unit of Profile_Scal.
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetPPMaxcVelocity(float max_velocity)
{
    
    
    uint8_t result = ERROR;
    uint8_t err_t = 0;
    max_velocity = max_velocity / Profile_Scal * ReductionRatio;

    result = Protocol_Write_3(W3_PPMaxVelocity, max_velocity);
    if (result != SUCCESS)
    {
        log_e("sca%d PPMaxcVelocity 写速度失败 ", id);
        return result;
    }

    GetPPMaxVelocity();

    return SUCCESS;
}



/*********************************Velocity******************************************** */

/**
 * @brief Sets the target velocity for the SCA motor.
 *
 * This function sends a command to the SCA motor to set its target velocity.
 * The target velocity is a float value representing the desired
 * velocity in revolutions per minute (rpm).
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param velocity_target Desired target velocity in revolutions per minute (rpm).
 *
 * @return Returns SUCCESS if the command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetVelocity(void)
{
    return Protocol_Write_3(W3_Velocity, Velocity_Target * ReductionRatio);
}

uint8_t Sca_Motor::SetVelocity(float velocity)
{
    return Protocol_Write_3(W3_Velocity, velocity * ReductionRatio);
}


/**
 * @brief Retrieves the velocity of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its velocity.
 * The velocity is a float value 
 * velocity in revolutions per minute (rpm).
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetVelocity(void)
{
    
  
    return Protocol_Read(R3_Velocity);
}

/**
 * @brief Retrieves the acceleration limit for the Profile Velocity Mode of the SCA motor.
 *
 * This function reads the acceleration limit value from the SCA motor's Profile Velocity Mode.
 * The acceleration limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the acceleration limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetPVAcceleration(void)
{
    return Protocol_Read(R3_PVMaxAcceleration);
}



/**
 * @brief Retrieves the deceleration limit for the Profile Velocity Mode of the SCA motor.
 *
 * This function reads the deceleration limit value from the SCA motor's Profile Velocity Mode.
 * The deceleration limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the deceleration limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetPVDeceleration(void)
{
    
    
    return Protocol_Read(R3_PVMaxDeceleration);
}

/**
 * @brief Sets the acceleration for the Profile Velocity Mode.
 *
 * This function sets the acceleration value for the Profile Velocity Mode of the SCA motor.
 * The acceleration value is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param acceleration Desired acceleration value in the unit of Profile_Scal.
 *
 * @return Returns SUCCESS if the acceleration value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetPVAcceleration(float acceleration)
{
    uint8_t result = ERROR;
    acceleration = acceleration / Profile_Scal * ReductionRatio;
    result = Protocol_Write_3(W3_PVMaxAcceleration, acceleration);
    if (result!= SUCCESS)
    {
        log_e("sac%d PVAcceleration 写入失败 ", id);
        return result;
    }
    GetPVAcceleration();
    return SUCCESS;
}


/**
 * @brief Sets the deceleration for the Profile Velocity Mode.
 *
 * This function sets the deceleration value for the Profile Velocity Mode of the SCA motor.
 * The deceleration value is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param deceleration Desired deceleration value in the unit of Profile_Scal.
 *
 * @return Returns SUCCESS if the deceleration value is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetPVDeceleration(float deceleration)
{
    uint8_t result = ERROR;
    deceleration = deceleration / Profile_Scal * ReductionRatio;
    result = Protocol_Write_3(W3_PVMaxDeceleration, deceleration); 
    if (result!= SUCCESS)
    {   
        log_e("sca%d PVDeceleration 写入失败 ", id);
        return result;
    } 
    GetPVDeceleration();
    return SUCCESS;
}


/**
 * @brief Retrieves the velocity limit of the SCA motor.
 *
 * This function reads the velocity limit value from the SCA motor's
 * internal registers and returns it.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the velocity limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetVelocityLimit(void)
{
    return Protocol_Read(R3_VelocityLimit);
}

/**
 * @brief Retrieves the maximum velocity limit for the Profile Velocity Mode of the SCA motor.
 *
 * This function reads the maximum velocity limit value from the SCA motor's Profile Velocity Mode.
 * The maximum velocity limit is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns the maximum velocity limit value if successful.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetPVMaxVelocity(void)
{
    return  Protocol_Read(R3_PVMaxVelocity);
}


/**
 * @brief Sets the maximum velocity for the Profile Velocity Mode of the SCA motor.
 *
 * This function sets the maximum velocity for the Profile Velocity Mode of the SCA motor.
 * The maximum velocity is scaled by dividing it by the Profile_Scal value.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 * @param maxVelocity Desired maximum velocity value in the unit of Profile_Scal.
 *
 * @return Returns SUCCESS if the maximum velocity is successfully set.
 *         Returns ERROR if the sca pointer is NULL.
 *         If the write operation fails, it logs an error message and returns the error code.
 */
uint8_t Sca_Motor::SetPVMaxVelocity(float maxVelocity)
{
    maxVelocity = maxVelocity / Profile_Scal * ReductionRatio;
    uint8_t result = ERROR;
    result = Protocol_Write_3(W3_PVMaxVelocity, maxVelocity);
    if (result!= SUCCESS)
    {
        log_e("sca%d PVMaxVelocity 写入失败 ", id);
        return result;
    }
    GetPVMaxVelocity();
    return SUCCESS;
}



/*********************************currents******************************************** */


/**
 * @brief Sets the target current for the SCA motor.
 *
 * This function sends a command to the SCA motor to set its target current.
 * The target current is a float value representing the desired
 * current in amperes. The actual current output of the motor may not match the
 * target current exactly due to internal control algorithms.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SetCurrent(void)
{
    return Protocol_Write_3(W3_Current, Current_Target);
}

uint8_t Sca_Motor::SetCurrent(float current)
{
    
    return Protocol_Write_3(W3_Current, current);
}

/**
 * @brief Retrieves the maximum current maximum range of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its maximum current range.
 * The maximum current limit is a float value representing the maximum current in amperes.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the write command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetCurrentMaxRange(void)
{
    return Protocol_Read(R2_Current_Max);
}

uint8_t Sca_Motor::GetCurrentLimit(void)
{
    return Protocol_Read(R3_CurrentLimit);
    
}

/*********************************others******************************************** */


/**
 * @brief Retrieves the error code from the SCA motor.
 *
 * This function sends a command to the SCA motor to read its error code.
 * The error code is a 16-bit value that represents the status of the motor.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::GetErrorCode(void)
{
    return Protocol_Read(R2_Error);
}


/**
 * @brief Retrieves the CVP value from the SCA motor.
 *
 * This function sends a command to the SCA motor to read its CVP value.
 * The CVP value represents the current, velocity and position value of the motor
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 *         
 */
uint8_t Sca_Motor::GetCVP(void)
{
    return Protocol_Read(R4_CVP);
}


/**
 * @brief Retrieves the voltage of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its voltage.
 * The voltage is a float value representing the voltage in volts.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 *         
 */
uint8_t Sca_Motor::GetVoltage(void)
{
    
    return Protocol_Read(R2_Voltage);
}


/**
 * @brief Retrieves the temperature of the SCA motor.
 *
 * This function sends a command to the SCA motor to read its temperature.
 * The temperature is a float value representing the temperature in degrees Celsius.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the command send successfully.
 *         Returns ERROR if the sca pointer is NULL.
 *         
 */
uint8_t Sca_Motor::GetTemperature(void)
{
    return  Protocol_Read(R2_MotorTemp);
}


/**
 * @brief Retrieves all parameters from the SCA motor.
 *
 * This function sends commands to the SCA motor to retrieve various parameters
 * such as serial number, temperature, voltage, mode, and profile velocity
 * parameters. The function then waits for a short period before sending the
 * next command to ensure that the data is ready.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if all commands are successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 *         
 */
uint8_t Sca_Motor::GetAllparameters(void)
{
    

    GetSerialNumbe();
    vTaskDelay(10);
    GetTemperature();
    vTaskDelay(10);
    GetVoltage();
    vTaskDelay(10);
    GetMode();
    vTaskDelay(10);
    GetPPAcceleration();
    vTaskDelay(10);
    GetPPDeceleration();
    vTaskDelay(10);
    GetPPMaxVelocity();
    vTaskDelay(10);
    GetPVAcceleration();
    vTaskDelay(10);
    GetPVDeceleration();
    vTaskDelay(10);
    GetPVMaxVelocity();
    vTaskDelay(10);
    GetVelocityLimit();
    vTaskDelay(10);
    GetCurrentMaxRange();
    vTaskDelay(10);
    GetCurrentLimit();
    return SUCCESS;
}




/**
 * @brief Saves all parameters of the SCA motor.
 *
 * This function sends a command to the SCA motor to save all its parameters.
 * The parameters are saved to non-volatile memory, so they will remain after
 * a power cycle.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the save command is successfully sent.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::SaveAllParamters(void)
{
    
    
    return Protocol_Write_4(W4_Save);
}


/****************************data process*******************************/


/**
 * @brief Processes the received data from the SCA motor.
 *
 * This function checks the first byte of the received data to determine the type of data,
 * and then calls the appropriate data processing function to handle the data.
 *
 * @param sca Pointer to the SCA_t structure representing the SCA motor.
 *
 * @return Returns SUCCESS if the function completes successfully.
 *         Returns ERROR if the sca pointer is NULL.
 */
uint8_t Sca_Motor::DataProcess(void)
{
    

	switch(can_obj->rx_data[0])
	{
		case R1_Heartbeat:
		case R1_Mode:
		case R1_PositionLimitState:
		case R1_PowerState:
			R1dataProcess();
		    break;

		case R2_Voltage:
		case R2_MotorTemp:
		case R2_MotorProtectTemp:
		case R2_MotorRecoverTemp:
        case R2_Current_Max:
		case R2_Error:
            R2dataProcess();
		    break;

		case R3_Current:
		case R3_Velocity:	
		case R3_Position:	
		case R3_PPMaxVelocity:
		case R3_PPMaxAcceleration:
		case R3_PPMaxDeceleration:
		case R3_PVMaxVelocity:
		case R3_PVMaxAcceleration:
		case R3_PVMaxDeceleration:
		case R3_CurrentLimit:	
		case R3_VelocityLimit:
		case R3_PositionLimitH:
		case R3_PositionLimitL:
		case R3_PositionOffset:
		case R3_BlockEngy:
			R3dataProcess();
            Angle_Real = (Position_Real) * 360.0f;
		    break;

		case R4_CVP:
			R4dataProcess();
            Angle_Real = (Position_Real) * 360.0f;
		    break;

		case R5_ShakeHands:
			R5dataProcess();
		    break;

		default:
			break;
	}
    return 0;
}




/**
 * @brief Initializes the SCA motor.
 *
 * This function initializes the SCA motor with the given parameters.
 * It sets the motor's ID, target mode, CAN interface, and other necessary variables.
 * It then enables the motor, gets its state, sets the target mode, and initializes the model parameters.
 * If the current maximum range cannot be obtained within 20 attempts, it logs an error message and returns an error.
 *
 * @param sca Pointer to the t structure representing the SCA motor.
 * @param model The model of the SCA motor.
 * @param id The ID of the SCA motor.
 * @param workmode The target mode of the SCA motor.
 * @param can Pointer to the CAN interface used for communication with the SCA motor.
 *
 * @return Returns SUCCESS if the initialization is successful.
 *         Returns ERROR if the sca pointer or can pointer is NULL, or if the current maximum range cannot be obtained within 20 attempts.
 */
uint8_t Sca_Motor::Init(sca_model_e model, uint8_t id, uint8_t workmode, Can* can)
{

    if(can == NULL)
        return ERROR;

    uint8_t err_t = 0;
    Sca_Motor::id = id;
    Mode_Target = workmode;
    Mode = 0x00;
    can_obj = can;
    Online_State = 1;
    Power_State = 0;
    Current_MaxRange = 0;
    // ptErrcode->Error_Code = 0x0000;
    Error_Code = 0x00;

    Enable();
    GetState();
    vTaskDelay(10);
    SetMode(Mode_Target);
    InitModelParameters(model);
    while (Current_MaxRange == 0)
    {
        err_t++;
        GetCurrentMaxRange();
        if (err_t >= 20)
        {
            log_e("sca%d电流量程获取失败 ", id);
            return ERROR;
        }
        vTaskDelay(100);
    }
    return SUCCESS;
}


/************************************user code************************************************************* */
extern "C"{

typedef struct
{
    uint8_t id;
    char * name;
}index_name_t;

index_name_t mode_name[] = {
    {0x01, "Current_Mode"},
    {0x02, "Velocity_Mode"},
    {0x03, "Position_Mode"},
    {0x06, "Profile_Position_Mode"},
    {0x07, "Profile_Velocity_Mode"},
    {0x08, "Homing_Mode"},
};

index_name_t error_name[] = {
    {0x00, "over voltage fault"},
    {0x01, "under voltage fault"},
    {0x02, "lock rotor fault"},
    {0x03, "over temperature fault"},
    {0x04, "r/w parameter fault"},
    {0x05, "mul circle count fault"},
    {0x06, "inv temperature sensor fault"},
    {0x07, "can bus fault"},
    {0x08, "motor temperature sensor fault"},
    {0x09, "over step fault"},
    {0x0A, "drv protect fault"},
    {0x0B, "device fault"}
};

char *  find_name_of_mode(uint8_t index)
{
    for (uint8_t i = 0; i < sizeof(mode_name) / sizeof(mode_name[0]); i++)
    {
        if (mode_name[i].id == index)
        {
            return mode_name[i].name;
        }
    }
    return NULL;
}

char * find_name_of_error(uint8_t index)
{
    for (uint8_t i = 0; i < sizeof(error_name) / sizeof(error_name[0]); i++)
    {
        if (error_name[i].id == index)
        {
            return error_name[i].name;
        }
    }
    return NULL;
}


void Sca_Motor::show_sca_t(void)
{
    GetAllparameters();
    vTaskDelay(5);
    log_v("----------motor %d----------", id);
    log_v("work mode:\t\t%s", find_name_of_mode(Mode));
    log_v("reduction ratio rate:\t%.1f", ReductionRatio);
    log_v("target current:\t\t%.2f", Current_Target);
    log_v("target velocity:\t%.2frpm", Velocity_Target);
    log_v("target position:\t%.2fR", Position_Target);
    log_v("real current:\t\t%.2fA", Current_Real);
    log_v("real velocity:\t\t%.2frpm", Velocity_Real);
    log_v("real position:\t\t%.2fR", Position_Real);
    log_v("serial numbe:\t\t%02x %02x %02x %02x %02x %02x", Serial_Num[0], Serial_Num[1],Serial_Num[2],Serial_Num[3],Serial_Num[4], Serial_Num[5]);
    log_v("power voltage:\t\t%.2fV", Voltage);
    log_v("velocity limit:\t\t%.2fRPM", Velocity_Limit);
    log_v("motor temperture:\t%.2f°C", Motor_Temp);
    log_v("pp acceleration:\t%.2f", PP_Max_Acceleration);
    log_v("pp deceleration:\t%.2f", PP_Max_Deceleration);
    log_v("pp max velocity:\t%.2fRPM", PP_Max_Velocity);
    log_v("pv acceleration:\t%.2f", PV_Max_Acceleration);
    log_v("pv deceleration:\t%.2f", PV_Max_Deceleration);
    log_v("pv max velocity:\t%.2fRPM", PV_Max_Velocity);
    log_v("max current range:\t%.2fA", Current_MaxRange);
    log_v("limit current:\t\t%.2fA", Current_Limit);
}

}