#include "my_fdcan.h"



uint8_t Can::peripherals_number = 0;
SemaphoreHandle_t FDCAN1_Rx_Sema= NULL;
SemaphoreHandle_t FDCAN2_Rx_Sema= NULL;

Can::Can()
{
	peripherals_number++;
	if(peripherals_number == 1)
	{
		FDCAN1_Rx_Sema = xSemaphoreCreateBinary();
		rx_sema = &FDCAN1_Rx_Sema;
	}
	else if(peripherals_number == 2)
	{
        FDCAN2_Rx_Sema = xSemaphoreCreateBinary();
        rx_sema = &FDCAN2_Rx_Sema;
	}
}

Can::~Can()
{
	peripherals_number--;
}

/**
 * @brief can bus initialization
 * @param can can bus instance
 * @param hcan hal library can handle
 * @return SUCCESS or ERROR
 */
uint8_t Can::init(FDCAN_HandleTypeDef* hcan)
{
	if(hcan == NULL || hcan == NULL)
		return ERROR; 
	hcan_t = hcan;
	mutex = xSemaphoreCreateMutex();
	HAL_FDCAN_Start(hcan_t);
	HAL_FDCAN_ActivateNotification(hcan_t,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	HAL_FDCAN_ActivateNotification(hcan_t,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
    FDCAN_FilterTypeDef * can_filter_config = (FDCAN_FilterTypeDef*)pvPortMalloc(sizeof(FDCAN_FilterTypeDef));
	if (can_filter_config == NULL)		
	{
		vPortFree(can_filter_config);
//		log_e("内存分配失败 CAN初始化失败 ");
		return ERROR;
	}
	
	can_filter_config->IdType = FDCAN_STANDARD_ID;                      //ID mode  FDCAN_STANDARD_ID
    can_filter_config->FilterIndex = 1;                  				//filter index                   
    can_filter_config->FilterType = FDCAN_FILTER_RANGE;        			//filter type :range, mask
    can_filter_config->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;          //FIFOx   FDCAN_FILTER_TO_RXFIFOx
    can_filter_config->FilterID1 = 0x001;                  				//start id
    can_filter_config->FilterID2 = 0xFFF;                  				//end id
    if(HAL_FDCAN_ConfigFilter(hcan_t, can_filter_config) != HAL_OK) 			//can filter config
	{
//		log_e("CAN滤波器配置失败 ");
		Error_Handler();
	}
	vPortFree(can_filter_config);
	return SUCCESS;
}


/**
 * @brief can bus send message
 * @param id message id
 * @param data message data
 * @param dlc message length
 * @return SUCCESS or ERROR
 */
uint8_t Can::send(uint32_t id, uint8_t * data, uint8_t dlc)
{
	uint8_t result = ERROR;
	if(data == NULL)
	    return ERROR;
	FDCAN_TxHeaderTypeDef  tx_header;

	tx_header.Identifier = id;                           	//消息ID
	tx_header.IdType = FDCAN_STANDARD_ID;                  	//消息ID类型
	tx_header.TxFrameType = FDCAN_DATA_FRAME;              	//数据帧格式
	tx_header.DataLength = dlc;                         	//发送数据长度
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;            
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;                //FDCAN经典CAN模式
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;                           
	if(xSemaphoreTake(mutex, 100) == pdTRUE)
	{
		result = HAL_FDCAN_AddMessageToTxFifoQ(hcan_t, &tx_header, data);
		xSemaphoreGive(mutex);
		return result;
	}
	return result;	
}

/**
 * @brief can bus receive message
 * @param can can bus instance
 * @return SUCCESS or ERROR 
 */
uint8_t Can::receive(void)
{
	uint8_t result = ERROR;
	result = HAL_FDCAN_GetRxMessage(hcan_t, FDCAN_RX_FIFO0, rx_header_t, rx_data);
	return result;
}

/********************************IRQ********************************************************* */
extern "C"{
/**
 * @brief can bus fifo0 irq callback
 * @param hcan can instacne
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan->Instance == FDCAN1)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(FDCAN1_Rx_Sema, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
}