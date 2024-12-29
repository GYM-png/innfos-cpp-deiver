#ifndef __CONFIG_H
#define __CONFIG_H


/*FreeRTOS CLI组件命令查找定义*/
#define PARAMETER_SEPARATOR '-'


/*串口调试相关定义*/
#define DEBUG_UART (huart1)
#define DEBUG_UART_DMA_RX (hdma_usart1_rx)

/*串口缓冲区*/
#define UART_RX_LEN_MAX 30//最大接收长度
#define UART_TX_LEN_MAX 50//最大发送长度

/*  */
//#define ERROR   1
//#define OK      0
#endif
