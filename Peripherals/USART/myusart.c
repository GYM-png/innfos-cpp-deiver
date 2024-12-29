
#include "myusart.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/**
 * @brief 初始化对应串口号的MDA并开始接收
 * @param uart_dma 串口DMA结构体指针
 * @param huart 串口号
 * @param dma dma号
 * @return 
 */
uint8_t uart_dma_init(uart_t* uart, UART_HandleTypeDef* huart, DMA_HandleTypeDef* dma)
{
    uart->huart = huart;
    uart->dma_t = dma;
    uart->rx_flag = 0;
    uart->rx_len = 0;
    uart->mutex = NULL;
    uart->mutex = xSemaphoreCreateMutex();


    /*开启空闲中断并开启DMA接收*/
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, uart->rx_buffer, UART_RX_LEN_MAX);
    return SUCCESS;
}



/**
 * @brief 串口空闲中断
 *        直接在串口IRQ中调用
 * @param uart_dma 串口DMA结构体指针
 */
void uart_idle_callback(uart_t *uart)
{
    if(__HAL_UART_GET_FLAG(uart->huart, UART_FLAG_IDLE) != RESET)
    {
        /*DMA接收处理*/
        __HAL_UART_CLEAR_IDLEFLAG(uart->huart);
        HAL_UART_DMAStop(uart->huart);
        uart->rx_len = UART_RX_LEN_MAX - __HAL_DMA_GET_COUNTER(uart->dma_t);
        memset(uart->rx_buffer + uart->rx_len, 0, UART_RX_LEN_MAX - uart->rx_len);
        HAL_UART_Receive_DMA(uart->huart, uart->rx_buffer, UART_RX_LEN_MAX);

        /*标志位置1*/
        uart->rx_flag = 1;
    }
}

/**
 * @brief 串口发送 应用层调用
 * @param uart 串口句柄
 * @param data 发送数据
 * @param len 数据长度
 * @return 1-失败 0成功
 */
uint8_t uart_transmit(uart_t *uart, uint8_t* data, uint16_t len)
{
    if(uart->mutex != NULL)
    {
        xSemaphoreTake(uart->mutex, 200);
        HAL_UART_Transmit(uart->huart, data, len, 100);
        xSemaphoreGive(uart->mutex);
    }
    else
    {
        HAL_UART_Transmit(uart->huart, data, len, 100);
    }
    return SUCCESS;
}