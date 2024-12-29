#ifndef __MY_FDCAN_H
#define __MY_FDCAN_H

#include "fdcan.h"
#include "system.h"

#define Actr_Enable 0x01
#define Actr_Disable 0x00


class Can
{
public:
    Can();
    ~Can();
private:
    FDCAN_HandleTypeDef * hcan_t;   // hal library handle
    SemaphoreHandle_t mutex;        //tx mutex semaphore
    static uint8_t peripherals_number;
public:
    SemaphoreHandle_t *rx_sema;      //rx semaphore
    uint8_t rx_data[8];             //rx data buffer
    FDCAN_RxHeaderTypeDef * rx_header_t;
    uint8_t init(FDCAN_HandleTypeDef* hcan);
    uint8_t send(uint32_t id, uint8_t * data, uint8_t dlc);
    uint8_t receive(void);
};



#endif
