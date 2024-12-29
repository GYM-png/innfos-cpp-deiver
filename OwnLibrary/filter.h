#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"

/**
 * @brief  滤波使用的Ringbuffer 仅有循环复写的功能
 */
typedef struct
{
    void * buffer;
    uint32_t write_index;
    uint32_t buffer_size;
}RingBuffer_t;

/**
 * @brief 加权均值滤波
 */
typedef struct
{
    RingBuffer_t * rb_data; //滤波使用的Ringbuffer
    float * weight_backup;  //权值数组
    float weight_sum;       //权值之和
    uint32_t write_index;   //写入位置索引
    uint32_t buffer_length; //数据长度 
    uint32_t data_num;      //databuffer中数据个数
}Weight_Filter_t;

void wfilter_init(Weight_Filter_t * wf, float * data_buffer, float * weight_bk, uint32_t len);
void wfilter_add_data(Weight_Filter_t * wf, float data);
float wfilter_cal(Weight_Filter_t * wf);


#endif /* __FILTER_H */
