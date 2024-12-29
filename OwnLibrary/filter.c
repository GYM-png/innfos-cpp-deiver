#include <string.h>
#include "system.h"
#include "filter.h"

/**
 * @brief ringbuffer 初始化
 * @param rb Ringbuffer指针
 * @param buffer 所使用的缓冲区指针
 * @param size 缓冲区所占内存 单位：Byte
 */
static void rb_init(RingBuffer_t * rb, void * buffer, size_t size)
{
    rb->buffer = buffer;
    rb->write_index = 0;
    rb->buffer_size = size;
}


/**
 * @brief ringbuffer 强制写入(循环复写)
 * @param rb Ringbuffer指针
 * @param data 指向需要写入数据的指针
 * @param size 写入数据大小 单位：Byte
 */
static void rb_write_force(RingBuffer_t * rb, void * data, size_t size)
{
    uint32_t remaining = rb->buffer_size - rb->write_index;//计算剩余空间
    if (remaining >= size)//剩余空间大于目标空间就直接拷贝
    {
        memcpy(rb->buffer + rb->write_index, data, size);
        rb->write_index = (rb->write_index + size) % rb->buffer_size;
    }
    else//分两次拷贝
    {
        memcpy(rb->buffer + rb->write_index, data, remaining);
        rb->write_index = (rb->write_index + remaining) % rb->buffer_size;
        memcpy(rb->buffer + rb->write_index, data + remaining, size - remaining);
        rb->write_index = (rb->write_index + size - remaining) % rb->buffer_size;
    }
}

/**
 * @brief 加权均值滤波初始化
 * @param Weight_Filter_t Weight_Filter指针  
 * @param data_buffer 指向存放数据的内存
 * @param weight_bk 指向存放权值的内存，长度需与data_buffer一致
 * @param len 最大数据个数，数据超过len之后将循环复写data_buffer
 */
void wfilter_init(Weight_Filter_t * wf, float * data_buffer, float * weight_bk, uint32_t len)
{
    wf->rb_data = (RingBuffer_t *) pvPortMalloc(sizeof (RingBuffer_t));
    if (wf->rb_data == NULL)
    {
        vPortFree(wf->rb_data);
        log_e("wfilter_init error: malloc failed");
    }
    
    rb_init(wf->rb_data, data_buffer, len * sizeof (float ));
    wf->weight_backup = weight_bk;
    wf->buffer_length = len;
    wf->write_index = 0;
    wf->weight_sum = 0;
    wf->data_num = 0;
    for (uint32_t i = 0; i < len; i++)
    {
        wf->weight_sum += weight_bk[i];
    }
}

/**
 * @brief 添加采集数据
 * @param wf Weight_Filter指针
 * @param data 采集的数据
 */
void wfilter_add_data(Weight_Filter_t * wf, float data)
{
    rb_write_force(wf->rb_data, &data, sizeof (data));
    wf->write_index = (wf->write_index + 1) % wf->buffer_length;
    wf->data_num = (wf->data_num < wf->buffer_length) ? (wf->data_num + 1) : (wf->buffer_length);//记录前几次没有写满data-buffer的数据

}

/**
 * @brief 计算加权平均值
 * @param wf Weight_Filter指针
 * @return 滤波后的结果
 */
float wfilter_cal(Weight_Filter_t * wf)
{
    float sum = 0;
    float data = 0;
    float weight_sum = 0;
    if(wf->data_num < wf->buffer_length)//当databuffer未满时
    {
        for (uint32_t i = 0; i < wf->data_num; i++)
        {
            data = *((float *)wf->rb_data->buffer + i);
            sum += wf->weight_backup[i] * data;
            weight_sum += wf->weight_backup[i];
        }
    }
    else
    {
        weight_sum = wf->weight_sum;
        for(uint32_t i = 0; i < wf->buffer_length; i++)
        {
            data = *((float *)(wf->rb_data->buffer) +((i + wf->write_index) % wf->buffer_length));//根据write_index取出权值对应的数据
            sum += wf->weight_backup[i] * data;
        }
    }
    return sum / weight_sum;
}


