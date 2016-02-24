/********************************************************
*
* @file      [ringbuffer.h]
* @author    [True]
* @version   V2.3
* @date      2015-07-06
*
* @brief     机智云 只为智能硬件而生
*            Gizwits Smart Cloud  for Smart Products
*            链接｜增值｜开放｜中立｜安全｜自有｜自由｜生态
*            www.gizwits.com
*
*********************************************************/
#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdlib.h>
#include "GizWits.h"

typedef struct {
    size_t rb_capacity;
    char  *rb_head;
    char  *rb_tail;
//  char rb_buff[256];
    char rb_buff[128]; //MAX_P0_LEN
}RingBuffer;

// RingBuffer* rb_new(size_t capacity);
void        rb_new(RingBuffer* rb);
void        rb_free(RingBuffer *rb);

size_t      rb_capacity(RingBuffer *rb);
size_t      rb_can_read(RingBuffer *rb);
size_t      rb_can_write(RingBuffer *rb);

size_t      rb_read(RingBuffer *rb, void *data, size_t count);
size_t      rb_write(RingBuffer *rb, const void *data, size_t count);

#endif
