/*
 * Buffer.h - Simple ring buffer for serial I/O
 *
 *  Created on: Nov 10, 2014
 *
 *  Copyright (C) 2014-2015  TiZed
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <p32xxxx.h>
#include <stdint.h>

typedef struct {
	unsigned int * data  ;
	volatile unsigned int head ;
	volatile unsigned int tail ;
	const unsigned int size ;
} Buffer ;

#define BUFFER(NAME, SIZE) \
    unsigned int NAME ## _da[SIZE] ; \
    Buffer NAME = {NAME ## _da, 0, 0, SIZE} ; \

static inline unsigned int __attribute__((always_inline)) isEmpty(Buffer * buffer)
{
    return(buffer->head == buffer->tail) ;
}

static inline unsigned int __attribute__((always_inline)) isFull(Buffer * buffer)
{
    return((buffer->head + 1) % buffer->size == buffer->tail) ;
}

static inline void __attribute__((always_inline)) push(Buffer * buffer, unsigned int value) {
    if(isFull(buffer)) return ;
    buffer->data[buffer->head] = value ;
    buffer->head = (buffer->head + 1) % buffer->size ;
}

static inline unsigned int __attribute__((always_inline)) pop(Buffer * buffer) {
    if(isEmpty(buffer)) return (0) ;
    unsigned int ret_val = buffer->data[buffer->tail] ;
    buffer->tail = (buffer->tail + 1) % buffer->size ;
    return(ret_val) ;
}

static inline unsigned int __attribute__((always_inline)) pop_wait(Buffer * buffer) {
    while(isEmpty(buffer)) WDTCONSET = 1 ;
    unsigned int ret_val = buffer->data[buffer->tail] ;
    buffer->tail = (buffer->tail + 1) % buffer->size ;
    return(ret_val) ;
}

unsigned int peek(Buffer * buffer) ;
void clear(Buffer * buffer) ;

#endif /* BUFFER_H_ */
