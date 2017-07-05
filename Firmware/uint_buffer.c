/*
 * Buffer.c - Simple ring buffer for serial I/O
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

#include "uint_buffer.h"

unsigned int peek(Buffer * buffer) {
    if(isEmpty(buffer)) return(0) ;
    unsigned int ret_val = buffer->data[buffer->tail] ;
    return(ret_val) ;
}


void clear(Buffer * buffer) {
    buffer->tail = buffer->head = 0 ;
    return ;
}
