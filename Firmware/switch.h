/* 
 * File:   switch.h
 * Author: tized
 *
 * Created on 02 July 2016, 01:15
 */

#ifndef SWITCH_H
#define	SWITCH_H

#include <stdint.h>
#include "axis.h"


#ifdef	__cplusplus
extern "C" {
#endif
    
enum switch_type {
    EMO,
    LIMIT,
    HOME,
    FAULT
};

typedef union {
    struct {
        unsigned int switch_x_home :1 ;     // bit 1
        unsigned int switch_x_limit :1 ;    // bit 2
        unsigned int switch_y_home :1 ;     // bit 3
        unsigned int switch_y_limit :1 ;    // bit 4
        unsigned int switch_z_home :1 ;     // bit 5
        unsigned int switch_z_limit :1 ;    // bit 6
        unsigned int switch_a_home :1 ;     // bit 7
        unsigned int switch_a_limit :1 ;    // bit 8
        unsigned int switch_b_home :1 ;     // bit 9
        unsigned int switch_b_limit :1 ;    // bit 10
        unsigned int switch_c_home :1 ;     // bit 11
        unsigned int switch_c_limit :1 ;    // bit 12
        unsigned int switch_e_home :1 ;     // bit 13
        unsigned int switch_e_limit :1 ;    // bit 14
        unsigned int z_level :1 ;           // bit 15
        unsigned int reserved_sw :6 ;       // bits 16-21
        unsigned int xsum_error :1 ;        // bit 22
        unsigned int switch_emo :1 ;        // bit 23
        unsigned int ucont_fault :1 ;       // bit 24
        unsigned int x_drv_fault :1 ;       // bit 25
        unsigned int y_drv_fault :1 ;       // bit 26
        unsigned int z_drv_fault :1 ;       // bit 27
        unsigned int a_drv_fault :1 ;       // bit 28
        unsigned int b_drv_fault :1 ;       // bit 29
        unsigned int c_drv_fault :1 ;       // bit 30
        unsigned int e_drv_fault :1 ;       // bit 31
        unsigned int reserved :1 ;          // bit 32
    } ;
    struct {
        unsigned int all :32 ;
    } ;
} cnc_flags_t ;

typedef struct {
    uint32_t axis ;
    uint32_t type ;
    
    volatile uint32_t state ;
    
    volatile unsigned int *port ;
    volatile unsigned int *tris_set ;
    uint32_t pin ;
    int32_t cn_number ;
    uint32_t pullup ;
} switch_t ;

#define ADD_SWITCH(AXIS, TYPE, P_NAME, PIN, CN, PULLUP) \
    switch_t switch_ ## AXIS ## _ ## TYPE = {AXIS_ ## AXIS, TYPE, 0, \
                                                 &PORT ## P_NAME,        \
                                                 &TRIS ## P_NAME ## SET, \
                                                 PIN, CN, PULLUP}

#ifdef	__cplusplus
}
#endif

#endif	/* SWITCH_H */

