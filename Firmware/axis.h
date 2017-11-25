/* 
 * File:   axis.h
 * Author: tized
 *
 * Created on 17 June 2016, 02:34
 */

#ifndef AXIS_H
#define	AXIS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
    
#define PICKOFF     (28)
    
#define BIT_0   0x00000001 
#define BIT_1   0x00000002
#define BIT_2   0x00000004
#define BIT_3   0x00000008 
#define BIT_4   0x00000010
#define BIT_5   0x00000020
#define BIT_6   0x00000040 
#define BIT_7   0x00000080
#define BIT_8   0x00000100
#define BIT_9   0x00000200 
#define BIT_10  0x00000400
#define BIT_11  0x00000800
#define BIT_12  0x00001000 
#define BIT_13  0x00002000
#define BIT_14  0x00004000
#define BIT_15  0x00008000 
  
enum axis_name  {
    AXIS_NONE = 0,
	AXIS_X =  1,
	AXIS_Y =  2,
	AXIS_Z =  4,
	AXIS_A =  8,
	AXIS_B = 16,
	AXIS_C = 32,
	AXIS_E = 64,
	INVALID = 128,
};

enum axis_index {
    X_IND = 0,
    Y_IND,
    Z_IND,
    A_IND,
    B_IND,
    C_IND,
    E_IND
};

// Phases of signaling 
typedef enum {
    STEP_UP = 0,                // Step signal is up
    STEP_DOWN,                  // Step signal is down
    DIR_SETUP,                  // Setup of direction signal
    DIR_HOLD,                   // Hold between step and direction change
    STEP_WAIT       
} step_fsm_t ;

typedef struct {
    uint32_t axis ;
    int64_t velocity ;
    int32_t dir ;
    int64_t position ;
    int64_t old_position ;
    uint32_t enable ;
    uint32_t inverse ;
    
    volatile unsigned int * const tris_clr ;
    volatile unsigned int * const port_set ;
    volatile unsigned int * const port_clr ;
    const uint32_t en_pin ;
    uint32_t dir_pin ;
    uint32_t step_pin ;
    
    volatile unsigned int * const fault_tris_set ;
    volatile unsigned int * const fault_port ;
    const uint32_t fault_pin ;
    
    uint32_t step_len ;
    uint32_t step_space ;
    uint32_t dir_setup ;
    uint32_t dir_hold ;
    
    step_fsm_t step_state ;
    int32_t step_counter ;
    int32_t dir_counter ;
} axis_t ; 

#define ADD_AXIS(AXIS, INVERSE, PRT, EN_PIN, DIR_PIN, STEP_PIN, FAULT_PORT, FAULT_PIN) \
    axis_t axis_ ## AXIS = {AXIS_ ## AXIS, 0, 1, 0, 0, 0, INVERSE, \
                                   &TRIS ## PRT ## CLR , &PORT ## PRT ## SET, \
                                   &PORT ## PRT ## CLR, EN_PIN, \
                                   DIR_PIN, STEP_PIN, \
                                   &TRIS ## FAULT_PORT ## SET, \
                                   &PORT ## FAULT_PORT, \
                                   FAULT_PIN, \
                                   2, 2, 1, 0, DIR_HOLD, 0, 0}                       

    
void stepgen(axis_t * const * axes, int32_t * active_axes) ;
void axis_activate(axis_t * const axis) ;
void axis_deactivate(axis_t * const axis) ;
void axis_setup(axis_t * const axis) ;
int32_t get_axis_ind(int32_t axis_name, uint32_t axes_mask) ;

#ifdef	__cplusplus
}
#endif

#endif	/* AXIS_H */

