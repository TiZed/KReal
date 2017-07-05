#include "axis.h"


void stepgen(axis_t * const* axes, int32_t * active_axes) {
    axis_t * axis ;
    int32_t * a ;
    
    for (a = active_axes ; *a != -1 ; a++) {
        axis = axes[*a] ;
    
        // Decrement step high/low counter
        if(axis->step_counter > 0) axis->step_counter-- ;

        // Decrement dir setup/hold /counter
        if(axis->dir_counter > 0) axis->dir_counter-- ;

        // Increment position, if not waiting on dir hold
        if(axis->step_state != DIR_HOLD) axis->position += axis->velocity ;

        // If waited for dir hold and timer elapsed, release stepping
        else if(axis->step_state == DIR_HOLD && axis->dir_counter == 0) {
            // Positive direction
            if(axis->velocity >= 0) {
                *(axis->port_set) = axis->dir_pin ;
                axis->old_dir = 1 ;
            }
            // Negative direction
            else {
                *(axis->port_clr) = axis->dir_pin ;
                axis->old_dir = -1 ;
            }

            axis->step_state = DIR_SETUP ;
            axis->dir_counter = axis->dir_setup ;
        }

        // If waited for dir setup and timer elapsed
        if(axis->step_state == DIR_SETUP && axis->dir_counter == 0) {
            axis->step_state = STEP_WAIT ;
        }

        if(axis->step_state == STEP_WAIT) {
            if((axis->position ^ axis->old_position) & (1LL << PICKOFF)) {
                *(axis->port_set) = axis->step_pin ; 

                axis->old_position = axis->position ;
                axis->step_state = STEP_UP ;
                axis->step_counter = axis->step_len ;
            }
        }

        // Step up state
        if(axis->step_state == STEP_UP && axis->step_counter == 0) {
            *(axis->port_clr) = axis->step_pin ;  

            axis->step_state = STEP_DOWN ;
            axis->step_counter = axis->step_space ;
            axis->dir_counter = axis->dir_hold ;   
        }

        if(axis->step_state == STEP_DOWN && axis->step_counter == 0) {
            if((axis->old_dir * axis->velocity) < 0) axis->step_state = DIR_HOLD ;
            else axis->step_state = STEP_WAIT ;
        }
    }
}

void axis_activate(axis_t * const axis) {
    axis->position = 0 ;
    axis->old_position = 0 ;
    *(axis->tris_clr) = axis->en_pin | axis->dir_pin | axis->step_pin ;
    *(axis->port_set) = axis->en_pin ;  
}

void axis_deactivate(axis_t * const axis) {
    *(axis->port_clr) = axis->en_pin | axis->step_pin ; 
}

void axis_setup(axis_t * const axis) {
    *(axis->port_clr) = axis->en_pin | axis->dir_pin | axis->step_pin ; 
    *(axis->tris_clr) = axis->en_pin | axis->dir_pin | axis->step_pin ;
}

int32_t get_axis_ind(int32_t axis_name, uint32_t axes_mask) {
    if(!(axis_name & axes_mask)) return -1 ;
   
    switch(axis_name) {
        case AXIS_X:
            return X_IND ;
        case AXIS_Y:
            return Y_IND ;
        case AXIS_Z:
            return Z_IND ;
        case AXIS_A:
            return A_IND ;
        case AXIS_B:
            return B_IND ;
        case AXIS_C:
            return C_IND ;
        case AXIS_E:
            return E_IND ;
        default:
            return -1 ;         
    }
}
