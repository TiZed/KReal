#include "pwm.h"
#include <p32xxxx.h>
#include <float.h>
#include <math.h>

void pwm_activate(pwm_t * const pwm) {
    unsigned int ps = 1 ;
    
    // Setup output compare
    *(pwm->oc_con) = 0x0 ;
    *(pwm->oc_r) = 0x0 ;
    *(pwm->oc_rs) = 0x0 ;
    *(pwm->oc_con) = 0x06 ;       //  PWM mode w/o fault pin
    
    // Prevent divide by 0
    if (pwm->frequency == 0) pwm->frequency = 50 ;
    
    pwm->pr = 80000000 / (ps * pwm->frequency) - 1 ;
    
    while (pwm->pr > MAX_PR && ps <= 256) {
       ps <<= 1 ;
        pwm->pr = (80000000 / (ps * pwm->frequency)) - 1 ;
    }
    
    ps >>= 1 ;
    
    switch(ps) {
        case 1:
            ps = 0 ; break ;
        case 2:
            ps = 1 ; break ;
        case 4:
            ps = 2 ; break ;
        case 8:
            ps = 3 ; break ;
        case 16:
            ps = 4 ; break ;
        case 32:
            ps = 5 ; break ;
        case 64:
            ps = 6 ; break ;
        case 256:
            ps = 7 ; break ;            
    }
    
    ps <<= 4 ;
    
    // Select timer
    if(pwm->freq_src == TIMER2) {
        T2CON = 0 ;
        T2CON = 0x2000 | ps ;        // 16-bit timer
        PR2 = pwm->pr ;
        T2CONSET = 0x8000 ;          // Enable timer
    }
    
    // Select timer
    else if (pwm->freq_src == TIMER3) {
        T3CON = 0 ;
        T3CON = 0x2000 | ps ;        // 16-bit timer 
        PR3 = pwm->pr ;
        T3CONSET = 0x8000 ;          // Enable timer
        *(pwm->oc_con) |= 0x8 ;
    }
    
    pwm->pr += 1 ;
    
    *(pwm->oc_con) |= 0x8000 ;       // Enable OC on 16-bit mode
}


void pwm_set_duty(pwm_t * const pwm) {
    if (pwm->inverse) 
        *(pwm->oc_rs) = (long)((1.0 - pwm->duty.flt / 100.0) * (float)pwm->pr + 0.5) ;
    else 
        *(pwm->oc_rs) = (long)((pwm->duty.flt / 100.0) * (float)pwm->pr + 0.5) ;
}


void pwm_deactivate(pwm_t * const pwm) {
    *(pwm->oc_rs) = 0 ;
    *(pwm->oc_con) = 0x0 ;
}