/* 
 * File:   pwm.h
 * Author: tized
 *
 * Created on 17 October 2016, 22:49
 */

#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define MAX_PR 65535
    
enum frequency_source {
    TIMER2,
    TIMER3
}; 

typedef union {
    unsigned int bin ;
    float flt ;
} duty_cycle_t ;
    
typedef struct {
    volatile unsigned int * const oc_con ;
    volatile unsigned int * const oc_r ;
    volatile unsigned int * const oc_rs ;
    unsigned int frequency ;
    
    unsigned int freq_src ;
    unsigned int pr ;
    unsigned int inverse ;
    duty_cycle_t duty ;
    duty_cycle_t old_duty ;
} pwm_t ;

#define ADD_PWM(PWM, OC, TIMER, INVERSE) \
    pwm_t pwm_ ## PWM = {&OC ## CON, &OC ## R, &OC ## RS, 0, TIMER, 0, 0, 0, 0}

void pwm_activate(pwm_t * const pwm) ;
void pwm_set_duty(pwm_t * const pwm) ;
void pwm_deactivate(pwm_t * const pwm) ;
void pwm_reverse(pwm_t * const pwm) ;


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

