/* 
 * File:   leds.h
 * Author: tized
 *
 * Created on 23 September 2017, 22:06
 */

#ifndef LEDS_H
#define	LEDS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
enum led_state_e {
    LED_LOW = 0,
    LED_HIGH 
};

typedef struct {
    volatile unsigned int * tris_clr ;
    volatile unsigned int * port_clr ;
    volatile unsigned int * port_set ;
    unsigned int pin ;
    
    unsigned int on_state ;    
} led_t ;

#define DEF_LED(NAME, PRT, PIN, ON_STATE) \
    led_t led_ ## NAME = {&TRIS ## PRT ## CLR, &PORT ## PRT ## CLR, \
                          &PORT ## PRT ## SET, PIN, ON_STATE}

void led_off(led_t * led) {
    if(led->on_state) *(led->port_clr) = led->pin ;
    else *(led->port_set) = led->pin ;
}

void led_on(led_t * led) {
    if(led->on_state) *(led->port_set) = led->pin ;
    else *(led->port_clr) = led->pin ;
}

void setup_led(led_t * led) {
    *(led->tris_clr) = led->pin ;
    led_off(led) ;
}

#ifdef	__cplusplus
}
#endif

#endif	/* LEDS_H */

