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
    
enum led_on_state_e {
    LED_LOW = 0,
    LED_HIGH 
};

enum led_state_e {
    LED_OFF = 0,
    LED_ON,
    LED_BLINK_FAST,
    LED_BLINK_SLOW
};

typedef struct {
    volatile unsigned int * const tris_clr ;
    volatile unsigned int * const port_clr ;
    volatile unsigned int * const port_set ;
    volatile unsigned int * const port_inv ;
    unsigned int pin ;
    
    unsigned int on_state ; 
    unsigned int state ;
    
    int blink_timer ;
} led_t ;

#define DEF_LED(NAME, PRT, PIN, ON_STATE) \
    led_t led_ ## NAME = {&TRIS ## PRT ## CLR, &PORT ## PRT ## CLR, \
                          &PORT ## PRT ## SET, &PORT ## PRT ## INV, \
                          PIN, ON_STATE, LED_OFF, 0}

void led_off(led_t * const led) {
    if(led->on_state) *(led->port_clr) = led->pin ;
    else *(led->port_set) = led->pin ;
    
    led->state = LED_OFF ;
}

void led_on(led_t * const led) {
    if(led->on_state) *(led->port_set) = led->pin ;
    else *(led->port_clr) = led->pin ;
    
    led->state = LED_ON ;
}

void led_blink(led_t * const led, unsigned int blink_speed) {
    led->state = blink_speed ;
    led->blink_timer = 0 ;
}

void setup_led(led_t * const led) {
    *(led->tris_clr) = led->pin ;
    led_off(led) ;
}

void _led_blink_check(led_t * const led) {
    if (led->blink_timer-- == 0) {
        *(led->port_inv) = led->pin ;
        led->blink_timer = (led->state == LED_BLINK_FAST) ? 2:5 ;
    }
}

#ifdef	__cplusplus
}
#endif

#endif	/* LEDS_H */

