/* 
 * File:   cd4048b.h
 * Author: tized
 *
 * Created on 20 September 2017, 15:28
 */

#ifndef CD4048B_H
#define	CD4048B_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
    
enum logic_mode_e {
    //                KKK
    //                cba
    LOGIC_NOR     = 0b000,
    LOGIC_OR      = 0b100,
    LOGIC_OR_AND  = 0b010,
    LOGIC_OR_NAND = 0b110,
    LOGIC_AND     = 0b001,
    LOGIC_NAND    = 0b101,
    LOGIC_AND_NOR = 0b011,
    LOGIC_AND_OR  = 0b111
};

typedef struct {
    volatile unsigned int *ka_tris_clr ;
    volatile unsigned int *ka_port_set ;
    volatile unsigned int *ka_port_clr ;
    uint32_t ka_pin ;
    
    volatile unsigned int *kb_tris_clr ;
    volatile unsigned int *kb_port_set ;
    volatile unsigned int *kb_port_clr ;
    uint32_t kb_pin ;
    
    volatile unsigned int *kc_tris_clr ;
    volatile unsigned int *kc_port_set ;
    volatile unsigned int *kc_port_clr ;
    uint32_t kc_pin ;
    
    volatile unsigned int *en_tris_clr ;
    volatile unsigned int *en_port_set ;
    volatile unsigned int *en_port_clr ;
    uint32_t en_pin ;
    
    uint32_t logic_mode ;
    
} cd4048b_t;

#define SET_CD4048B(NAME, KA_PORT, KA_PIN, KB_PORT, KB_PIN, KC_PORT, KC_PIN, EN_PORT, EN_PIN) \
    static cd4048b_t NAME ## _logic = {&TRIS ## KA_PORT ## CLR,  \
                                      &PORT ## KA_PORT ## SET,  \
                                      &PORT ## KA_PORT ## CLR,  \
                                      KA_PIN, \
                                      &TRIS ## KB_PORT ## CLR,  \
                                      &PORT ## KB_PORT ## SET,  \
                                      &PORT ## KB_PORT ## CLR,  \
                                      KB_PIN, \
                                      &TRIS ## KC_PORT ## CLR,  \
                                      &PORT ## KC_PORT ## SET,  \
                                      &PORT ## KC_PORT ## CLR,  \
                                      KC_PIN, \
                                      &TRIS ## EN_PORT ## CLR,  \
                                      &PORT ## EN_PORT ## SET,  \
                                      &PORT ## EN_PORT ## CLR,  \
                                      EN_PIN, \
                                      LOGIC_NOR}

void setup_logic(uint32_t mode, cd4048b_t * logic) ;
void enable_logic(cd4048b_t * logic) ;
void disable_logic(cd4048b_t * logic) ;

#ifdef	__cplusplus
}
#endif

#endif	/* CD4048B_H */

