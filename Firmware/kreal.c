/* 
 * kreal.c - Step generator for Machinekit
 *
 * Created on 1 July 2017
 *
 *  Copyright (C) 2017 TiZed
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
 * ----------------------------------------------------------------------------
 *
 *
 * LED blue - RF0
 * LED Red  - RF1 
 * 
 * SPI MISO - SDO2/RG8 
 * SPI MOSI - SDI2/RG7
 * SPI CLK  - SCLK2/RG6
 * SPI SS   - SS2/RG9
 * 
 * UART1 RX - U2RX/RF4 - Used as program signal at startup
 * UART1 TX - U2TX/RF5
 * 
 * Logic:
 * Ka - RE4
 * Kb - RE5
 * Kc - RE6
 * En - RE7
 * 
 * Switches Interrupt - INT3/RD10
 * Faults Interrupt   - INT2/RD9
 * Z-Level interrupt  - INT1/RD8
 * EMO switch         - INT0/RD0
 * 
 */

#include <p32xxxx.h>
#include <GenericTypeDefs.h>
#include <stdint.h>
#include <sys/attribs.h>
#include <float.h>

#include "kreal.h"
#include "pwm.h"
#include "axis.h"
#include "switch.h"
#include "uint_buffer.h"
#include "cd4048b.h"
#include "leds.h"


// Axes Configuration

//         Port                      Fault
//     Axis      EN     Dir     Step  Port  Pin
ADD_AXIS(X, B, BIT_6 ,  BIT_5,  BIT_4, E, BIT_0) ;  
ADD_AXIS(Y, B, BIT_9 ,  BIT_8,  BIT_7, E, BIT_1) ;  
ADD_AXIS(Z, B, BIT_12, BIT_11, BIT_10, E, BIT_2) ;   
// ADD_AXIS(A, B, BIT_15, BIT_14, BIT_13, E, BIT_3) ;

axis_t * const axes_arr[] = {&axis_X, &axis_Y, &axis_Z} ; // &axis_A

// Switches Configuration

//        Axis  Type  P   Pin   CN  PU
ADD_SWITCH(X,  LIMIT, D,  BIT_4, 13, 1) ;  
ADD_SWITCH(X,   HOME, D,  BIT_5, 14, 1) ;  
ADD_SWITCH(Y,  LIMIT, D,  BIT_6, 15, 1) ;  
ADD_SWITCH(Y,   HOME, D,  BIT_7, 16, 1) ;  
ADD_SWITCH(Z,  LIMIT, B,  BIT_0,  2, 1) ;  
ADD_SWITCH(Z,   HOME, B,  BIT_1,  3, 1) ;  
// ADD_SWITCH(A,  LIMIT, B,  BIT_2,  4, 1) ;  
// ADD_SWITCH(A,   HOME, B,  BIT_3,  5, 1) ; 

switch_t * const switches[] = {&switch_X_LIMIT, &switch_X_HOME,
                               &switch_Y_LIMIT, &switch_Y_HOME,
                               &switch_Z_LIMIT, &switch_Z_HOME} ;
//                             &switch_A_LIMIT, &switch_A_HOME} ;

// PWM Channels configuration

//      NAME     Ch.  Timer   Inverse 
ADD_PWM(spindle, OC2, TIMER2, 1) ;
ADD_PWM(laser,   OC3, TIMER3, 1) ;

pwm_t * const pwms[] = {&pwm_spindle, &pwm_laser} ;

// LEDs

DEF_LED(red, F, BIT_1, LED_HIGH) ;
DEF_LED(blue, F, BIT_0, LED_HIGH) ;

led_t * const leds[] = {&led_red, &led_blue} ;

SET_CD4048B(ic1, E, BIT_4, E, BIT_5, E, BIT_6, E, BIT_7) ;

static uint32_t axes_mask = 0,  num_active_axes = 0, active_axes_mask = 0 ;
static uint32_t core_count = 0 ;
static int32_t active_axes[8] = {-1} ;

static uint32_t num_active_pwm = 0 ;

const uint32_t num_axes = sizeof(axes_arr) / sizeof(axis_t *) ;
const uint32_t num_switches = sizeof(switches) / sizeof(switch_t *) ;
const uint32_t num_pwms = sizeof(pwms) / sizeof(pwm_t *) ;
const uint32_t num_leds = sizeof(leds) / sizeof(led_t *) ; 

volatile cnc_flags_t flags = {0} ;

BUFFER(rxb, BUFFER_SIZE) ;
BUFFER(txb, BUFFER_SIZE) ;

static uint32_t checksum = 0 ;
static uint32_t cmd = CMD_NUL ;
static uint32_t spi_cs_last = 0 ;
static uint32_t spi_cs_current = 0 ;

// Configure SPI 2
static void config_spi() {
    int t ;
    
    SPI2CON = 0x0 ;
    
    // Clear SPI interrupt flags 
    IFS1CLR = _IFS1_SPI2RXIF_MASK | _IFS1_SPI2TXIF_MASK | _IFS1_SPI2EIF_MASK ;
   
    // SPI Interrupts
    IEC1CLR = _IEC1_SPI2RXIE_MASK | _IEC1_SPI2TXIE_MASK | _IEC1_SPI2RXIE_MASK ;
    
    t = SPI2BUF ;                   // Clear buffer
    
    // SPI 32-bit, slave mode with enhanced buffer
    // TX buffer not full interrupt mode
    // RX buffer not empty interrupt mode
    SPI2CONSET = _SPI2CON_MODE32_MASK | _SPI2CON_CKE_MASK | // _SPI2CON_ENHBUF_MASK |
                 (0b11 << _SPI2CON_STXISEL_POSITION) |
                 (0b01 << _SPI2CON_SRXISEL_POSITION);
    
    TRISGSET = _TRISG_TRISG9_MASK ;              // Set SS pin as input
    spi_cs_current = PORTG & _PORTG_RG9_MASK ;
    spi_cs_last = spi_cs_current ;
    
    // SPI Interrupt priority 2, sub-priority 0
//    IPC7bits.SPI2IP = 6 ;
//    IPC7bits.SPI2IS = 0 ;
    
    // Clear RX overflow flag
    SPI2STATCLR = _SPI2STAT_SPIROV_MASK ;
    
    // Enable SPI
    SPI2CONSET = _SPI2CON_ON_MASK ;
}

static void dma_setup() {
    DMACONSET = _DMACON_ON_MASK ;         // Turn DMA controller on
    
    // Disable DMA Ch. 0 & 1 Interrupt
    IEC1CLR = _IEC1_DMA0IE_MASK | _IEC1_DMA1IE_MASK ;
    
    // Clear and DMA Ch. 0 & 1 pending interrupts
    IFS1CLR = _IFS1_DMA0IF_MASK | _IFS1_DMA1IF_MASK ;
    
    // Set DMA Ch. 0 & 1 to interrupt priority 3
    IPC9SET = (3 << _IPC9_DMA0IP_POSITION) | (3 << _IPC9_DMA0IS_POSITION) ;
    IPC9SET = (3 << _IPC9_DMA1IP_POSITION) | (2 << _IPC9_DMA1IS_POSITION) ;
    
    // Configure DMA Channel 0 for SPI Rx
    
    // Disable ch. 0 set priority 2, automatic open
    DCH0CON = (2 << _DCH0CON_CHPRI_POSITION) | _DCH0CON_CHAEN_MASK ;
    
    // Set DMA Ch. 0 start IRQ from SPI2 Rx
    DCH0ECON = (_SPI2_RX_IRQ << _DCH0ECON_CHSIRQ_POSITION) | _DCH0ECON_SIRQEN_MASK ;
    
    DCH0SSA = _VirtToPhys(&SPI2BUF) ;   // DMA Ch. 0 source address is SPI2 Buffer
    DCH0DSA = _VirtToPhys(rxb.data) ;   // DMA Ch. 0 destination address is RX buffer
    
    DCH0SSIZ = 4 ;                      // 4B source size (SPI2BUF)
    DCH0CSIZ = 4 ;                      // Set cell transfer to 4B
    DCH0DSIZ = 4 * rxb.size ;           //
    
    DCH0INTCLR = 0x00ff00ff ;           // Clear all Ch. 0 events
    DCH0INTSET = _DCH0INT_CHERIE_MASK ; // Enable Address error interrupt
    
    // Configure DMA Channel 1 for SPI Tx
    
    // Disable ch. 1 set priority 3, automatic open
    DCH1CON = (3 << _DCH1CON_CHPRI_POSITION) | _DCH1CON_CHAEN_MASK ;
  
    // Set DMA Ch. 1 start IRQ from SPI2 Tx
    DCH1ECON = (_SPI2_TX_IRQ << _DCH1ECON_CHSIRQ_POSITION) | _DCH1ECON_SIRQEN_MASK ;
   
    DCH1SSA = _VirtToPhys(txb.data) ;   // DMA Ch. 1 source address is the TX buffer
    DCH1DSA = _VirtToPhys(&SPI2BUF) ;   // DMA Ch. 1 destination address SPI2BUF
    
    DCH1DSIZ = 4 ;                      // 4B destination size (SPI2BUF)
    DCH1CSIZ = 4 ;                      // Set cell transfer to 4B
    DCH1SSIZ = 4 * txb.size ;           // 
    
    DCH1INTCLR = 0x00ff00ff ;           // Clear all Ch. 1 events
    DCH1INTSET = _DCH1INT_CHERIE_MASK ; // Enable Address error interrupt
    
    IEC1SET = _IEC1_DMA0IE_MASK | _IEC1_DMA1IE_MASK ;
    
    DCH0CONSET = _DCH0CON_CHEN_MASK ;   // Enable channel 0
    DCH1CONSET = _DCH1CON_CHEN_MASK ;   // Enable channel 1              
}

void config_switches() {
    uint32_t cn_num, i ;
    
    for (i = 0 ; i < num_switches ; i++) {
        // Set pin as input
        *(switches[i]->tris_set) = switches[i]->pin ;
        
        // Pins with CN -1 have no CN
        if (switches[i]->cn_number != -1) {
            cn_num = 1 << switches[i]->cn_number ;

            // If requested, enable pull-up 
            if (switches[i]->pullup) CNPUESET = cn_num ;
        }
        
        // Get current pin state
        switches[i]->state = *(switches[i]->port) & switches[i]->pin ;
        
        // Set or clear flag related to the pin
        if (switches[i]->state) set_switch(switches[i]->axis, switches[i]->type) ;
        else clr_switch(switches[i]->axis, switches[i]->type) ;  
    }  
}

void config_ints() {
    IEC0CLR = _IEC0_INT0IE_MASK | _IEC0_INT1IE_MASK | _IEC0_INT2IE_MASK | _IEC0_INT3IE_MASK ;
    
    TRISDSET = BIT_0 | BIT_8 | BIT_9 | BIT_10 ;   // Set INT0-3 pins as input
    
    // Set falling edge for EMO & Z-Level
    INTCONCLR = _INTCON_INT0EP_MASK | _INTCON_INT1EP_MASK ;
   
    // Set raising edge for motor faults & switches
    INTCONSET = _INTCON_INT2EP_MASK | _INTCON_INT3EP_MASK ;
    
    // Clear all pending external interrupts
    IFS0CLR = _IFS0_INT0IF_MASK | _IFS0_INT1IF_MASK | _IFS0_INT2IF_MASK | _IFS0_INT3IF_MASK ;
   
    // Set external interrupts priority
    IPC0SET = (4 << _IPC0_INT0IP_POSITION) | (3 << _IPC0_INT0IS_POSITION) ;
    IPC1SET = (4 << _IPC1_INT1IP_POSITION) | (0 << _IPC1_INT1IS_POSITION) ;
    IPC2SET = (4 << _IPC2_INT2IP_POSITION) | (2 << _IPC2_INT2IS_POSITION) ;
    IPC3SET = (4 << _IPC3_INT3IP_POSITION) | (1 << _IPC3_INT3IS_POSITION) ;
     
    // Enable external interrupts
    IEC0SET = _IEC0_INT0IE_MASK | _IEC0_INT1IE_MASK | _IEC0_INT2IE_MASK | _IEC0_INT3IE_MASK ;
}

void setup(void) {
    uint32_t wait_states, cache_reg, sys_freq, tmp ;
    
    disableInterrupts() ;
    
    // Set cache wait states and enable predictive prefetch
    sys_freq = SYS_FREQ ;
    wait_states = 0 ;
    
    while(sys_freq > FLASH_SPEED_HZ) {
        wait_states++ ;
        sys_freq -= FLASH_SPEED_HZ ;
    }
    
    cache_reg = (3 << _CHECON_PREFEN_POSITION) ;
    cache_reg |= wait_states ;
    
    CHECON = cache_reg ;
    
    // Turn on caching for KSEG0
	asm("mfc0 %0,$16,0" :  "=r"(tmp));
	tmp = (tmp & ~7) | 3;
	asm("mtc0 %0,$16,0" :: "r" (tmp));
    
//    tmp = OSCCON ;
//    tmp &= ~_OSCCON_PBDIV_MASK ;   // Set peripherals clock to sys_clk
//    tmp |= _OSCCON_SOSCEN_MASK ;   // Enable 32.768 kHz secondary clock 
    
    SYSKEY = 0 ;                   // Unlock sequence
    SYSKEY = 0xAA996655 ; 
    SYSKEY = 0x556699AA ;  
    
    OSCCONSET = _OSCCON_SOSCEN_MASK ;
//    OSCCON = tmp ;
//    tmp = OSCCON ;
    
    SYSKEY = 0 ;                   // Lock
    
    // Enable MultiVectorInt
    asm volatile("mfc0   %0,$13" : "=r"(tmp));
    tmp |= 0x00800000;
    asm volatile("mtc0   %0,$13" : "+r"(tmp));
    
    INTCONSET = _INTCON_MVEC_MASK ; 
    
    enableInterrupts() ;
    
    AD1PCFGSET = 0xffff ;         // Make all AN pins digital
    CNPUE = 0 ;                   // Disable all pull-ups
    
    // Core timer enable and set interrupt priority
    _CP0_BIS_CAUSE(_CP0_CAUSE_DC_MASK) ;
    IPC0bits.CTIP = 5 ;
    IPC0bits.CTIS = 0 ;
    
    // Timer1 Setup
    T1CON = 0 ;                   // Disable Timer1
    TMR1 = 0 ;                    // Reset Timer1
    
    // Use external secondary oscillator
    // Set prescaler to 256 = 128 beats/sec.
    // TODO: Check why secondary oscillator is not working
    T1CONSET =  (0b11 << _T1CON_TCKPS_POSITION) ; // _T1CON_TCS_MASK |
    PR1 = 0x7a13 ;                    // Cycle every 0.1sec. (should be 0.25sec.)
    
    IPC1bits.T1IP = 1 ;           // Low priority RT interrupt
    IPC1bits.T1IS = 0 ;
    
    IFS0CLR = _IFS0_T1IF_MASK ;   // Clear Timer1 interrupt
    IEC0SET = _IEC0_T1IE_MASK ;   // Enable Timer1 interrupt
  
    T1CONSET = _T1CON_ON_MASK ;      // Enable Timer1 ;            
}

int main(void) {
    unsigned int base_freq, int_store, state, update_rate ;
    int i, axis, padding ;
    unsigned int chks ;
    int32_t * a ;
    int64_t pos ;
    
    setup() ;
    
    setup_led(&led_red) ;
    setup_led(&led_blue) ;
    led_on(&led_red) ;
    led_on(&led_blue) ;
    
    config_switches() ;
    setup_logic(LOGIC_OR, &ic1_logic) ;
    enable_logic(&ic1_logic) ;
    
    clear(&rxb) ;
    clear(&txb) ;
    
    config_spi() ;
    dma_setup() ;

    for(i = 0 ; i < num_axes ; i++) {
        axes_mask |= axes_arr[i]->axis ;
        axis_setup(axes_arr[i]) ;
    }
    
    // Report timeout/brownout fault
    if (RCONbits.WDTO || RCONbits.BOR) {
        flags.ucont_fault = 1 ;
        RCONbits.WDTO = 0 ;
        RCONbits.BOR = 0 ;
        led_off(&led_blue) ;
    }
              
    // Enable Watchdog timer
    WDTCONbits.ON = 1 ;
    
    led_off(&led_red) ;
    
    while (1) {
        WDTCONSET = 1 ;
        
        if(cmd != CMD_NUL) {
            checksum = cmd ; 
            
            switch(cmd) {
                
            // Configure command: configure frequency and axes
            case CMD_CFG:
                IEC0CLR = _IEC0_CTIE_MASK ;
                
                base_freq = pop(&rxb) ;
                checksum ^= base_freq ;
                
                num_active_axes = pop(&rxb) ;
                checksum ^= num_active_axes ;

                num_active_pwm = pop(&rxb) ;
                checksum ^= num_active_pwm ;

                for(i = 0 ; i < num_active_axes ; i++) {
                    axis = pop(&rxb) ;
                    checksum ^= axis ;

                    active_axes_mask |= axis ;
                    axis = get_axis_ind(axis, axes_mask) ;
                    active_axes[i] = axis ;

                    axes_arr[axis]->step_len = pop(&rxb) ;
                    checksum ^= axes_arr[axis]->step_len ;

                    axes_arr[axis]->step_space = pop(&rxb) ;
                    checksum ^= axes_arr[axis]->step_space ;

                    axes_arr[axis]->dir_setup = pop(&rxb) ; 
                    axes_arr[axis]->dir_counter = 0 ;
                    checksum ^= axes_arr[axis]->dir_setup ;

                    axes_arr[axis]->dir_hold = pop(&rxb) ;
                    checksum ^= axes_arr[axis]->dir_hold ;

                    // Activate axis (rise enable)
                    axis_activate(axes_arr[axis]) ;
                }

                active_axes[i] = -1 ;

                for(i = 0 ; i < num_active_pwm ; i++) {
                    pwms[i]->frequency = pop(&rxb) ;
                    checksum ^= pwms[i]->frequency ;

                    if(pwms[i]->frequency > 0)
                        pwm_activate(pwms[i]) ;
                }

                chks = pop(&rxb) ;
                if(chks != checksum) {
                    led_blink(&led_red, LED_BLINK_SLOW) ;
                    flags.xsum_error = 1 ;
                    base_freq = 0 ;
                }

                padding = 1 + num_active_axes - num_active_pwm ;
                padding = (padding >= 0) ? padding : 0 ;

                // Set core timer compare ticks to selected pulse
                // generation base frequency
                if (base_freq > 0) {
                    core_count = CORE_TIMER_FREQ / base_freq ;

                    if(core_count > 0) {
                        int_store = disableInterrupts() ;
                        _CP0_SET_COUNT(0) ;
                        _CP0_SET_COMPARE(core_count) ;
                        _CP0_BIC_CAUSE(_CP0_CAUSE_DC_MASK) ;
                        
                        config_ints() ;
                        restoreInterrupts(int_store) ;
                        
                        led_blink(&led_blue, LED_BLINK_SLOW) ;

                        IFS0CLR = _IFS0_CTIF_MASK ; 
                        IEC0SET = _IEC0_CTIE_MASK ;
                    }
                }

                break ;

            // Update command: Update velocity and PWM.
            case CMD_UPD:
                IEC0CLR = _IEC0_CTIE_MASK ;
              
                for (a = active_axes ; *a != -1 ; a++) {
                    axes_arr[*a]->velocity = pop(&rxb) ; 
                    checksum ^= axes_arr[*a]->velocity ;
                }
                
                for (i = 0 ; i < num_active_pwm ; i++) {
                    pwms[i]->duty.bin = pop(&rxb) ;
                    checksum ^= pwms[i]->duty.bin ;
                }

                chks = pop(&rxb) ;
                
                if(checksum != chks) {
                    flags.xsum_error = 1 ;
                    all_stop() ;
                }
                else
                    IEC0SET = _IEC0_CTIE_MASK ;
                
                // Clear padding that makes sure that tx == rx data
                for(i = 0 ; i < padding ; i++) pop(&rxb) ;
                
                break ;

            // Stop command: Deactivate all active axes and PWM channels
            case CMD_STP:
                all_stop() ;
                led_off(&led_red) ;
                base_freq = 0 ;
                break ;
            // switch (cmd)
            }

            cmd = CMD_NUL ;
        // if(cmd != ...)
        }
        
        else {
            spi_cs_current = PORTG & _PORTG_RG9_MASK ;
            
            // Check if transmit is done (rise of CS)
            if(!spi_cs_last && spi_cs_current) { 
                DCH0CONCLR = _DCH0CON_CHEN_MASK ;   // Suspend DMA channel 0
                DCH1CONCLR = _DCH1CON_CHEN_MASK ;   // Suspend DMA channel 1
                
                while(DCH0CON & _DCH0CON_CHBUSY_MASK || DCH1CON & _DCH1CON_CHBUSY_MASK) ;
                
                set_head(&rxb, 1 + DCH0DPTR >> 2) ;      // Realign RX buffer head
                set_tail(&txb, 1 + DCH1SPTR >> 2) ;      // Realign TX buffer tail
                
                cmd = pop(&rxb) ;                   // Get new command
                
                DCH0CONSET = _DCH0CON_CHEN_MASK ;   // Release DMA channel 0
                DCH1CONSET = _DCH1CON_CHEN_MASK ;   // Release DMA channel 1
            }
            
            else {
                // Update switches states
                update_switches() ;
            
                // Prepare transmit buffer, only if SPI is not in use
                if(PORTG & BIT_9) {
                    // Reset TX buffer
                    clear(&txb) ;
                    
                    // Place flags of switches and errors
                    push(&txb, flags.all) ;
                    chks = flags.all ;

                    // Place positions of all axes
                    for (a = active_axes ; *a != -1 ; a++) {
                        IEC0CLR = _IEC0_CTIE_MASK ;  // Suspend stepgen interrupt
                        pos = axes_arr[*a]->position ; 
                        IEC0SET = _IEC0_CTIE_MASK ;  // Release stepgen interrupt

                        push(&txb, pos >> 32) ;
                        chks ^= pos >> 32 ;
                        push(&txb, pos) ;
                        chks ^= pos ;
                    }
                    
                    // Place checksum
                    push(&txb, chks) ; 
                }
            }
            spi_cs_last = spi_cs_current ;
        }
    }

    return(EXIT_SUCCESS) ;
}

void all_stop() {
    int32_t * a ;
    int32_t i ; 
    
    IEC0CLR = _IEC0_CTIE_MASK ;
    _CP0_BIS_CAUSE(_CP0_CAUSE_DC_MASK) ;

    for (a = active_axes ; *a != -1 ; a++)
        axis_deactivate(axes_arr[*a]) ;

    for (i = 0 ; i < num_active_pwm ; i++) 
        pwm_deactivate(pwms[i]) ;

    IFS0CLR = _IFS0_CTIF_MASK ;
    
    led_off(&led_blue) ;
    led_on(&led_red) ;
}

void update_switches() {
    int32_t i ;
    
    for (i = 0 ; i < num_switches ; i++) {
        switches[i]->state = *(switches[i]->port) & switches[i]->pin ;
        
        if(!switches[i]->state)
            clr_switch(switches[i]->axis, switches[i]->type) ;
    }  
}

void set_switch(uint32_t axis, uint32_t type) {
    switch(axis) {
        case AXIS_X: 
            if (type == LIMIT) flags.switch_x_limit = 1 ;
            else if (type == HOME) flags.switch_x_home = 1 ;
            else if (type == FAULT) flags.x_drv_fault = 1 ;
            break ;
            
        case AXIS_Y: 
            if (type == LIMIT) flags.switch_y_limit = 1 ;
            else if (type == HOME) flags.switch_y_home = 1 ;
            else if (type == FAULT) flags.y_drv_fault = 1 ;
            break ;
            
        case AXIS_Z: 
            if (type == LIMIT) flags.switch_z_limit = 1 ;
            else if (type == HOME) flags.switch_z_home = 1 ;
            else if (type == FAULT) flags.z_drv_fault = 1 ;
            break ;
            
        case AXIS_A: 
            if (type == LIMIT) flags.switch_a_limit = 1 ;
            else if (type == HOME) flags.switch_a_home = 1 ;
            else if (type == FAULT) flags.a_drv_fault = 1 ;
            break ;
            
        case AXIS_B: 
            if (type == LIMIT) flags.switch_b_limit = 1 ;
            else if (type == HOME) flags.switch_b_home = 1 ;
            else if (type == FAULT) flags.b_drv_fault = 1 ;
            break ;
            
        case AXIS_C: 
            if (type == LIMIT) flags.switch_c_limit = 1 ;
            else if (type == HOME) flags.switch_c_home = 1 ;
            else if (type == FAULT) flags.c_drv_fault = 1 ;
            break ;
            
        case AXIS_E: 
            if (type == LIMIT) flags.switch_e_limit = 1 ;
            else if (type == HOME) flags.switch_e_home = 1 ;
            else if (type == FAULT) flags.e_drv_fault = 1 ;
            break ;
    }
}

void clr_switch(uint32_t axis, uint32_t type) {
    switch (axis) {
        case AXIS_X: 
            if (type == LIMIT) flags.switch_x_limit = 0 ;
            else if (type == HOME) flags.switch_x_home = 0 ;
            else if (type == FAULT) flags.x_drv_fault = 0 ;
            break ;
            
        case AXIS_Y: 
            if (type == LIMIT) flags.switch_y_limit = 0 ;
            else if (type == HOME) flags.switch_y_home = 0 ;
            else if (type == FAULT) flags.y_drv_fault = 0 ;
            break ;
            
        case AXIS_Z: 
            if (type == LIMIT) flags.switch_z_limit = 0 ;
            else if (type == HOME) flags.switch_z_home = 0 ;
            else if (type == FAULT) flags.z_drv_fault = 0 ;
            break ;
            
        case AXIS_A: 
            if (type == LIMIT) flags.switch_a_limit = 0 ;
            else if (type == HOME) flags.switch_a_home = 0 ;
            else if (type == FAULT) flags.a_drv_fault = 0 ;
            break ;
            
        case AXIS_B: 
            if (type == LIMIT) flags.switch_b_limit = 0 ;
            else if (type == HOME) flags.switch_b_home = 0 ;
            else if (type == FAULT) flags.b_drv_fault = 0 ;
            break ;
            
        case AXIS_C: 
            if (type == LIMIT) flags.switch_c_limit = 0 ;
            else if (type == HOME) flags.switch_c_home = 0 ;
            else if (type == FAULT) flags.c_drv_fault = 0 ;
            break ;
            
        case AXIS_E: 
            if (type == LIMIT) flags.switch_e_limit = 0 ;
            else if (type == HOME) flags.switch_e_home = 0 ;
            else if (type == FAULT) flags.e_drv_fault = 0 ;
            break ;
    }
}

// Emergency off interrupt handler, shutdown everything
void __ISR(_EXTERNAL_0_VECTOR, IPL4AUTO) EmoHandler(void) {
    all_stop() ;
    flags.switch_emo = 1 ;
    IFS0CLR = _IFS0_INT0IF_MASK ;  // Clear EMO interrupt  
}

// Z-level interrupt, just raise the flag
void __ISR(_EXTERNAL_1_VECTOR, IPL4AUTO) ZLevelHandler(void) {
    flags.z_level = 1 ;
    IFS0CLR = _IFS0_INT1IF_MASK ;  // Clear Z-Level interrupt       
}

// Motor fault interrupt, shutdown everything
void __ISR(_EXTERNAL_2_VECTOR, IPL4AUTO) MotorFaultHandler(void) {
    int32_t * a ;
    
    all_stop() ;
    
    for (a = active_axes ; *a != -1 ; a++) {
        if(*(axes_arr[*a]->fault_port) & axes_arr[*a]->fault_pin) 
            set_switch(axes_arr[*a]->axis, FAULT) ;
    }
    
    IFS0CLR = _IFS0_INT2IF_MASK ;  // Clear motor fault interrupt      
}

// Limit switch triggered interrupt, just raise the matching flag
void __ISR(_EXTERNAL_3_VECTOR, IPL4AUTO) SwitchHandler(void) {
    int32_t i ;
    
    for (i = 0 ; i < num_switches ; i++) {
        WDTCONSET = 1 ;
        switches[i]->state = *(switches[i]->port) & switches[i]->pin ; 
        
        if(switches[i]->state) 
            set_switch(switches[i]->axis, switches[i]->type) ;
    }
    
    IFS0CLR = _IFS0_INT3IF_MASK ;  // Clear switches interrupt       
}

// Main timing interrupt
void __ISR(_CORE_TIMER_VECTOR, IPL5AUTO) CoreTimerHandler(void) {
   
    // Reset core timer
    _CP0_SET_COUNT(0) ;
    _CP0_SET_COMPARE(core_count) ;
    
    stepgen(axes_arr, active_axes) ;
    
    IFS0CLR = _IFS0_CTIF_MASK ;   // Clear Core Timer interrupt         
}

// Real time clock timer, should trigger every 1/4 sec. 
void __ISR(_TIMER_1_VECTOR, IPL1AUTO) RTimer(void) {
    int32_t i = 0 ;
      
    while (i < num_leds) {
        if(leds[i]->state >= LED_BLINK_FAST) _led_blink_check(leds[i]) ;
        i++ ;
    }
    
    IFS0CLR = _IFS0_T1IF_MASK ;   // Clear Timer1 interrupt 
}

// Blink Red LED on DMA Tx or Rx address error
void __ISR(_DMA0_VECTOR, IPL3AUTO) RxDMAError(void) {
    
    flags.ucont_fault = 1 ;
    led_blink(&led_red, LED_BLINK_FAST) ;
    
    IFS1CLR = _IFS1_DMA0IF_MASK ;
}

void __ISR(_DMA1_VECTOR, IPL3AUTO) TxDMAError(void) {
    
    flags.ucont_fault = 1 ;
    led_blink(&led_red, LED_BLINK_FAST) ;
    
    IFS1CLR = _IFS1_DMA1IF_MASK ;
}
