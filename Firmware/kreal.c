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
 * LED Read - RF1 
 * 
 * SPI MISO - SDO2/RG8 
 * SPI MOSI - SDI2/RG7
 * SPI CLK  - SCLK2/RG6
 * SPI SS   - SS2/RG9
 * 
 * UART1 RX - U2RX/RF4
 * UART1 TX - U2TX/RF5
 * 
 * Logic:
 * Ka - RE4
 * Kb - RE5
 * Kc - RE6
 * En - RE7
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

//         Port
//     Axis      EN     Dir     Step
ADD_AXIS(X, B, BIT_6,  BIT_5,  BIT_4) ;  
ADD_AXIS(Y, B, BIT_9,  BIT_8,  BIT_7) ;  
ADD_AXIS(Z, B, BIT_12, BIT_11, BIT_10) ;   

static axis_t * const axes_arr[] = {&axis_X, &axis_Y, &axis_Z} ;

// TODO: Add EMO and Faults interrupts handler

//        Axis  Type  P   Pin   CN  PU
ADD_SWITCH(NONE, EMO, D,  BIT_0, -1, 0) ;  
ADD_SWITCH(X,  LIMIT, D,  BIT_4, 13, 1) ;  
ADD_SWITCH(X,   HOME, D,  BIT_5, 14, 1) ;  
ADD_SWITCH(Y,  LIMIT, D,  BIT_6, 15, 1) ;  
ADD_SWITCH(Y,   HOME, D,  BIT_7, 16, 1) ;  
ADD_SWITCH(Z,  LIMIT, B,  BIT_0,  2, 1) ;  
ADD_SWITCH(Z,   HOME, B,  BIT_1,  3, 1) ;  
ADD_SWITCH(X,  FAULT, E,  BIT_0, -1, 0) ;  
ADD_SWITCH(Y,  FAULT, E,  BIT_1, -1, 0) ;  
ADD_SWITCH(Z,  FAULT, E,  BIT_2, -1, 0) ;  

static switch_t * const switches[] = {&switch_NONE_EMO,
                                      &switch_X_LIMIT, &switch_X_HOME,
                                      &switch_Y_LIMIT, &switch_Y_HOME,
                                      &switch_Z_LIMIT, &switch_Z_HOME,
                                      &switch_X_FAULT, &switch_Y_FAULT,
                                      &switch_Z_FAULT} ;

ADD_PWM(spindle, OC2, TIMER2, 1) ;
ADD_PWM(laser,   OC3, TIMER3, 1) ;

static pwm_t * const pwms[] = {&pwm_spindle, &pwm_laser} ;

static uint32_t axes_mask = 0,  num_active_axes = 0, active_axes_mask = 0 ;
static uint32_t core_count = 0 ;
static int32_t active_axes[8] = {-1} ;
static uint32_t pos_acc_arr[16] = {0} ;

static uint32_t num_active_pwm = 0 ;

static const uint32_t num_axes = sizeof(axes_arr) / sizeof(axis_t *) ;
static const uint32_t num_switches = sizeof(switches) / sizeof(switch_t *) ;
static const uint32_t num_pwms = sizeof(pwms) / sizeof(pwm_t *) ;

volatile cnc_flags_t flags = {0} ;

BUFFER(rxb, BUFFER_SIZE) ;
BUFFER(txb, BUFFER_SIZE) ;

static uint32_t checksum = 0 ;
static uint32_t cmd = CMD_NUL ;
static uint32_t spi_done = 0 ;
static uint32_t pos_send_count = 0 ;

// Configure SPI 2
static void config_spi() {
    int t ;
    
    SPI2CON = 0x0 ;
    SPI2CONbits.MODE32 = 1 ;        // 32-bit mode
    SPI2CONbits.CKP = 0 ;           // Clock polarity
    SPI2CONbits.CKE = 1 ;           // Clock edge select
    SPI2CONbits.MSTEN = 0 ;
    SPI2CONbits.SSEN = 0 ;
    SPI2CONbits.ENHBUF = 1 ;        // Enable Enhanced buffer
    t = SPI2BUF ;                   // Clear buffer
    
    // SPI Interrupt priority 2, sub-priority 0
//    IPC7bits.SPI2IP = 6 ;
//    IPC7bits.SPI2IS = 0 ;
    
    // Clear RX overflow flag
    SPI2STATbits.SPIROV = 0 ;
    
    // Clear SPI interrupt flags 
    IFS1bits.SPI2RXIF = 0 ;
    IFS1bits.SPI2TXIF = 0 ;
    IFS1bits.SPI2EIF = 0 ;
    
    // Enable SPI Rx interrupt
    IEC1bits.SPI2TXIE = 1 ;
    IEC1bits.SPI2RXIE = 0 ;
    IEC1bits.SPI2EIE = 0 ;
    
    // Enable SPI
    SPI2CONbits.ON = 1 ;
}

static void dma_setup() {
    IEC1bits.DMA0IE = 0 ;       // Disable DMA Ch. 0 Interrupt
    IEC1bits.DMA1IE = 0 ;       // Disable DMA Ch. 1 Interrupt
    IFS1bits.DMA0IF = 0 ;       // Clear and DMA Ch. 0 pending interrupts
    IFS1bits.DMA1IF = 0 ;       // Clear and DMA Ch. 1 pending interrupts
    
    DMACONbits.ON = 1 ;         // Turn DMA controller on
    
    // Configure DMA Channel 0 for SPI Rx
    DCH0CONbits.CHEN = 0 ;              // Disable DMA Ch. 0
    DCH0CONbits.CHPRI = 3 ;             // DMA Ch. 0 set high priority (3)
    DCH0CONbits.CHAEN = 1 ;             // DMA Ch. 0 automatic open
    
    DCH0ECONbits.CHSIRQ = _SPI2_RX_IRQ ; // Set DMA Ch. 0 start IRQ from SPI2 Rx
    DCH0ECONbits.SIRQEN = 1 ;            // Enable DMA Ch. 0 start from IRQ
    
    DCH0SSA = (void *) &SPI2BUF ;       // DMA Ch. 0 source address is SPI2 Buffer
    DCH0DSA = (void *) rxb.data ;       // DMA Ch. 0 destination address is RX buffer
    
    DCH0SSIZ = 4 ;                      // 4B source size (SPI2BUF)
    DCH0CSIZ = 4 ;                      // Set cell transfer to 4B
    
    // Configure DMA Channel 1 for SPI Tx
    DCH1CONbits.CHEN = 0 ;              // Disable DMA Ch. 1
    DCH1CONbits.CHPRI = 3 ;             // DMA Ch. 1 set high priority (3)
    DCH1CONbits.CHAEN = 1 ;             // DMA Ch. 1 automatic open
    
    DCH1ECONbits.CHSIRQ = _SPI2_TX_IRQ ; // Set DMA Ch. 1 start IRQ from SPI2 Tx
    DCH1ECONbits.SIRQEN = 1 ;            // Enable DMA Ch. 1 start from IRQ
    
    DCH1SSA = (void *) txb.data ;       // DMA Ch. 1 source address is the TX buffer
    DCH1DSA = (void *) &SPI2BUF ;       // DMA Ch. 0 destination address SPI2BUF
    
    DCH1DSIZ = 4 ;                      // 4B destination size (SPI2BUF)
    DCH1CSIZ = 4 ;                      // Set cell transfer to 4B
}

void dma_trig(uint size) {
    
    DCH0DSIZ = size ;                   // destination size 
    DCH1SSIZ = size ;                   // source size 
}
    
}


void config_switches() {
    uint32_t cn_num, i ;
    
    IEC1bits.CNIE = 0 ;         // Disable CN interrupt
    AD1PCFGSET = 0xffff ;       // Make all AN pins digital
    CNCONbits.ON = 1 ;          // Enable CN module
   
    for (i = 0 ; i < num_switches ; i++) {
        // Set pin as input
        *(switches[i]->tris_set) = switches[i]->pin ;
        
        // Pins with CN -1 have no CN
        if (switches[i]->cn_number != -1) {
            // Enable CN for pin
            cn_num = 1 << switches[i]->cn_number ;
            CNENSET = cn_num ;

            // If requested, enable pull-up 
            if (switches[i]->pullup) CNPUESET = cn_num ;
        }
        
        // Get current pin state
        switches[i]->state = *(switches[i]->port) & switches[i]->pin ;
        
        // Set or clear flag related to the pin
        if (switches[i]->state) set_switch(switches[i]->axis, switches[i]->type) ;
        else clr_switch(switches[i]->axis, switches[i]->type) ;  
    }  
    
    // Set CN Interrupt priority to 5, sub-priority to 0
    IPC6bits.CNIP = 5 ;  
    IPC6bits.CNIS = 1 ;
    
    IFS1bits.CNIF = 0 ;     // Clear CN interrupt
    IEC1bits.CNIE = 0 ;     // Enable CN interrupt
}

void setup(void) {
    uint32_t wait_states, cache_reg, sys_freq, tmp ;
    
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
    
    // Set peripherals clock to sys_clk
    tmp = OSCCON ;
    tmp &= ~_OSCCON_PBDIV_MASK ;
    
    SYSKEY = 0 ;                // Unlock sequence
    SYSKEY = 0xAA996655 ; 
    SYSKEY = 0x556699AA ;  
    
    OSCCON = tmp ;
    tmp = OSCCON ;
    
    SYSKEY = 0 ;                // Lock
    
    // Enable MultiVectorInt
    asm volatile("mfc0   %0,$13" : "=r"(tmp));
    tmp |= 0x00800000;
    asm volatile("mtc0   %0,$13" : "+r"(tmp));
    
    INTCONSET = _INTCON_MVEC_MASK ; 
}

int main(void) {
    unsigned int base_freq, int_store, state ;
    int i, axis ;
    unsigned int chks ;
    int32_t * a ;
    
    disableInterrupts() ;
    
    TRISD = 0x0 ;
    PORTD = 0x0 ;
    setup() ;
    
    // Disable all pull-ups
    CNPUE = 0 ;
    
    config_switches() ;
    config_spi() ;
    
    // Core timer enable and set interrupt priority
    _CP0_BIS_CAUSE(_CP0_CAUSE_DC_MASK) ;
    IPC0bits.CTIP = 5 ;
    IPC0bits.CTIS = 0 ;
    
    for(i = 0 ; i < num_axes ; i++) {
        axes_mask |= axes_arr[i]->axis ;
        axis_setup(axes_arr[i]) ;
    }
    
    TRISECLR = 0x3 ;
    PORTECLR = 0x3 ;
    
    // Set RF0 as output and turn on LED
    TRISFCLR = 0x1 ;
    PORTFCLR = 0x1 ;
    
    // Report timeout fault
    if (RCONbits.WDTO || RCONbits.BOR) {
        flags.ucont_fault = 1 ;
        RCONbits.WDTO = 0 ;
        RCONbits.BOR = 0 ;
        PORTFSET = 0x1 ;
    }
              
    clear(&rxb) ;
    clear(&txb) ;
    
    enableInterrupts() ;

    // Enable Watchdog timer
    WDTCONbits.ON = 1 ;
    
    while (1) {
        WDTCONSET = 1 ;
        
        if(cmd != CMD_NUL) {
            if (!spi_done) {
                switch(cmd) {
                    case CMD_CFG:
                        dma_setup(3 + 5 * num_active_axes + num_active_pwm) ;
                        break ;
                    case CMD_POS:
                        dma_setup(pos_send_count) ;
                        break ;
                    case CMD_VEL:
                        dma_setup(num_active_axes) ;
                        break ;
                    case CMD_PWM:
                        dma_setup(num_active_pwm) ;
                        break ;
                    case CMD_STP:
                        spi_done = 1 ;
                        break ;
                // switch(cmd)
                }
            // if (!spi_done)    
            }
            else {
                switch(cmd) {
                // Configure command: configure frequency and axes
                case CMD_CFG:
                    IEC0bits.CTIE = 0 ;
        //            checksum = cmd ; 

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

                        pwm_activate(pwms[i]) ;
                    }

                    // TODO: checksum check here
                    checksum = 0 ;

                    pos_send_count = 2 * num_active_axes + 2 ;

                    // Set core timer compare ticks to selected pulse
                    // generation base frequency
                    if (base_freq > 0) {
                        core_count = CORE_TIMER_FREQ / base_freq ;

                        if(core_count > 0) {
                            int_store = disableInterrupts() ;
                            _CP0_SET_COUNT(0) ;
                            _CP0_SET_COMPARE(core_count) ;
                            _CP0_BIC_CAUSE(_CP0_CAUSE_DC_MASK) ;
                            restoreInterrupts(int_store) ;

                            IFS0bits.CTIF = 0 ;
                            IEC0bits.CTIE = 1 ;
                            PORTFSET = 0x1 ;
                        }
                    }

                    break ;

                // Position command: Report position of each axis
                case CMD_POS:
                    IEC0bits.CTIE = 0 ;
                    clear(&txb) ;
                    checksum = 0 ;

                    for (i = 0 ; i < pos_send_count ; i++) {
                        if (SPI2STATbits.SPITBE) SPI2BUF = pos_acc_arr[i] ;
                        else push(&txb, pos_acc_arr[i]) ;
                    }

                    IEC0bits.CTIE = 1 ;
                    break ;

                // Velocity command: Update velocity of selected axes.
                case CMD_VEL:
                    IEC0bits.CTIE = 0 ;
                    clear(&txb) ;

                    for (a = active_axes ; *a != -1 ; a++) {
                        axes_arr[*a]->velocity = pop_wait(&rxb) ; 
                    }

                    SPI2BUF = checksum ;
                    checksum = 0 ;
                    IEC0bits.CTIE = 1 ;

                    break ;

                // PWM command: Update the duty cycle of the PWM sources
                case CMD_PWM:
                    IEC0bits.CTIE = 0 ;
                    clear(&txb) ;
        //            checksum = cmd ;

                    for (i = 0 ; i < num_active_pwm ; i++) {
                        pwms[i]->duty.bin = pop_wait(&rxb) ;
        //                checksum ^= pwms[i]->duty.bin ;
                    }

                    SPI2BUF = checksum ;
                    checksum = 0 ;
                    IEC0bits.CTIE = 1 ;

                    for (i = 0 ; i < num_active_pwm ; i++) pwm_set_duty(pwms[i]) ;

                    break ;

                // Stop command: Deactivate all active axes
                case CMD_STP:
                    IEC0bits.CTIE = 0 ;
                    _CP0_BIS_CAUSE(_CP0_CAUSE_DC_MASK) ;

                    for (a = active_axes ; *a != -1 ; a++)
                        axis_deactivate(axes_arr[*a]) ;

                    for (i = 0 ; i < num_active_pwm ; i++) 
                        pwm_deactivate(pwms[i]) ;

                    PORTFCLR = 0x1 ;
                    IFS0bits.CTIF = 0 ;

                    break ;
                // switch (cmd)
                }
            
                cmd = CMD_NUL ;
                spi_done = 0 ;
                
            // else .. if(!spi_done)
            }
        // if(cmd != ...)
        }
        
        else {
            for (i = 0 ; i < num_switches ; i++) {
                state = *(switches[i]->port) & switches[i]->pin ;

                if (state != switches[i]->state) {
                    switches[i]->state = state ;

                    if (switches[i]->state) set_switch(switches[i]->axis, switches[i]->type) ;
                    else clr_switch(switches[i]->axis, switches[i]->type) ; 
                }
                WDTCONSET = 1 ;
            }  
            
            i = 0 ;
            
            IEC0bits.CTIE = 0 ;
            pos_acc_arr[i++] = flags.all ;
            chks = flags.all ;
            for (a = active_axes ; *a != -1 ; a++) {
                pos_acc_arr[i] = axes_arr[*a]->position >> 32 ;
                chks ^= pos_acc_arr[i++] ;
                pos_acc_arr[i] ^= axes_arr[*a]->position ;
                chks ^= pos_acc_arr[i++] ;
            }
            pos_acc_arr[i] = chks ;
            IEC0bits.CTIE = 1 ;
        }

    return(EXIT_SUCCESS) ;
}


void set_switch(uint32_t axis, uint32_t type) {
    int32_t  a ;
    int i ;
    
    switch(axis) {
        case AXIS_NONE:
            if (type == EMO) { 
                flags.switch_emo = 1 ;
            
                for (i = 0 ; i < num_active_axes ; i++) {
                    a = active_axes[i] ;
                    axis_deactivate(axes_arr[a]) ;
                }

                IEC0bits.CTIE = 0 ;
                PORTFCLR = 0x1 ;
            }
            break ;
            
        case AXIS_X: 
            if (type == LIMIT) flags.switch_x_limit = 1 ;
            else if (type == HOME) flags.switch_x_home = 1 ;
            else if (type == FAULT) flags.x_drv_fault = 1 ;
//            axes_arr[X_IND]->velocity = 0 ;
            break ;
            
        case AXIS_Y: 
            if (type == LIMIT) flags.switch_y_limit = 1 ;
            else if (type == HOME) flags.switch_y_home = 1 ;
            else if (type == FAULT) flags.y_drv_fault = 1 ;
//            axes_arr[Y_IND]->velocity = 0 ;
            break ;
            
        case AXIS_Z: 
            if (type == LIMIT) flags.switch_z_limit = 1 ;
            else if (type == HOME) flags.switch_z_home = 1 ;
            else if (type == FAULT) flags.z_drv_fault = 1 ;
//            axes_arr[Z_IND]->velocity = 0 ;
            break ;
            
        case AXIS_A: 
            if (type == LIMIT) flags.switch_a_limit = 1 ;
            else if (type == HOME) flags.switch_a_home = 1 ;
            else if (type == FAULT) flags.a_drv_fault = 1 ;
//            axes_arr[A_IND]->velocity = 0 ;
            break ;
            
        case AXIS_B: 
            if (type == LIMIT) flags.switch_b_limit = 1 ;
            else if (type == HOME) flags.switch_b_home = 1 ;
            else if (type == FAULT) flags.b_drv_fault = 1 ;
//            axes_arr[B_IND]->velocity = 0 ;
            break ;
            
        case AXIS_C: 
            if (type == LIMIT) flags.switch_c_limit = 1 ;
            else if (type == HOME) flags.switch_c_home = 1 ;
            else if (type == FAULT) flags.c_drv_fault = 1 ;
//            axes_arr[C_IND]->velocity = 0 ;
            break ;
            
        case AXIS_E: 
            if (type == LIMIT) flags.switch_e_limit = 1 ;
            else if (type == HOME) flags.switch_e_home = 1 ;
            else if (type == FAULT) flags.e_drv_fault = 1 ;
//            axes_arr[E_IND]->velocity = 0 ;
            break ;
    }
}

void clr_switch(uint32_t axis, uint32_t type) {
    switch (axis) {
        case AXIS_NONE:
            if (type == EMO) flags.switch_emo = 0 ;
            break ;
            
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

/*
// Switch change interrupt
void __ISR(_CHANGE_NOTICE_VECTOR, IPL5SRS) SwitchHandler(void) {
    unsigned int i, state ;
    
    PORTEINV = 0x2 ;
    
    for (i = 0 ; i < num_switches ; i++) {
        state = *(switches[i]->port) & switches[i]->pin ;
        
        if (state != switches[i]->state) {
            switches[i]->state = state ;
            
            if (switches[i]->state) set_switch(switches[i]->axis, switches[i]->type) ;
            else clr_switch(switches[i]->axis, switches[i]->type) ; 
        }
        WDTCONSET = 1 ;
    }  
  
    IFS1bits.CNIF = 0 ;     // Clear CN interrupt 
    PORTEINV = 0x2 ;
}
*/


void __ISR(_SPI_2_VECTOR, IPL6AUTO) SpiHandler(void) {
    
    if (IFS1bits.SPI2RXIF) { 
        cmd = SPI2BUF ;
        IEC1bits.SPI2TXIE = 0 ;
    }
    
    flags.ucont_fault = IFS1bits.SPI2EIF ;
    IFS1CLR = 0xe0 ;
}


// Main timing interrupt
void __ISR(_CORE_TIMER_VECTOR, IPL5AUTO) CoreTimerHandler(void) {
   
    // Reset core timer
    _CP0_SET_COUNT(0) ;
    _CP0_SET_COMPARE(core_count) ;
    
    stepgen(axes_arr, active_axes) ;
    
    IFS0bits.CTIF = 0 ;
}
