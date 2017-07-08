/* 
 * karsam_uno.h - Step generator for Machinekit
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
 */

#ifndef KARSAM_UNO_H
#define	KARSAM_UNO_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define SYS_FREQ (80000000L)
#define PB_CLOCK (SYS_FREQ / (1 << OSCCONbits.PBDIV))
#define CORE_TIMER_FREQ (SYS_FREQ / 2)

#define FLASH_SPEED_HZ          30000000 //Max Flash speed
#define PB_BUS_MAX_FREQ_HZ      80000000 //Max Peripheral bus speed
    
#define BUFFER_SIZE     (50)

// Commands
#define CMD_CFG     0x47464323    // #CFG
#define CMD_VEL     0x4c455623    // #VEL
#define CMD_POS     0x524f5023    // #POS
#define CMD_STP     0x50545323    // #STP
#define CMD_CHK     0x4b484323    // #CHK
#define CMD_PWM     0x4d575023    // #PWM
#define CMD_NUL     0x00000000    // Dummy NULL command
    
void set_switch(uint32_t axis, uint32_t type) ;
void clr_switch(uint32_t axis, uint32_t type) ;

#define SWAP_BYTES(IN)  IN << 24 | (IN & 0xff00) << 8 | (IN & 0xff0000) >> 8 | IN >> 24

static inline unsigned int __attribute__((always_inline)) spiTransW(unsigned int word) {
    while(!SPI2STATbits.SPITBE) WDTCONSET = 1 ; 
    SPI2BUF = word ;
    while(!SPI2STATbits.SPIRBF) WDTCONSET = 1 ; 
    return SPI2BUF ;
}

uint32_t __attribute__((nomips16))  enableInterrupts(void)
{
    uint32_t status = 0 ;
    asm volatile("ei    %0" : "=r"(status)) ;
    return status ;
}

uint32_t __attribute__((nomips16)) disableInterrupts(void)
{
    uint32_t status = 0 ;
    asm volatile("di    %0" : "=r"(status)) ;
    return status ;
}

void __attribute__((nomips16))  restoreInterrupts(uint32_t st)
{
    if (st & 0x00000001) asm volatile("ei") ;
    else asm volatile("di") ;
}

#ifdef	__cplusplus
}
#endif

#endif	/* KARSAM_UNO_H */

