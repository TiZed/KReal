/* 
 * cd4048b.c - Control the CD4048B - CMOS multifunction gate
 *
 * Created on 20 September 2017
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
 */

#include "cd4048b.h"

// Setup the CD4048B logic IC
void setup_logic(uint32_t mode, cd4048b_t * logic) {
    // Define logic pins as output
    *(logic->en_tris_clr) = logic->en_pin ;
    *(logic->ka_tris_clr) = logic->ka_pin ;
    *(logic->kb_tris_clr) = logic->kb_pin ;
    *(logic->kc_tris_clr) = logic->kc_pin ;
    
    // Drop enable pin
    *(logic->en_port_clr) = logic->en_pin ;
    
    // Configure logic to selected mode
    if(mode & 0b001) *(logic->ka_port_set) = logic->ka_pin ;
    else *(logic->ka_port_clr) = logic->ka_pin ;
    
    if(mode & 0b010) *(logic->kb_port_set) = logic->kb_pin ;
    else *(logic->kb_port_clr) = logic->kb_pin ;
    
    if(mode & 0b100) *(logic->kc_port_set) = logic->kc_pin ;
    else *(logic->kc_port_clr) = logic->kc_pin ;
}

// Enable the CD4048B IC
void enable_logic(cd4048b_t * logic) {
    *(logic->en_port_set) = logic->en_pin ;
}

// Disable the CD4048B IC
void disable_logic(cd4048b_t * logic) {
    *(logic->en_port_clr) = logic->en_pin ;
}
