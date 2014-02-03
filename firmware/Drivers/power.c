/*
 * Power setting drivers for gina 4.0
 *
 * Authors:
 * Andrew Kwong <ankwong@berkeley.edu>, August 2013
 */
 
//===================================== includes ================================
#include "iocc2533.h"
//===================================== define ================================

//===================================== prototypes ============================


void low_power_mode_enable(){
  SLEEPCMD |= 0x03;
  PCON = 0x01;
}

void active_mode_enable(){
  SLEEPCMD &= 0xFC;
  PCON = 0x01;
}