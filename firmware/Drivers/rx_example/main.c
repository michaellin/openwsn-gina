#include "MPU9150.h"
#include "radio.h"
#include "ioCC2533.h"

void main(){
	P1DIR = 0x02;
	P1_2 = 0x01;
        uint8_t a = 1;
	radio_reset();
	radio_init();
     	radio_setFrequency(0x0F);
	radio_rxEnable();
	while(1);
}

#pragma vector = 0x83
__interrupt void RFRX_ISR(void){
	radio_isr();
}