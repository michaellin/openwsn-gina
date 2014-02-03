//=========================== includes ======================================
#include <iocc2533.h>


//=========================== prototypes ======================================
void    leds_init(){
  P1DIR |= 0x03;
}

void    led1_on(){
  P1_1 = 0x01;
}

void    led1_off(){
  P1_1 = 0x00;
}

void    led1_toggle(){
  P1_1 ^= 0x01;
}

void    led2_on(){
  P1_0 = 0x01;
}

void    led2_off(){
  P1_0 = 0x00;
}

void    led2_toggle(){
  P1_0 ^= 0x01;
}

void    leds_all_on(){
  P1_0 = 0x01;
  P1_1 = 0x01;
}

void    leds_all_off(){
  P1_0 = 0x00;
  P1_1 = 0x00;
}

void    leds_all_toggle(){
  P1_0 ^= 0x01;
  P1_1 ^= 0x01;
}