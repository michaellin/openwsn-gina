//===================================== includes ================================

#include "radio.h"
#include "MPU9150.h"

//===================================== variables ============================
uint16_t pktindex;
uint16_t CRC;

//===================================== prototypes ============================

void radio_init(){

  //Required power options for radio
  SLEEPCMD &= ~OSC_PD;                       /* turn on 16MHz RC and 32MHz XOSC */                \
  while (!(SLEEPSTA & XOSC_STB));            /* wait for 32MHz XOSC stable */                     \
  asm("NOP");                                /* chip bug workaround */                            \
  for (int i=0; i<504; i++) asm("NOP");          /* Require 63us delay for all revs */                \
  CLKCONCMD = CLKCONCMD_VALUE; /* Select 32MHz XOSC and the source for 32K clock */ \
  while (CLKCONSTA != CLKCONCMD_VALUE); /* Wait for the change to be effective */   \
  SLEEPCMD |= OSC_PD;      /* turn off 16MHz RC */                              \
    
	//Register values
	IEN2 = 0x01; //enable RF interrupt
	FRMCTRL0 = 0x00;  //0x40 for AUTOCRC 0x00 for no AUTOCRC
	FRMCTRL1 = 0x00; //Disable RX after a TX, abort if TX underflow, set to 0x01 if you want to enable RX after a TX
	TXCTRL = 0x69; //Controls TX Settings (current in DAC, dc level to TX mixer, etc.)
	FSMCTRL = 0x00; //No time-out, ack frame timing
	MDMCTRL0 = 0x85;//Modem Options, TX filtering on and other options
	CSPCTRL = 0x01; //Turns on the control on the CSP 
	EA = 1; // Enable interrupts
	S1CON = 0x00; // Clear RF interrupt flag
        RFIRQF1 = 0x00;
        RFIRQF0 = 0x00;
	}
	
void radio_reset(){
	RFST = 0xED; //Flush RXFIFO
	RFST = 0xEE; //Flush TXFIFO
	RFST = 0xFF; //Clear CSP Program memory
}
	
void radio_setFrequency(uint8_t frequency){
	//Uses channel, not frequency.
	FREQCTRL = 5*(frequency-11) + 11;
}

//rfOn is equivalent to rxenable + txenable
void radio_rfOn(){
  FRMFILT0 = 0x00; //Turn off frame filtering: accepts all types of frames.
  FRMCTRL1 |= 0x01; //Sets so that it returns to RX mode after transmission
  RFIRQM0 |= BV(6); //Turn on Packet Received Interrupt
  RFST = 0xE3; //ISRXON: Immediately enables RX mode.
  RFIRQM1 = 0x02; //Enable TXDone Interrupt
  S1CON = 0x00; // Clear RF interrupt flag
}

void     radio_rfOff(){
  RFST = 0xEF; //Turns off RF
}

void radio_loadPacket(uint8_t* packet, uint8_t len){
  RFD = len;
  uint8_t count = 0;
  while(len > 0){
      RFD = packet[count];
      count++;
      len--;
  }
}

void radio_txEnable(){
  RFIRQM1 = 0x02; //Enable TXDone Interrupt
  S1CON = 0x00; // Clear RF interrupt flag
}

void radio_txNow(){
	RFST = 0xE9; //ISTXON: Sends current frame instantly. Does not check for CCA. Use ISTXONCCA (RFST = 0xEA) for CCA check.
}

void radio_rxEnable(){
  FRMFILT0 = 0x00; //Turn off frame filtering: accepts all types of frames.
  FRMCTRL1 |= 0x01; //Sets so that it returns to RX mode after transmission
  RFIRQM0 |= BV(6); //Turn on Packet Received Interrupt
  S1CON = 0x00; // Clear RF interrupt flag
  RFST = 0xE3; //ISRXON: Immediately enables RX mode.
}

//LQI not implemented
void     radio_getReceivedFrame(uint8_t* bufRead, uint8_t* lenRead, uint8_t  maxBufLen, int8_t* rssi,
                                uint8_t* lqi, uint8_t* crc){
  uint8_t count = 0;
  //read length
  *lenRead = RFD;
  //read packet
  while ((count < *lenRead - 2) && (count < maxBufLen)){
    *bufRead = RFD;
    count ++;
    bufRead ++;
  }
  //read crc
  *crc = RFD;
  crc++;
  *crc = RFD;
  //read rssi
  *rssi = RSSI;
}

void   radio_isr(){	
	if (RFIRQF1 > 0){
	//TX Done Interrupt
	
	//Declare packet
  uint8_t IMU_values[24] = {0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,};
	
	//Prepare packet index
	pktindex++;
	IMU_values[0] = pktindex>>8;
    IMU_values[1] = pktindex & 0xFF;
    
	//Take IMU measurements
    imu_get_acc_gyr_temp_measurements(&IMU_values[2]);
 
	//Take IMU compass measurements
	if(imu_check_magnet_measurement()){
		imu_get_magnet_measurement(&IMU_values[16]);
		imu_take_magnet_measurement();
	}
	//Get CRC as checksum
	CRC = IMU_values[0] + IMU_values[1] + IMU_values[2]
			 + IMU_values[3] + IMU_values[4] + IMU_values[5] + IMU_values[6] + IMU_values[7] + IMU_values[8]
			 + IMU_values[9] + IMU_values[10] + IMU_values[11] + IMU_values[12] + IMU_values[13] + IMU_values[14]
                         + IMU_values[15] + IMU_values[16] + IMU_values[17] + IMU_values[18] + IMU_values[19] + IMU_values[20]
                         + IMU_values[21];
    IMU_values[22] = (CRC>>8);
    IMU_values[23] = (CRC & 0xFF);

	//Send Packet
	radio_loadPacket(IMU_values, 24);
	radio_txNow();
	
	//Clear interrupt flag
    RFIRQF1 = 0x00;
	
	}else if(RFIRQF0 > 0){
		//RX interrupt
		uint8_t crc[2];
		uint8_t lenRead[1];
		uint8_t bufRead[25];
		uint8_t maxBufLen = 25;
		uint8_t lqi[1];
		int8_t rssi[1];
		radio_getReceivedFrame(bufRead, lenRead,maxBufLen, rssi, lqi, crc);
		bufRead[24] = 1;
		//clear interrupt flag
		RFIRQF0 = 0x00;
	}
	
	//Clear interrupt flag
	S1CON = 0x00;
}
