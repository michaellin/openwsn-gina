#include "MPU9150.h"
#include "radio.h"
#include "ioCC2533.h"
#include "leds.h"

uint8_t IMU_values[24] = {0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,};

void main(){
	uint16_t pktindex;
	uint16_t CRC;
	
	leds_init();
	led1_on();
	
	imu_init();
	imu_enable();
        imu_gyro_set_fs_range(2000);
        imu_accel_set_fs_range(4);
	radio_reset();
	radio_init();
	radio_setFrequency(0x0F);
	radio_txEnable();
		
	pktindex = 0x0000;
	IMU_values[0] = pktindex>>8;
	IMU_values[1] = pktindex & 0xFF;
	
	imu_get_acc_gyr_temp_measurements(&IMU_values[2]);
	imu_take_magnet_measurement();
	
	CRC = IMU_values[0] + IMU_values[1] + IMU_values[2]
			+ IMU_values[3] + IMU_values[4] + IMU_values[5] + IMU_values[6] + IMU_values[7] + IMU_values[8]
			+ IMU_values[9] + IMU_values[10] + IMU_values[11] + IMU_values[12] + IMU_values[13] + IMU_values[14]
			+ IMU_values[15] + IMU_values[16] + IMU_values[17] + IMU_values[18] + IMU_values[19]+ IMU_values[20]
			+ IMU_values[21];
	IMU_values[22] = (CRC>>8);
	IMU_values[23] = (CRC & 0xFF);
	
	radio_loadPacket(IMU_values, 24);
	radio_txNow();
	while(1);
}

#pragma vector = 0x83
__interrupt void RFTransmit_ISR(void){
  radio_isr();
}