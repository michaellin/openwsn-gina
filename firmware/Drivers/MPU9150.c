/*
 * Driver for the MPU9150 IMU
 *
 * Authors:
 * Andrew Kwong <ankwong@berkeley.edu>, August 2013
 */

//===================================== includes ================================

#include "MPU9150.h"

//===================================== prototypes ============================
//Admin
void imu_init()
{
i2c_write_register(MPU_device,MPU9150_RA_PWR_MGMT_1,0x01);
i2c_write_register(MPU_device,MPU9150_RA_SMPLRT_DIV,0x07);
i2c_write_register(MPU_device,MPU9150_RA_CONFIG,0x00);
i2c_write_register(MPU_device,MPU9150_RA_GYRO_CONFIG,0x10);
i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x00);
i2c_write_register(MPU_device,MPU9150_RA_FIFO_EN,0x00);
i2c_write_register(MPU_device,MPU9150_RA_I2C_MST_CTRL,0x00);
i2c_write_register(MPU_device,MPU9150_RA_INT_PIN_CFG,0x02);
i2c_write_register(MPU_device,MPU9150_RA_I2C_MST_DELAY_CTRL,0x00);
i2c_write_register(MPU_device,MPU9150_RA_SIGNAL_PATH_RESET,0x00);
i2c_write_register(MPU_device,MPU9150_RA_USER_CTRL,0x00);
i2c_write_register(MPU_device,MPU9150_RA_PWR_MGMT_2,0x00);
i2c_write_register(MPU_device,MPU9150_RA_FIFO_R_W,0x00);
}

void imu_disable()
{
i2c_write_register(MPU_device,MPU9150_RA_PWR_MGMT_1,0x41);
}

void imu_enable()
{
i2c_write_register(MPU_device,MPU9150_RA_PWR_MGMT_1,0x01);
}

//Accel
void imu_accel_get_measurement(uint8_t* buffer)
{
i2c_read_registers(MPU_device,MPU9150_RA_ACCEL_XOUT_H,6,buffer);
}

void imu_accel_set_fs_range(uint8_t range){
if (range == 2){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x00);
}else if (range == 4){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x08);
}else if (range == 8){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x10);
}else if (range ==16){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x18);
}else{
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x00);
}
}

void imu_gyro_get_measurement(uint8_t* buffer)
{
i2c_read_registers(MPU_device,MPU9150_RA_TEMP_OUT_H,6,buffer);
}

void imu_gyro_set_fs_range(uint16_t range){
if (range == 250){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x00);
}else if (range == 500){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x08);
}else if (range == 1000){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x10);
}else if (range ==2000){
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x18);
}else{
  i2c_write_register(MPU_device,MPU9150_RA_ACCEL_CONFIG,0x10);
}
}

void imu_temp_get_measurement(uint8_t* buffer)
{
i2c_read_registers(MPU_device,MPU9150_RA_GYRO_XOUT_H,6,buffer);
}

void imu_take_magnet_measurement()
{
i2c_write_register(MPU_magnet, MPU9150_MG_CNTL, 0x01);
}

void imu_wait_magnet_measurement(){
	uint8_t magStatus[1] = {0};
	while (magStatus[0] == 0){
		i2c_read_registers(MPU_magnet,0x02, 1, magStatus);
	}
}

uint8_t imu_check_magnet_measurement(){
	uint8_t magStatus[1] = {0};
	i2c_read_registers(MPU_magnet,0x02, 1, magStatus);
	return magStatus[0];
}

void imu_get_magnet_measurement(uint8_t* buffer)
{
i2c_read_registers(MPU_magnet,0x03,6,buffer);
}

void imu_get_acc_gyr_temp_measurements(uint8_t* buffer)
{
i2c_read_registers(MPU_device,MPU9150_RA_ACCEL_XOUT_H,14,buffer);
}