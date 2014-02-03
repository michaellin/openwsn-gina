/**
\brief Cross-platform declaration for MPU9150 chip module.

\author Andrew Kwong <ankwong@berkeley.edu> August 2013.
*/

#ifndef __MPU9150_H
#define __MPU9150_H

//===================================== includes ================================

#include "i2c.h"

//===================================== define ================================
#define MPU_device 0x68
#define MPU_magnet 0x0C

#define MPU9150_RA_SMPLRT_DIV 0x19
#define MPU9150_RA_CONFIG 0x1A
#define MPU9150_RA_GYRO_CONFIG 0x1B
#define MPU9150_RA_ACCEL_CONFIG 0x1C
#define MPU9150_RA_FF_THR 0x1D
#define MPU9150_RA_FF_DUR 0x1E
#define MPU9150_RA_MOT_THR 0x1F
#define MPU9150_RA_MOT_DUR 0x20
#define MPU9150_RA_ZRMOT_THR 0x21
#define MPU9150_RA_ZRMOT_DUR 0x22
#define MPU9150_RA_FIFO_EN 0x23
#define MPU9150_RA_I2C_MST_CTRL 0x24
#define MPU9150_RA_I2C_SLV0_ADDR 0x25
#define MPU9150_RA_I2C_SLV0_REG 0x26
#define MPU9150_RA_I2C_SLV0_CTRL 0x27
#define MPU9150_RA_I2C_SLV1_ADDR 0x28
#define MPU9150_RA_I2C_SLV1_REG 0x29
#define MPU9150_RA_I2C_SLV1_CTRL 0x2A
#define MPU9150_RA_I2C_SLV2_ADDR 0x2B
#define MPU9150_RA_I2C_SLV2_REG 0x2C
#define MPU9150_RA_I2C_SLV2_CTRL 0x2D
#define MPU9150_RA_I2C_SLV3_ADDR 0x2E
#define MPU9150_RA_I2C_SLV3_REG 0x2F
#define MPU9150_RA_I2C_SLV3_CTRL 0x30
#define MPU9150_RA_I2C_SLV4_ADDR 0x31
#define MPU9150_RA_I2C_SLV4_REG 0x32
#define MPU9150_RA_I2C_SLV4_DO 0x33
#define MPU9150_RA_I2C_SLV4_CTRL 0x34
#define MPU9150_RA_I2C_SLV4_DI 0x35
#define MPU9150_RA_I2C_MST_STATUS 0x36
#define MPU9150_RA_INT_PIN_CFG 0x37
#define MPU9150_RA_INT_ENABLE 0x38
#define MPU9150_RA_INT_STATUS 0x3A
#define MPU9150_RA_ACCEL_XOUT_H 0x3B
#define MPU9150_RA_ACCEL_XOUT_L 0x3C
#define MPU9150_RA_ACCEL_YOUT_H 0x3D
#define MPU9150_RA_ACCEL_YOUT_L 0x3E
#define MPU9150_RA_ACCEL_ZOUT_H 0x3F
#define MPU9150_RA_ACCEL_ZOUT_L 0x40
#define MPU9150_RA_TEMP_OUT_H 0x41
#define MPU9150_RA_TEMP_OUT_L 0x42
#define MPU9150_RA_GYRO_XOUT_H 0x43
#define MPU9150_RA_GYRO_XOUT_L 0x44
#define MPU9150_RA_GYRO_YOUT_H 0x45
#define MPU9150_RA_GYRO_YOUT_L 0x46
#define MPU9150_RA_GYRO_ZOUT_H 0x47
#define MPU9150_RA_GYRO_ZOUT_L 0x48
#define MPU9150_RA_EXT_SENS_DATA_00 0x49
#define MPU9150_RA_EXT_SENS_DATA_01 0x4A
#define MPU9150_RA_EXT_SENS_DATA_02 0x4B
#define MPU9150_RA_EXT_SENS_DATA_03 0x4C
#define MPU9150_RA_EXT_SENS_DATA_04 0x4D
#define MPU9150_RA_EXT_SENS_DATA_05 0x4E
#define MPU9150_RA_EXT_SENS_DATA_06 0x4F
#define MPU9150_RA_EXT_SENS_DATA_07 0x50
#define MPU9150_RA_EXT_SENS_DATA_08 0x51
#define MPU9150_RA_EXT_SENS_DATA_09 0x52
#define MPU9150_RA_EXT_SENS_DATA_10 0x53
#define MPU9150_RA_EXT_SENS_DATA_11 0x54
#define MPU9150_RA_EXT_SENS_DATA_12 0x55
#define MPU9150_RA_EXT_SENS_DATA_13 0x56
#define MPU9150_RA_EXT_SENS_DATA_14 0x57
#define MPU9150_RA_EXT_SENS_DATA_15 0x58
#define MPU9150_RA_EXT_SENS_DATA_16 0x59
#define MPU9150_RA_EXT_SENS_DATA_17 0x5A
#define MPU9150_RA_EXT_SENS_DATA_18 0x5B
#define MPU9150_RA_EXT_SENS_DATA_19 0x5C
#define MPU9150_RA_EXT_SENS_DATA_20 0x5D
#define MPU9150_RA_EXT_SENS_DATA_21 0x5E
#define MPU9150_RA_EXT_SENS_DATA_22 0x5F
#define MPU9150_RA_EXT_SENS_DATA_23 0x60
#define MPU9150_RA_MOT_DETECT_STATUS 0x61
#define MPU9150_RA_I2C_SLV0_DO 0x63
#define MPU9150_RA_I2C_SLV1_DO 0x64
#define MPU9150_RA_I2C_SLV2_DO 0x65
#define MPU9150_RA_I2C_SLV3_DO 0x66
#define MPU9150_RA_I2C_MST_DELAY_CTRL 0x67
#define MPU9150_RA_SIGNAL_PATH_RESET 0x68
#define MPU9150_RA_MOT_DETECT_CTRL 0x69
#define MPU9150_RA_USER_CTRL 0x6A
#define MPU9150_RA_PWR_MGMT_1 0x6B
#define MPU9150_RA_PWR_MGMT_2 0x6C
#define MPU9150_RA_FIFO_COUNTH 0x72
#define MPU9150_RA_FIFO_COUNTL 0x73
#define MPU9150_RA_FIFO_R_W 0x74
#define MPU9150_RA_WHO_AM_I 0x75

#define MPU9150_MG_CNTL 0x0A
#define MPU9150_MG_HXL 0x03
#define MPU9150_MG_HXH 0x04
#define MPU9150_MG_HYL 0x05
#define MPU9150_MG_HYH 0x06
#define MPU9150_MG_HZL 0x07
#define MPU9150_MG_HZH 0x08

//===================================== prototypes ============================
//Admin
void imu_init(); //Sets configuration, turns on magnetometer, exits low power mode
void imu_disable(); //Makes IMU Sleep
void imu_enable(); //Makes IMU Wake Up
//Accelerometer
void imu_accel_get_measurement(uint8_t* buffer); //Writes accel register data to buffer
void imu_accel_set_fs_range(uint8_t range); //Possible values are 2, 4, 8, 16 (in +-g). Else sets to 2g.
//Gyroscope
void imu_gyro_get_measurement(uint8_t* buffer); //Writes gyro register data to buffer
void imu_gyro_set_fs_range(uint16_t range); //Possible values are 250, 500, 1000, 2000 (in +- degrees/second). Else sets to 1000.
//Temperature
void imu_temp_get_measurement(uint8_t* buffer); //Writes temp register data to buffer

//Magnetometer functions
void imu_take_magnet_measurement(); //Writes magnetometer register data to buffer
void imu_wait_magnet_measurement(); // Waits until magnetometer measurements are done writing to buffer
uint8_t imu_check_magnet_measurement();
void imu_get_magnet_measurement(uint8_t* buffer); //Reads from magnetometer buffer

void imu_get_acc_gyr_temp_measurements(uint8_t* buffer); //Get all (except magnetomter).
/*Using this ensures that we get all our measurements from the same sampling instant.
*Use imu_take_magnet_measurement right before this function; imu_wait_magnet_measurement and imu_get_magnet_measurement right after this function
*To ensure that we get the magnetometer values from the same sampling instant as well.
*/

#endif