/**
\brief CC2533-specific declaration "i2c" module.

\author Andrew Kwong <ankwong@berkeley.edu> August 2013
*/


#ifndef __I2C_H
#define __I2C_H

/**
\addtogroup drivers
\{
\addtogroup I2C
\{
*/
typedef unsigned char   uint8_t;    // Unsigned 8 bit integer
typedef unsigned int	uint16_t;   //Unsinged 16 bit integer

//=========================== define ==========================================

#define SR_SENT 0x08 // Write Start Bit Acknowledge
#define RS_SENT 0x10 // Repeated Start bit acknowledge
#define SLAW_ACK_SENT 0x18 // Slave device address ACK for Send
#define SLAW_NACK_SENT 0x20 // Slave device address NACK for Send
#define DATA_ACK_SENT 0x28 // Register Address and Value ACK
#define DATA_SENT 0x30
#define SLAR_ACK_SENT 0x40
#define SLAR_NACK_SENT 0x48
#define DATA_ACK_RECV 0x50
#define DATA_NACK_RECV 0x58

//33 kHz bit frequency
//#define I2C_SR  0x60 //Start CFG Value
//#define I2C_SP  0x50 //Stop CFG Value
//#define I2C_DO  0x40 //"Do" CFG Value
//#define I2C_CO  0x44 //"Continue" CFG Value

//533 kHz bit frequency
#define I2C_SR  0xE2 //Start CFG Value
#define I2C_SP  0xD2 //Stop CFG Value
#define I2C_DO  0xC2 //"Do" CFG Value
#define I2C_CO  0xC6 //"Continue" CFG Value


//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

// Removed bus_num because this uses a single bus.
void i2c_init();
void i2c_read_registers(uint8_t slave_addr,
                             uint8_t reg_addr,
                             uint8_t numBytes,
                             uint8_t* spaceToWrite);
void i2c_write_register(uint8_t slave_addr,
                             uint8_t reg_addr,
                             uint8_t reg_setting);
unsigned char i2c_slave_present(unsigned char slave_address);

/**
\}
\}
*/

#endif
