//=========================== includes ===========================================

#include <iocc2533.h>
#include "i2c.h"

//=========================== variables ===========================================

uint8_t dev_addr_write;
uint8_t dev_addr_read;

//=========================== public ==============================================

//Small function for waiting until I2CSTAT matches status_bits
void waitI2CStat(uint8_t status_bits)
{
while (I2CSTAT != status_bits);
}

void i2c_init(){
}

void i2c_read_registers(uint8_t slave_addr,uint8_t reg_addr,uint8_t numBytes, uint8_t* spaceToWrite)
{
//Find device addresses
dev_addr_write = (slave_addr << 1);
dev_addr_read = (slave_addr << 1)|0x01;
 
// Sent start condition and wait for it to be received
I2CCFG = I2C_SR;
waitI2CStat(SR_SENT);

// Send Device Address
I2CDATA = dev_addr_write;
I2CCFG = I2C_DO;
waitI2CStat(SLAW_ACK_SENT);

// Send Register address
I2CDATA = reg_addr;
I2CCFG = I2C_DO;
waitI2CStat(DATA_ACK_SENT);
      
// Send Restart condition
I2CCFG = I2C_SR;
waitI2CStat(RS_SENT);

// Send Device Address Read
I2CDATA = dev_addr_read;
I2CCFG = I2C_DO;
waitI2CStat(SLAR_ACK_SENT);

while(numBytes>1)
{
// Do Continued Transfer
I2CCFG=I2C_CO;
waitI2CStat(DATA_ACK_RECV);
*spaceToWrite = I2CDATA;
spaceToWrite++;
numBytes--;
}

//Do Final Transfer
I2CCFG=I2C_DO;
waitI2CStat(DATA_NACK_RECV);
*spaceToWrite = I2CDATA;

// Send Stop Condition
I2CCFG=I2C_SP;
}

void i2c_write_register(uint8_t slave_addr, uint8_t reg_addr, uint8_t reg_setting)
{
//Find device address write:
dev_addr_write = slave_addr<<1;

// Sent start condition and wait for it to be received
I2CCFG = I2C_SR;
waitI2CStat(SR_SENT);

// Send Device Address
I2CDATA = dev_addr_write;
I2CCFG = I2C_DO;
waitI2CStat(SLAW_ACK_SENT);

// Send Register address
I2CDATA = reg_addr;
I2CCFG = I2C_DO;
waitI2CStat(DATA_ACK_SENT);


// Send Register Value
I2CDATA = reg_setting;
I2CCFG = I2C_DO;
waitI2CStat(DATA_ACK_SENT);

// Send Stop Condition
I2CCFG=I2C_SP;
}

unsigned char i2c_slave_present(unsigned char slave_address){
//Find device address write:
dev_addr_write = slave_address<<1;

// Sent start condition and wait for it to be received
I2CCFG = I2C_SR;
waitI2CStat(SR_SENT);

// Send Device Address
I2CDATA = dev_addr_write;
I2CCFG = I2C_DO;
if (I2CSTAT == SLAW_ACK_SENT){
	return 0x01;
}else{
	return 0x00;
}
}