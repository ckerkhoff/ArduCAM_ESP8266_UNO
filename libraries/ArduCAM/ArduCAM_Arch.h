#ifndef ArduCAM_ARCH_H
#define ArduCAM_ARCH_H

#ifdef ARCH_ARDUINO
 #include "Arduino.h"
#endif

bool arducam_i2c_init(uint8_t sensor_addr);

// Read/write 8 bit value to/from 8 bit register address
byte wrSensorReg8_8(byte sensor_addr, int regID, int regDat);
byte rdSensorReg8_8(byte sensor_addr, uint8_t regID, uint8_t* regDat);

// Read/write 16 bit value to/from 8 bit register address
byte wrSensorReg8_16(byte sensor_addr, int regID, int regDat);
byte rdSensorReg8_16(byte sensor_addr, uint8_t regID, uint16_t* regDat);

// Read/write 8 bit value to/from 16 bit register address
byte wrSensorReg16_8(byte sensor_addr, int regID, int regDat);
byte rdSensorReg16_8(byte sensor_addr, uint16_t regID, uint8_t* regDat);

//I2C Write 16bit address, 16bit data
byte wrSensorReg16_16(byte sensor_addr, int regID, int regDat);

//I2C Read 16bit address, 16bit data
byte rdSensorReg16_16(byte sensor_addr, uint16_t regID, uint16_t* regDat);

void arducam_spi_init();

void arducam_spi_write(uint8_t address, uint8_t value);
uint8_t arducam_spi_read(uint8_t address);

void arducam_spi_transfer(uint8_t data);
void arducam_spi_transfers(uint8_t *buf, uint32_t size);

void arducam_delay_ms(uint32_t delay_ms);

#endif