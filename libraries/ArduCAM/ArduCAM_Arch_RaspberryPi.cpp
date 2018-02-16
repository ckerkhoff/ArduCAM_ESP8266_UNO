#include "memorysaver.h"

#if defined ( RASPBERRY_PI )

#include "ArduCAM_Arch.h"
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>

bool arducam_i2c_init(uint8_t sensor_addr)
{
    return wiringPiI2CSetup(sensor_addr) != -1;
}

// Read/write 8 bit value to/from 8 bit register address	
byte wrSensorReg8_8(byte sensor_addr, int regID, int regDat)
{
	arducam_i2c_write( regID , regDat );
	return 1;
}
byte rdSensorReg8_8(byte sensor_addr, uint8_t regID, uint8_t* regDat)
{
	arducam_i2c_read(regID,regDat);
	return 1;
}

// Read/write 16 bit value to/from 8 bit register address
byte wrSensorReg8_16(byte sensor_addr, int regID, int regDat)
{
	arducam_i2c_write16(regID, regDat );
	return 1;
}
byte rdSensorReg8_16(byte sensor_addr, uint8_t regID, uint16_t* regDat)
{
  	arducam_i2c_read16(regID, regDat);
  	return 1;
}

// Read/write 8 bit value to/from 16 bit register address
byte wrSensorReg16_8(byte sensor_addr, int regID, int regDat)
{
	arducam_i2c_word_write(regID, regDat);
	//arducam_delay_ms(1);

	return 1;
}
byte rdSensorReg16_8(byte sensor_addr, uint16_t regID, uint8_t* regDat)
{
	arducam_i2c_word_read(regID, regDat );

	return 1;
}

//I2C Write 16bit address, 16bit data
byte wrSensorReg16_16(byte sensor_addr, int regID, int regDat)
{
    return (1);
}

//I2C Read 16bit address, 16bit data
byte rdSensorReg16_16(byte sensor_addr, uint16_t regID, uint16_t* regDat)
{
    return (1);
}

void arducam_spi_write(uint8_t address, uint8_t value)
{
    uint8_t spiData[2];
	spiData[0] = address | 0x80;
    spiData[1] = value;
	wiringPiSPIDataRW(SPI_ARDUCAM, spiData, 2);
}

uint8_t arducam_spi_read(uint8_t address)
{
	uint8_t spiData[2];
	spiData[0] = address & 0x7F;
	spiData[1] = 0x00;
  	wiringPiSPIDataRW(SPI_ARDUCAM, spiData, 2);

  	return spiData[1];
}

void arducam_spi_transfer(uint8_t data)
{
	uint8_t spiData [1];
	spiData [0] = data;
	wiringPiSPIDataRW(SPI_ARDUCAM, spiData, 1);
}

void arducam_spi_transfers(uint8_t *buf, uint32_t size)
{
	wiringPiSPIDataRW(SPI_ARDUCAM, buf, size);
}

void arducam_delay_ms(uint32_t delay_ms)
{
	usleep(1000*delay_ms);
}

#endif