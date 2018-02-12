#include "memorysaver.h"

#if defined ( RASPBERRY_PI )

#include "ArduCAM_Arch.h"
#include <wiringPiI2C.h>

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

#endif