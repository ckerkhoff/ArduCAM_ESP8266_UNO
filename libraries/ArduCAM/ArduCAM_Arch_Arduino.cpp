#include "memorysaver.h"

#if defined ( ARCH_ARDUINO )

#include "ArduCAM_Arch.h"
#include <Wire.h>
#if defined(__SAM3X8E__)
	#define Wire Wire1
#endif

bool arducam_i2c_init(uint8_t sensor_addr)
{
	#if defined(__SAM3X8E__)
    	Wire1.begin();
  	#else
    	Wire.begin();
  	#endif
}

// Read/write 8 bit value to/from 8 bit register address
byte wrSensorReg8_8(byte sensor_addr, int regID, int regDat)
{
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID & 0x00FF);
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission())
	{
	    return 0;
	}
	delay(1);
	return 1;
}
byte rdSensorReg8_8(byte sensor_addr, uint8_t regID, uint8_t* regDat)
{
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID & 0x00FF);
	Wire.endTransmission();

	Wire.requestFrom(sensor_addr, 1);
	if (Wire.available())
		*regDat = Wire.read();
	delay(1);
	return 1;
}

// Read/write 16 bit value to/from 8 bit register address
byte wrSensorReg8_16(byte sensor_addr, int regID, int regDat)
{
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID & 0x00FF);

	Wire.write(regDat >> 8);            // sends data byte, MSB first
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission())
	{
	    return 0;
	}
	delay(1);

	return 1;
}
byte rdSensorReg8_16(byte sensor_addr, uint8_t regID, uint16_t* regDat)
{
  	uint8_t temp;
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID);
	Wire.endTransmission();

	Wire.requestFrom(sensor_addr, 2);
	if (Wire.available())
	{
	    temp = Wire.read();
	    *regDat = (temp << 8) | Wire.read();
	}
	delay(1);

  	return 1;
}

// Read/write 8 bit value to/from 16 bit register address
byte wrSensorReg16_8(byte sensor_addr, int regID, int regDat)
{
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID >> 8);            // sends instruction byte, MSB first
	Wire.write(regID & 0x00FF);
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission())
	{
	    return 0;
	}
	delay(1);

	return 1;
}
byte rdSensorReg16_8(byte sensor_addr, uint16_t regID, uint8_t* regDat)
{
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID >> 8);
	Wire.write(regID & 0x00FF);
	Wire.endTransmission();
	Wire.requestFrom(sensor_addr, 1);
	if (Wire.available())
	{
	    *regDat = Wire.read();
	}
	delay(1);

	return 1;
}

//I2C Write 16bit address, 16bit data
byte wrSensorReg16_16(byte sensor_addr, int regID, int regDat)
{
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID >> 8);            // sends instruction byte, MSB first
	Wire.write(regID & 0x00FF);
	Wire.write(regDat >> 8);            // sends data byte, MSB first
	Wire.write(regDat & 0x00FF);
	if (Wire.endTransmission())
	{
	    return 0;
	}
	delay(1);

  	return (1);
}

//I2C Read 16bit address, 16bit data
byte rdSensorReg16_16(byte sensor_addr, uint16_t regID, uint16_t* regDat)
{
	uint16_t temp;
	Wire.beginTransmission(sensor_addr);
	Wire.write(regID >> 8);
	Wire.write(regID & 0x00FF);
	Wire.endTransmission();
	Wire.requestFrom(sensor_addr, 2);
	if (Wire.available())
	{
	    temp = Wire.read();
	    *regDat = (temp << 8) | Wire.read();
	}
	delay(1);

  	return (1);
}

#endif