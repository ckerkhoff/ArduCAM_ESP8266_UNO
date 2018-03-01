#include "memorysaver.h"

#if defined ( ARCH_ARDUINO )

#include "ArduCAM_Arch.h"
#include <SPI.h>
#include <Wire.h>
#if defined(__SAM3X8E__)
	#define Wire Wire1
#endif

int CS_PIN;
regtype *P_CS;
regsize B_CS;

void ArduCAM_Arch::set_spi_cs(int CS)
{
	CS_PIN = CS;
	#if defined(ESP8266)
		B_CS = CS;
	#else
		P_CS  = portOutputRegister(digitalPinToPort(CS));
		B_CS  = digitalPinToBitMask(CS);
	#endif
}

void ArduCAM_Arch::spi_cs_low()
{
	cbi(P_CS, B_CS);
}

void ArduCAM_Arch::spi_cs_high()
{
	sbi(P_CS, B_CS);
}

bool ArduCAM_Arch::arducam_i2c_init(uint8_t sensor_addr)
{
	#if defined(__SAM3X8E__)
    	Wire1.begin();
  	#else
    	Wire.begin();
  	#endif
}

void ArduCAM_Arch::arducam_spi_init()
{
	pinMode(CS_PIN, OUTPUT);
	sbi(P_CS, B_CS);

	// initialize SPI:
	SPI.begin();
	SPI.setFrequency(4000000); //4MHz
}

// Read/write 8 bit value to/from 8 bit register address
byte ArduCAM_Arch::wrSensorReg8_8(byte sensor_addr, int regID, int regDat)
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
byte ArduCAM_Arch::rdSensorReg8_8(byte sensor_addr, uint8_t regID, uint8_t* regDat)
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
byte ArduCAM_Arch::wrSensorReg8_16(byte sensor_addr, int regID, int regDat)
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
byte ArduCAM_Arch::rdSensorReg8_16(byte sensor_addr, uint8_t regID, uint16_t* regDat)
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
byte ArduCAM_Arch::wrSensorReg16_8(byte sensor_addr, int regID, int regDat)
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
byte ArduCAM_Arch::rdSensorReg16_8(byte sensor_addr, uint16_t regID, uint8_t* regDat)
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
byte ArduCAM_Arch::wrSensorReg16_16(byte sensor_addr, int regID, int regDat)
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
byte ArduCAM_Arch::rdSensorReg16_16(byte sensor_addr, uint16_t regID, uint16_t* regDat)
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

void ArduCAM_Arch::arducam_spi_write(uint8_t address, uint8_t value)
{
	cbi(P_CS, B_CS);

	SPI.transfer(address);
	SPI.transfer(value);

	sbi(P_CS, B_CS);
}

uint8_t ArduCAM_Arch::arducam_spi_read(uint8_t address)
{
	uint8_t value;

	cbi(P_CS, B_CS);

	SPI.transfer(address);
	value = SPI.transfer(0x00);

	sbi(P_CS, B_CS);

	return value;
}

void ArduCAM_Arch::arducam_spi_transfer(uint8_t data)
{
	SPI.transfer(data);
}

void ArduCAM_Arch::arducam_spi_transfers(uint8_t *buf, uint32_t size)
{
}

void ArduCAM_Arch::arducam_delay_ms(uint32_t delay_ms)
{
	delay(delay_ms);
}

#endif