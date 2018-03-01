#ifndef ArduCAM_ARCH_H
#define ArduCAM_ARCH_H

#ifdef ARCH_ARDUINO
 #include "Arduino.h"
#endif

#if defined (__AVR__)
    #define cbi(reg, bitmask) *reg &= ~bitmask
    #define sbi(reg, bitmask) *reg |= bitmask
    #define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
    #define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
    #define cport(port, data) port &= data
    #define sport(port, data) port |= data
    #define swap(type, i, j) {type t = i; i = j; j = t;}
    #define fontbyte(x) pgm_read_byte(&cfont.font[x])
    #define regtype volatile uint8_t
    #define regsize uint8_t
#endif

#if defined(__SAM3X8E__)
    #define cbi(reg, bitmask) *reg &= ~bitmask
    #define sbi(reg, bitmask) *reg |= bitmask
    #define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
    #define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
    #define cport(port, data) port &= data
    #define sport(port, data) port |= data
    #define swap(type, i, j) {type t = i; i = j; j = t;}
    #define fontbyte(x) cfont.font[x]
    #define regtype volatile uint32_t
    #define regsize uint32_t
    #define PROGMEM
    #define pgm_read_byte(x)        (*((char *)x))
    #define pgm_read_word(x)        ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
    #define pgm_read_byte_near(x)   (*((char *)x))
    #define pgm_read_byte_far(x)    (*((char *)x))
    #define pgm_read_word_near(x)   ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
    #define pgm_read_word_far(x)    ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x))))
    #define PSTR(x)  x
    #if defined F
	    #undef F
    #endif
    #define F(X) (X)
#endif

#if defined(ESP8266)
	#define cbi(reg, bitmask) digitalWrite(bitmask, LOW)
	#define sbi(reg, bitmask) digitalWrite(bitmask, HIGH)
	#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
	#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
	#define cport(port, data) port &= data
	#define sport(port, data) port |= data
	#define swap(type, i, j) {type t = i; i = j; j = t;}
	#define fontbyte(x) cfont.font[x]
	#define regtype volatile uint32_t
	#define regsize uint32_t
#endif

#if defined(__CPU_ARC__)
	#define cbi(reg, bitmask) *reg &= ~bitmask
	#define sbi(reg, bitmask) *reg |= bitmask
	#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
	#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);
	#define cport(port, data) port &= data
	#define sport(port, data) port |= data
	#define swap(type, i, j) {type t = i; i = j; j = t;}
	#define fontbyte(x) pgm_read_byte(&cfont.font[x])
	#define regtype volatile uint32_t
	#define regsize uint32_t
#endif

#if defined (RASPBERRY_PI)
	#define regtype volatile uint32_t
	#define regsize uint32_t
	#define byte uint8_t
	#define cbi(reg, bitmask) digitalWrite(bitmask, LOW)
    #define sbi(reg, bitmask) digitalWrite(bitmask, HIGH)
    #define PROGMEM
	//#define pgm_read_byte(x)        (*((char *)x))
	//#define pgm_read_word(x)        ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
	//#define pgm_read_byte_near(x)   (*((char *)x))
	//#define pgm_read_byte_far(x)    (*((char *)x))
	//#define pgm_read_word_near(x)   ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
	//#define pgm_read_word_far(x)    ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x))))

	#define PSTR(x)  x
	#if defined F
	    #undef F
	#endif
	#define F(X) (X)
#endif

class ArduCAM_Arch
{
  public:
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
};

#endif