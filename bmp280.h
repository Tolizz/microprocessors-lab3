// bmp280 header file
#include <stdint.h>

#ifndef BMP280_H
#define BMP280_H

// REGISTER ADDRESSES

#define BMP280_REG_CHIPID 			0xD0		// 0x58
#define BMP280_REG_RESET				0xE0		// 0xB6
#define BMP280_REG_STATUS				0xF3		
#define BMP280_REG_CTRL_MEAS		0xF4
#define BMP280_REG_CONFIG				0xF5

// Pressure Registers
#define BMP280_REG_PRESS_MSB		0xF7
#define BMP280_REG_PRESS_LSB		0xF8
#define BMP280_REG_PRESS_XLSB		0xF9

// Temprature Registers
#define BMP280_REG_TEMP_MSB			0xFA
#define BMP280_REG_TEMP_LSB			0xFB
#define BMP280_REG_TEMP_XLSB		0xFC

// Calibration data start address 
#define BMP280_REG_CALIB_START	0x88

//I2C Adress
#define BMP280_I2C_ADDR					0x76		// could be 0x77 depending SDO pin connection(SDO to GND-->0x76 / to VDDIO-->0x77)

// Macros for Power Modes
#define BMP280_MODE_SLEEP				0x00		// 00
#define BMP280_MODE_FORCED			0x01		// 01
#define BMP280_MODE_NORMAL			0x03		// 11

// Macros for Oversampling
#define BMP280_OSRS_SKIPPED			0x00		// 000
#define BMP280_OSRS_1X					0x01		// 001
#define BMP280_OSRS_2X					0x02		// 010
#define BMP280_OSRS_4X					0x03		// 011
#define BMP280_OSRS_8X					0x04		// 100
#define BMP280_OSRS_16X					0x05		// 101

// Macros for IIR Filter
#define BMP280_FILTER_OFF				0x00
#define BMP280_FILTER_2					0x01
#define BMP280_FILTER_4					0x02
#define BMP280_FILTER_8					0x03
#define BMP280_FILTER_16				0x04

// Calibration Data

typedef struct{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
} bmp280_calib_data;

// FUNCTIONS
void bmp280_init(void);
void bmp280_read_measurements(int32_t *temprature, uint32_t *pressure);
	
void bmp280_set_mode(uint8_t mode);
void bmp280_set_filter(uint8_t filter);
void bmp280_trigger_forced_measurement(void);

#endif //BMP280_H