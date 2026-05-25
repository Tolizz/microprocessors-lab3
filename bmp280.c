#include "bmp280.h"
#include "i2c.h"

// Global variables
bmp280_calib_data calib_data;
// t_fine carries fine temprature as global value
int32_t t_fine;

// ==========================================
// I2C WRAPPERS
// ==========================================

void bmp280_write_register(uint8_t reg_addr, uint8_t data){

	uint8_t buffer[2];
	buffer[0] = reg_addr;
	buffer[1] = data;
	
	// shift address 1 bit left 
	i2c_write((BMP280_I2C_ADDR << 1), buffer, 2);
	
}

// reads 'len' bytes starting from 'reg_addr' (Burst Read)
void bmp280_read_registers(uint8_t reg_addr, uint8_t *data, int len){
	// Send address of register we want to read
	i2c_write((BMP280_I2C_ADDR << 1), &reg_addr, 1);
	
	// Read data
	i2c_read((BMP280_I2C_ADDR << 1), data, len);
}

// ==========================================
// Functions
// ==========================================

// Change Power Mode (bits 1 and 0 of register 0xF4)
void bmp_set_mode(uint8_t mode){
	uint8_t ctrl_meas;
	bmp280_read_registers(BMP280_REG_CTRL_MEAS, &ctrl_meas, 1);
	ctrl_meas = (ctrl_meas & ~0x03) | (mode & 0x03);
	bmp280_write_register(BMP280_REG_CTRL_MEAS, ctrl_meas);	
}


// Change IIR filter (bits 4,3 and 2 of register 0xF5)
void bmp_set_filter(uint8_t filter){
	uint8_t config;
	bmp280_read_registers(BMP280_REG_CONFIG, &config, 1);
	config = (config & ~0x1C) | ((filter << 2) & 0x1C);
	bmp280_write_register(BMP280_REG_CONFIG, config);
}

// enable forced mode to get a single measurement
void bmp280_trigger_forced_measurement(void){
	bmp280_set_mode(BMP280_MODE_FORCED);
}

// Read calibration data
void bmp280_read_calibration(void){
	uint8_t calib[24];
	
	// Burst read 24 bytes starting from 0x88
	bmp280_read_registers(BMP280_REG_CALIB_START, calib, 24);
	
	// connect LSB and MSB
	calib_data.dig_T1 = (calib[1] << 8) | calib[0];
	calib_data.dig_T2 = (calib[3] << 8) | calib[2];
	calib_data.dig_T3 = (calib[5] << 8) | calib[4];
	
	calib_data.dig_P1 = (calib[7] << 8) | calib[6];
	calib_data.dig_P2 = (calib[9] << 8) | calib[8];
	calib_data.dig_P3 = (calib[11] << 8) | calib[10];
	calib_data.dig_P4 = (calib[13] << 8) | calib[12];
	calib_data.dig_P5 = (calib[15] << 8) | calib[14];
	calib_data.dig_P6 = (calib[17] << 8) | calib[16];
	calib_data.dig_P7 = (calib[19] << 8) | calib[18];
	calib_data.dig_P8 = (calib[21] << 8) | calib[20];
	calib_data.dig_P9 = (calib[23] << 8) | calib[22];
}

// initialize bmp280
void bmp280_init(void){
	
	uint8_t chip_id;	
	i2c_init();
	
	bmp280_read_registers(BMP280_REG_CHIPID, &chip_id, 1);
	if(chip_id != 0x58){
		return; // Fail - Wrong CHIP ID
	}
	
	// read calibration
	bmp280_read_calibration();
	
	// Basic initialization settings
	// Oversampling Temp x2, Press x16, Normal Mode
	uint8_t ctrl_meas_val = (BMP280_OSRS_2X << 5) | (BMP280_OSRS_16X << 2) | BMP280_MODE_NORMAL;
	bmp280_write_register(BMP280_REG_CTRL_MEAS, ctrl_meas_val);
	
	// Standby 1000ms (5 << 5), IIR Filter OFF (00)
	uint8_t config_val = (5 << 5) | (BMP280_FILTER_OFF << 2);
	bmp280_write_register(BMP280_REG_CONFIG, config_val);
}

// ==========================================
// CALCULATE AND READ FUNCTIONS (Datasheet)
// ==========================================

// Returns temprature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
int32_t bmp280_compensate_T_int32(int32_t adc_T){
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)calib_data.dig_T1<<1))) * ((int32_t)calib_data.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calib_data.dig_T1)) * ((adc_T>>4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integers and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPA

uint32_t bmp280_compensate_P_int32(int32_t adc_P){
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_data.dig_P6);
	var2 = var2 + ((var1 * ((int32_t)calib_data.dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)calib_data.dig_P4) << 16);
	var1 = (((calib_data.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_data.dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)calib_data.dig_P1)) >> 15);
	if (var1 == 0) { return 0; } // avoid exception caused by division by zero
	p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)var1);
	} else {
		p = (p / (uint32_t)var1) * 2;
	}
	
	var1 = (((int32_t)calib_data.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_data.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + calib_data.dig_P7) >> 4));
	return p;
}

// Basic function called in main()
void bmp280_read_measurements(int32_t *temprature, uint32_t *pressure){
	uint8_t data[6];
	
	// Burst read 6 bytes (Pressure and Temprature MSB,LSB,XLSB)
	bmp280_read_registers(BMP280_REG_PRESS_MSB, data, 6);
	
	// Rebuild 20-bit raw values
	int32_t raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	int32_t raw_temprature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
	
	// calculate final values through compensation calculations
	*temprature = bmp280_compensate_T_int32(raw_temprature);
	*pressure = bmp280_compensate_P_int32(raw_pressure);
}
