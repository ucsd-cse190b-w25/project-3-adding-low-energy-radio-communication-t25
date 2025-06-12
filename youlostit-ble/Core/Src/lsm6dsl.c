#include <stdint.h>
#include <stm32l475xx.h>
#include <i2c.h>
#include <stdio.h>

#define LSM6DSL_ADDR 0x6A // 7-bit I2C address
#define WHO_AM_I_REG  0x0F // Ensures we are communicating with the correct sensor
#define CTRL1_XL      0x10 // Sets accelerometer sampling rate (416Hz) and full-scale range (±2g)
#define CTRL3_C       0x12 // Enables Block Data Update (BDU) and auto-increment
#define OUTX_L_XL 	  0x28 // Address of the Lower byte of X-axis acceleration

/*
 * We need CTRL3_C b/c
 * When BDU (Block Data Update) is enabled,
 * the sensor locks both bytes until the entire value is read
 */

// This function initializes the LSM6DSL accelerometer
// We do not need the gyroscope bc we don't need rotation data

void lsm6dsl_init() {
    uint8_t data[2]; // 2-byte array that can store two separate 8-bit values


    // Step 1: Enable Accelerometer (CTRL1_XL)
    // First byte: Register address
    data[0] = CTRL1_XL;
    // Second byte: Value to enable accelerometer at 416 Hz (High-Performance mode)
    data[1] = 0x60;

    // Perform I2C write transaction
    if (i2c_transaction(LSM6DSL_I2C_ADDR, I2C_WRITE, data, 2) != 0) {
    		return;
    }

    // // Step 2: Configure CTRL3_C for BDU and Auto-increment
    // First byte: Register address
    data[0] = CTRL3_C;
    // Second byte: Value to enable accelerometer at 416 Hz (High-Performance mode)
    data[1] = 0x44; // Enable Block Data Update (BDU) and Auto-increment
    // Perform I2C write transaction
    if (i2c_transaction(LSM6DSL_I2C_ADDR, I2C_WRITE, data, 2) != 0) {
            return;
    }
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z){
	uint8_t reg = OUTX_L_XL;  // Start reading from the X-axis low byte
	uint8_t data[6];          // 6 bytes for X, Y, and Z (each 16-bit)

	// Request data from accelerometer
	if (i2c_transaction(LSM6DSL_I2C_ADDR, I2C_WRITE, &reg, 1) != 0) {
	    return;
	}

	// Read 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
	if (i2c_transaction(LSM6DSL_I2C_ADDR, I2C_READ, data, 6) != 0) {
	    return;
	}

	// Convert raw data to signed 16-bit values
	*x = (int16_t)(data[1] << 8 | data[0]); // X-axis
	*y = (int16_t)(data[3] << 8 | data[2]); // Y-axis
	*z = (int16_t)(data[5] << 8 | data[4]); // Z-axis

	printf("Acceleration: X=%d, Y=%d, Z=%d\n", *x, *y, *z);
}
