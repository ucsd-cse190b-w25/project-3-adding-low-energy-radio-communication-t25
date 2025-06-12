#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stm32l475xx.h> // Include STM32 hardware definitions

// Define I2C2 Address and Speed
#define LSM6DSL_I2C_ADDR  0x6A  // 7-bit I2C address of LSM6DSL
#define I2C_WRITE         0x00  // Write operation
#define I2C_READ          0x01  // Read operation

// Function to initialize I2C2
void i2c_init(void);

// Function to perform I2C transactions (read/write)
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);

#endif // I2C_H
