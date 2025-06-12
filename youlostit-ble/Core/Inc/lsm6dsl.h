#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <stdint.h>
#include <stm32l475xx.h> // Include STM32 hardware definitions

// Function to initialize lsm6dsl
void lsm6dsl_init();

// Function to read lsm6dsl's xyz values
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);

#endif // LSM6DSL_H
