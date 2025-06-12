#include <stdint.h>
#include <stm32l475xx.h>
#include <stdio.h>

// Define I2C2 Pins and Speed
#define I2C2_SCL_PIN      10  // PB10
#define I2C2_SDA_PIN      11  // PB11
#define I2C_SPEED_400KHZ  0x10110102  // Timing for 400 kHz with 4 MHz PCLK1
// PRESC ( 1 << 28 ) | SCLDEL = 1 << 20 ) | SDADEL ( 1 << 16 ) | SCLH ( 1 << 8 ) | SCLL ( 2 )

// Define I2C Read/Write Directions
#define I2C_WRITE 0
#define I2C_READ  1

// LSM6DSL I2C Address (7-bit)
#define LSM6DSL_I2C_ADDR  0x6A

// Function to initialize I2C2
void i2c_init() {
    // Enable clocks for GPIOB and I2C2
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIOB clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;  // Enable I2C2 clock

    // Reset I2C2 before configuring it
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST;

    // Configure PB10 (SCL) and PB11 (SDA) as Alternate Function mode (AF4 for I2C)
    GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11); // Clear mode
    GPIOB->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1; // Set AF mode

    // Set PB10 and PB11 as open-drain outputs
    GPIOB->OTYPER |= GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11;

//    Enable pull-up resistors for PB10 and PB11
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11); // Clear bits
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0; // Enable pull-ups

    // Set Alternate Function AF4 for I2C2 on PB10 and PB11
    //0xFF is 1111 used for clearing the bits
    //4 is for writing the Alternate Function 4 (AF4) into the Alternate Function Register (AFR) for PB10 and PB11
    GPIOB->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL10_Pos) | (0xF << GPIO_AFRH_AFSEL11_Pos)); // Clear bits
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL10_Pos) | (4 << GPIO_AFRH_AFSEL11_Pos); // Set AF4


    // Configure I2C timing for 400 kHz
    I2C2->TIMINGR = I2C_SPEED_400KHZ;

    // Configure noise filters (default: analog filter enabled)
    I2C2->CR1 &= ~I2C_CR1_ANFOFF; // Enable analog filter
    I2C2->CR1 &= ~I2C_CR1_DNF;    // Disable digital filter (set DNF[3:0] = 0)

    // Enable I2C2 peripheral
    I2C2->CR1 |= I2C_CR1_PE;
}



// Function to perform I2C transactions (Read/Write)
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
    uint32_t timeout = 100000;

    // Clear any previous STOP condition
    // This prevents unexpected behavior in case the previous transmission didn't complete correctly.
    // ICR is the Interrupt Clear register, we have to clear the interrupt for the next interrupt to do its job
    I2C2->ICR |= I2C_ICR_STOPCF;

    // Configure transaction
    // I2C Address is 110101, which is 6 bit, so it needs to be shifted to the left by 1 bit
    I2C2->CR2 = (address << 1) | (dir ? I2C_CR2_RD_WRN : 0) | (len << 16); //if the dir is read mode, set 1, else 0
    I2C2->CR2 |= I2C_CR2_START;  // Start condition

    // If writing, send data
    if (dir == I2C_WRITE) {
//        printf("Starting I2C WRITE Transaction\n");
        for (uint8_t i = 0; i < len; i++) {
            timeout = 10000;
            // Ensures the transmission register is ready
            // Error Message will be print, if timeout
            while (!(I2C2->ISR & I2C_ISR_TXIS) && (--timeout)) {  //Ensures register is ready
                if (timeout % 10000 == 0) printf("Waiting for TXIS... ISR=0x%08lX\n", I2C2->ISR);
            }
            if (timeout == 0) {
                return 1;
            }
            I2C2->TXDR = data[i]; // Send byte, Moves data to the Transmit Data Register
            printf("Sent Byte %d: 0x%02X\n", i, data[i]);
        }

        // Wait for transfer completion
        timeout = 100000;
        while (!(I2C2->ISR & I2C_ISR_TC) && (--timeout));
        if (timeout == 0) return 1;
    }
    // If reading, receive data
    else {
        for (uint8_t i = 0; i < len; i++) {
            timeout = 10000;
            while (!(I2C2->ISR & I2C_ISR_RXNE) && (--timeout)) { //Waits for RXNE (Receive Buffer Not Empty) flag
                if (timeout % 10000 == 0) printf("Waiting for RXNE... ISR=0x%08lX\n", I2C2->ISR);
            }
            if (timeout == 0) {
                return 1;
            }

            data[i] = I2C2->RXDR; // Read byte

            if (i == len - 1) {
                I2C2->CR2 |= I2C_CR2_NACK; // Send NACK on last byte
            }
        }
    }

    // Generate STOP condition
    I2C2->CR2 |= I2C_CR2_STOP;
    return 0;
}
