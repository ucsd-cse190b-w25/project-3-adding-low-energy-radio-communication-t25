/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "ble.h"
//adding project 2 dependencies
#include "i2c.h"
#include "lsm6dsl.h"
//#include "leds.h"
#include "timer.h"   // <-- make sure this file exists and includes declarations for timer functions

/* Include memory map of our MCU */
#include <stm32l475xx.h>

/* Include standard libraries*/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define WHO_AM_I 0x0F
//#define LOST_MODE_TIME 2000 // 1 minute in milliseconds (assuming 1ms per loop)9400 60
#define LOST_MODE_TIME 60000 // 60 seconds
#define MOVEMENT_THRESHOLD 330000000  // Define a threshold to detect movement 330000000   294300000

/*Consider Removing this later*/
volatile uint8_t stop_transmission = 0;  // 0 = blinking, 1 = stop blinking
volatile uint8_t nonDiscoverable = 0;    // 0 = discoverable, 1 = hidden
volatile uint8_t was_lost = 0;
volatile uint32_t last_ble_tick = 0;
volatile uint32_t last_movement_time = 0;  // track last movement time globally
volatile uint32_t lost_time = 0;

// This make the standard C lib write char over the STM32's UART-like character printing bus (called SMV ITM) over USB
// This is our printf
int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}


int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */

void privtag_detect_lost_mode(int16_t x, int16_t y, int16_t z);

int main(void)
{
  /*Setting the priorities of our interrupt*/
/*
	NVIC_SetPriority(TIM2_IRQn, 1);  // Lower priority number = higher priority
	NVIC_SetPriority(I2C2_EV_IRQn, 2);  // Ensure I2C interrupt is lower priority
	NVIC_SetPriority(I2C2_ER_IRQn, 2);
*/

  /*Inits from Project 2*/
	//leds_init();
	i2c_init();
	lsm6dsl_init();
	timer_init(TIM2);

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config(); //print works after configuring clock

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);
  ble_init();
  HAL_Delay(150);
  setDiscoverability(0);
  nonDiscoverable = 1;



  //We SHOULD CHANGE THIS TO BLE instead of LED
  /*defining the register and data*/
    //uint8_t reg = 0x0F;  // WHO_AM_I register
    uint8_t reg = WHO_AM_I; //REG
    uint8_t data = 0x00;

    //printf("Sending Register: 0x%X\n", reg);
    	printf("Hello SWV1\n");
        if (i2c_transaction(LSM6DSL_I2C_ADDR, I2C_WRITE, &reg, 1) != 0) {
                //leds_set(0b10);  // Indicate failure
                while (1);  // Halt execution
            }

            // Perform I2C Read (Read WHO_AM_I response)
        if (i2c_transaction(LSM6DSL_I2C_ADDR, I2C_READ, &data, 1) != 0) {
                printf("I2C Read Failed!\n");
                //leds_set(0b10);  // Indicate failure
                while (1);  // Halt execution
            }

        int16_t x, y, z;



        static uint32_t last_tick = 0;
  while (1)
  {
	  //lsm6dsl_read_xyz(&x, &y, &z);
	  //printf("Acceleration Data - X: %d, Y: %d, Z: %d\n", x, y, z);
	  if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
	    catchBLE();

	  }else{
//		  HAL_Delay(1000);
//		  //directly giving the address of the x,y,z so that the function can modify it
//		  lsm6dsl_read_xyz(&x, &y, &z);
//		  printf("Acceleration Data - X: %d, Y: %d, Z: %d\n", x, y, z);
//
//		  /* CHANGE THIS*/
//		  // Detect lost mode
//		  privtag_detect_lost_mode(x, y, z);








		  if (HAL_GetTick() - last_tick >= 1000) {  // 1 second has passed
		      last_tick = HAL_GetTick();

		      lsm6dsl_read_xyz(&x, &y, &z);
		      printf("Acceleration Data - X: %d, Y: %d, Z: %d\n", x, y, z);
		      privtag_detect_lost_mode(x, y, z);

		      if (was_lost) {
		    	  lost_time += 1;

		      }
		  }

//		  // Send a string to the NORDIC UART service, remember to not include the newline
//		  unsigned char test_str[] = "youlostit BLE test";
//		  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(test_str)-1, test_str);

		  if (!stop_transmission && was_lost && HAL_GetTick() - last_ble_tick >= 10000) {
		      last_ble_tick = HAL_GetTick();
		      char msg1[] = "PrivTag missing ";

		      char msg2[50];  // allocate enough space
		      snprintf(msg2, sizeof(msg2), "for %d seconds", lost_time - 1);

		      updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(msg1), (uint8_t*)msg1);
		      HAL_Delay(50);
		      updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(msg2), (uint8_t*)msg2);

//		      if (notifications_enabled) {
//		              char msg1[] = "PrivTag BLE";
//		              char msg2[] = " missing for 60s";
//
//		              updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(msg1), (uint8_t*)msg1);
//		              HAL_Delay(50);
//		              updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(msg2), (uint8_t*)msg2);
//		      }
		  }


	  }
	  // Wait for interrupt, only uncomment if low power is needed
	  //__WFI();
  }






}


/*-----------------------------------------------------------------------------------------------------------*/
//WE NEED TO CHANGE THIS AS WELL
// Lost mode detection function
void privtag_detect_lost_mode(int16_t x, int16_t y, int16_t z) {


	int64_t magnitude = (int64_t)x * x + (int64_t)y * y + (int64_t)z * z;

		uint32_t current_time = HAL_GetTick();

		if (!was_lost){
			setDiscoverability(0);
		}

		if (magnitude > MOVEMENT_THRESHOLD) {
		    if (was_lost) {
		        printf("Motion resumed! Disconnecting BLE\n");
		        catchBLE();
		        // Disconnect and make non-discoverable
		        disconnectBLE();

		        HAL_Delay(150);  // Wait ~100ms for BLE stack to clean up
		        setDiscoverability(0);

		        was_lost = 0;
		        nonDiscoverable = 1;

		        lost_time = 0;
		    }

		    last_movement_time = current_time;
		    stop_transmission = 1;
		    printf("movement detected\n");


		} else {
			current_time = HAL_GetTick();
			printf("was lost: %d; nondiscoverable: %d; current time: %d; last_movement_time: %d; lost mode time: %d ; lost time: %d \n", was_lost, nonDiscoverable, current_time, last_movement_time, LOST_MODE_TIME, lost_time);
		    if (!was_lost && (current_time - last_movement_time >= LOST_MODE_TIME)) {
		        printf("Device Lost! Entering lost mode...\n");
		        stop_transmission = 0;
		        was_lost = 1;

		        // Make discoverable again so phone can connect
		        // Start message timer
		        timer_reset(TIM2);
		        printf("checking 1 \n");
		        timer_set_ms(TIM2, 50);
		        printf("checking 2\n");
		        setDiscoverability(1);
		        printf("checking 3 \n");
		        HAL_Delay(150);  // Wait ~100ms for BLE stack to clean up
		        nonDiscoverable = 0;
		        last_ble_tick = HAL_GetTick();
		        printf("checking \n");
		    }
		}
//
//	int64_t magnitude = (int64_t)x * x + (int64_t)y * y + (int64_t)z * z;
//	static uint32_t lost_counter = 0;
//	static uint8_t nonDiscoverable = 0;
//	static uint8_t was_lost = 0;
//
//	if (magnitude > MOVEMENT_THRESHOLD) {
//	    if (was_lost) {
//	        printf("Motion resumed! Disconnecting BLE\n");
//	        // TODO: disconnect_ble_client(); make_non_discoverable();
//	        was_lost = 0;
//	    }
//
//	    lost_counter = 0;
//	    //leds_set(0b00);
//	    stop_transmission = 1;
//	    printf("movement detected\n");
//	} else {
//	    lost_counter++;
//
//	    if (lost_counter == LOST_MODE_TIME) {
//	        printf("Device Lost! Entering lost mode...\n");
//	        stop_transmission = 0;
//	        was_lost = 1;
//
//	        // TODO: make_discoverable();
//	        timer_reset(TIM2);
//	        timer_set_ms(TIM2,50);
//	    }
//	}





//    static uint32_t lost_counter = 0;
//
//    // Compute acceleration magnitude
//    int64_t magnitude = (int64_t)x * (int64_t)x +
//                                (int64_t)y * (int64_t)y +
//                                (int64_t)z * (int64_t)z;
//
//    printf("lost_counter %d\n", lost_counter);
//
//    // Detect movement
//    if (magnitude > MOVEMENT_THRESHOLD) {
//        lost_counter = 0;  // Reset counter if movement is detected
//        leds_set(0b00);  // Indicate active mode
//        stop_transmission = 1;
//        printf("movement detected\n");
//    } else {
//        lost_counter++;  // Increment counter if no movement
//    }
//
//    // Check if device has been still for 1 minute
//    if (lost_counter >= LOST_MODE_TIME) {
//        printf("Device Lost! Entering lost mode...\n");
//        stop_transmission = 0;
//
//        if (lost_counter == LOST_MODE_TIME){
//        	printf("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
//        	timer_reset(TIM2);
//        	timer_set_ms(TIM2,50);
//        }
//    }

    //delay(1);  // Small delay to ensure accurate 1-minute timing
}





//Here we are printing the preamble when the MCU has been still for 1 minute
//Instead of printing through led we should use the ble
// Define the binary sequences for the preamble and student ID
const uint8_t preamble[] = {0b10, 0b01, 0b10, 0b01};  // 8 bits in 2-bit chunks
const uint8_t student_id[] = {0b00, 0b00, 0b01, 0b10, 0b01, 0b11, 0b00, 0b01};  // 16 bits for 11001110001

volatile uint8_t current_index = 0;  // Index of the current bit pair
volatile uint8_t transmitting_sid = 0;  // 0 = transmitting preamble, 1 = transmitting student ID

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {  // Check if the interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF;  // Clear the interrupt flag

        if (stop_transmission) {
        //            leds_set(0b00); // Turn off LED
                    TIM2->CR1 &= ~TIM_CR1_CEN; // Disable TIM2 Timer
                    return;
        } else {
        	TIM2->CR1 |= TIM_CR1_CEN;
        }

        if (!transmitting_sid) {
            // Transmit preamble
       //     leds_set(preamble[current_index]);
            current_index++;

            if (current_index >= sizeof(preamble)) {
                current_index = 0;       // Reset index for student ID
                transmitting_sid = 1;   // Start transmitting student ID
            }
        } else {
            // Transmit student ID
        //    leds_set(student_id[current_index]);
            current_index++;

            if (current_index >= sizeof(student_id)) {
                current_index = 0;       // Reset index for preamble
                transmitting_sid = 0;   // Restart preamble transmission
            }
        }
    }
}

/*-----------------------------------------------------------------------------------------------------------*/



/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
