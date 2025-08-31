/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "nrf24l01p.h"
#include "MPU6050.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buff[1] = {0};
uint8_t led = 0;
uint16_t ledTimer = 0;
uint16_t uartTimer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// for rx interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void nrf24l01p_dump_registers(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Initialization();
  nrf24l01p_tx_init(2500, _250kbps);

  uint8_t en_aa = read_register(NRF24L01P_REG_EN_AA);
  en_aa &= ~(1 << 0);           // Disable AA on pipe 0
  write_register(NRF24L01P_REG_EN_AA, en_aa);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // transmit
	  if(MPU6050_DataReady() == 1)
		{
		    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			MPU6050_ProcessData(&MPU6050);
			uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = {MPU6050.acc_x_raw & 0xFF, (MPU6050.acc_x_raw >> 8) & 0xFF, MPU6050.acc_y_raw & 0xFF, (MPU6050.acc_y_raw >> 8) & 0xFF, MPU6050.acc_z_raw & 0xFF, (MPU6050.acc_z_raw >> 8) & 0xFF,
														MPU6050.temperature_raw & 0xFF, (MPU6050.temperature_raw >> 8) & 0xFF,
														MPU6050.gyro_x_raw & 0xFF, (MPU6050.gyro_x_raw >> 8) & 0xFF, MPU6050.gyro_y_raw & 0xFF, (MPU6050.gyro_y_raw >> 8) & 0xFF, MPU6050.gyro_z_raw & 0xFF, (MPU6050.gyro_z_raw >> 8) & 0xFF
													   };
			nrf24l01p_tx_transmit(tx_data);
		}


//	  uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH];
//		nrf24l01p_tx_transmit(tx_data);
//
//		  // for serial monitor
//		HAL_UART_Transmit_IT(&huart1, tx_data, 14);
//		HAL_Delay(100);

	  // LED loop always runs
	  ledTimer ++;
	  if(ledTimer > 200){
		  nrf24l01p_dump_registers();
//		  HAL_UART_Transmit_IT(&huart1, &status, 1);
		  ledTimer = 0;
		  led = !led;
		  if(led){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		  }else{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  }
	  }

	  // if failed to send message, then reset the radio
	  if (tx_failed) {
		  tx_failed = false;
		  nrf24l01p_tx_init(2500, _250kbps);
	  }

	  // if radio lost power, reset the radio
	  uint8_t rf_ch = read_register(NRF24L01P_REG_RF_CH);
	  if (rf_ch != 0x64) {  // 0x64 is your desired channel
	      nrf24l01p_tx_init(2500, _250kbps);

	      uint8_t en_aa = read_register(NRF24L01P_REG_EN_AA);
	      en_aa &= ~(1 << 0);           // Disable AA on pipe 0
	      write_register(NRF24L01P_REG_EN_AA, en_aa);
	  }

	  uint8_t status = read_register(NRF24L01P_REG_STATUS);
	  if (status == 0xFF || status == 0x00) {
	      // Radio is not responding
	      nrf24l01p_tx_init(2500, _250kbps);

	      uint8_t en_aa = read_register(NRF24L01P_REG_EN_AA);
	      en_aa &= ~(1 << 0);           // Disable AA on pipe 0
	      write_register(NRF24L01P_REG_EN_AA, en_aa);
	  }

	  uint8_t observe_tx = read_register(NRF24L01P_REG_OBSERVE_TX);
	  uint8_t plos_cnt = (observe_tx >> 4) & 0x0F;

	  if (plos_cnt > 0) {
	      // Transmission failed, reinit radio
		  nrf24l01p_tx_init(2500, _250kbps);

		  uint8_t en_aa = read_register(NRF24L01P_REG_EN_AA);
		  en_aa &= ~(1 << 0);           // Disable AA on pipe 0
		  write_register(NRF24L01P_REG_EN_AA, en_aa);
	  }

	  HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//		  HAL_UART_Receive_IT(&huart1, rx_buff, 1);
//		  HAL_UART_Transmit_IT(&huart1, rx_buff, 1);
//	}

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{

		if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER) {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			nrf24l01p_tx_irq(); // clear interrupt flag
//		    uint8_t success_arr[] = {'y','\n'};
//		    HAL_UART_Transmit_IT(&huart1, success_arr, 2);
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		}


	}

	void nrf24l01p_dump_registers(void) {
//	    char buf[64];
//	    uint8_t val[5];
//
//	    // CONFIG (0x00)
//	    snprintf(buf, sizeof(buf), "CONFIG     = 0x%02X\r\n", read_register(0x00));
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//	    // RF_CH (0x05)
//	    snprintf(buf, sizeof(buf), "RF_CH      = 0x%02X\r\n", read_register(0x05));
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//	    // RF_SETUP (0x06)
//	    snprintf(buf, sizeof(buf), "RF_SETUP   = 0x%02X\r\n", read_register(0x06));
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//	    // STATUS (0x07)
//	    snprintf(buf, sizeof(buf), "STATUS     = 0x%02X\r\n", read_register(0x07));
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//	    // FIFO_STATUS (0x17)
//	    snprintf(buf, sizeof(buf), "FIFO_STAT  = 0x%02X\r\n", read_register(0x17));
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//	    // TX_ADDR (0x10) - 5 bytes
//	    read_register(0x10, val, 5);
//	    snprintf(buf, sizeof(buf), "TX_ADDR    = %02X %02X %02X %02X %02X\r\n", val[0], val[1], val[2], val[3], val[4]);
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//	    // RX_ADDR_P0 (0x0A) - 5 bytes
//	    read_register(0x0A, val, 5);
//	    snprintf(buf, sizeof(buf), "RX_ADDR_P0 = %02X %02X %02X %02X %02X\r\n", val[0], val[1], val[2], val[3], val[4]);
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
//
//	    // RX_ADDR_P1 (0x0B) - 5 bytes
//	    read_register(0x0B, val, 5);
//	    snprintf(buf, sizeof(buf), "RX_ADDR_P1 = %02X %02X %02X %02X %02X\r\n", val[0], val[1], val[2], val[3], val[4]);
//	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    char buf[64];

	    // CONFIG (0x00)
	    snprintf(buf, sizeof(buf), "CONFIG     = 0x%02X\r\n", read_register(0x00));
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    // RF_CH (0x05)
	    snprintf(buf, sizeof(buf), "RF_CH      = 0x%02X\r\n", read_register(0x05));
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    // RF_SETUP (0x06)
	    snprintf(buf, sizeof(buf), "RF_SETUP   = 0x%02X\r\n", read_register(0x06));
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    // STATUS (0x07)
	    snprintf(buf, sizeof(buf), "STATUS     = 0x%02X\r\n", read_register(0x07));
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    // FIFO_STATUS (0x17)
	    snprintf(buf, sizeof(buf), "FIFO_STAT  = 0x%02X\r\n", read_register(0x17));
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    // TX_ADDR (0x10) - 5 bytes
	    snprintf(buf, sizeof(buf), "TX_ADDR    = 0x%02X\r\n", read_register(0x10));
	    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	    snprintf(buf, sizeof(buf), "RX_ADDR_P0    = 0x%02X\r\n", read_register(0x0A));
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

		snprintf(buf, sizeof(buf), "RX_ADDR_P1    = 0x%02X\r\n", read_register(0x0B));
		HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	}

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
#ifdef USE_FULL_ASSERT
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
