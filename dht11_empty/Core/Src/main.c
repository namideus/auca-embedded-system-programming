/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Thermometer project based on stm32l053-disco (with E-paper display) and sensors DHT11/DHT22
  * Class: Embedded System Programming, AUCA, 2023
  * Instructor: Boris Kuznetsov
  * Student: Yiman Altynbek uulu
  * Department: Applied Mathematics and Informatics, AMI-19
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "picture.h"
#include "gde021a1.h"
#include "stm32l0538_discovery.h"
#include "stm32l0538_discovery_epd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	IMPLEMENTATION	LIFO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// DHT sensor configs
//static DHT_sensor sensor = {GPIOA, GPIO_PIN_4, DHT22, GPIO_PULLUP};
char msg[40];
DHT_data d;

float temp, hum, avg_temp, temp_sum;
uint8_t i, n;
float input[] = {0,0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LoopDelay(volatile uint32_t n) {
	while(n > 0) n--;
}

static void SPIx_Init(void) {

    /* On STM32L0538-DISCO, EPD ID cannot be read then keep a common configuration */
    /* for EPD (SPI_DIRECTION_2LINES) */
    /* Note: To read a register a EPD, SPI_DIRECTION_1LINE should be set */
  DISCOVERY_SPIx->CR1 = 0
    | ( 1 * SPI_CR1_CPHA     )         // Clock Phase
    | ( 1 * SPI_CR1_CPOL     )         // Clock Polarity
    | ( 1 * SPI_CR1_MSTR     )         // Master Selection
    | ( SPI_CR1_BR__8 * SPI_CR1_BR_0     )         // BR[2:0] bits (Baud Rate Control)  // fPCLK/8
    | ( 1 * SPI_CR1_SPE      )         // SPI Enable
    | ( 0 * SPI_CR1_LSBFIRST )         // Frame Format
    | ( 1 * SPI_CR1_SSI      )         // Internal slave select
    | ( 1 * SPI_CR1_SSM      )         // Software slave management
    | ( 0 * SPI_CR1_RXONLY   )         // Receive only
    | ( SPI_CR1_DFF__8 * SPI_CR1_DFF      )         // Data Frame Format // 8 bits
    | ( 0 * SPI_CR1_CRCNEXT  )         // Transmit CRC next
    | ( 0 * SPI_CR1_CRCEN    )         // Hardware CRC calculation enable
    | ( 0 * SPI_CR1_BIDIOE   )         // Output enable in bidirectional mode
    | ( 0 * SPI_CR1_BIDIMODE )         // Bidirectional data mode enable
  ;
}

// won't be used anyway, as there is no MISO from the display
static uint32_t SPIx_Read(void) {
  return 0;
}

static void SPIx_Write(uint8_t n) {
  DISCOVERY_SPIx->DR = n;
  while (!(DISCOVERY_SPIx->SR & SPI_SR_TXE)) {
    // wait until empty
  }
  while ((DISCOVERY_SPIx->SR & SPI_SR_BSY)) {
    // wait until not busy
  }
}

void EPD_IO_Init(void) {
  // pins pinit - see gpio-clock init and gpio pins init in main()

  /* Enable Display */
  PIN_CLR(EPD_PWR);

  /* Set or Reset the control line */
  PIN_CLR(EPD_CS);
  PIN_SET(EPD_CS);

  /* EPD reset pin mamagement */
  PIN_SET(EPD_RESET);
  EPD_Delay(10);

  PIN_SET(DISCOVERY_SPIx_SCK);
  /* SPI Configuration */
  SPIx_Init();
}

void EPD_IO_WriteData(uint16_t n) {
  /* Reset EPD control line CS */
  PIN_CLR(EPD_CS);
  /* Set EPD data/command line DC to High */
  PIN_SET(EPD_DC);
  /* Send Data */
  SPIx_Write(n);
  /* Deselect: Chip Select high */
  PIN_SET(EPD_CS);
}

/**
  * @brief  Writes command to selected EPD register.
  * @param  Reg: Address of the selected register.
  * @retval None
  */
void EPD_IO_WriteReg(uint8_t n) {
  /* Reset EPD control line CS */
  PIN_CLR(EPD_CS);
  /* Set EPD data/command line DC to Low */
  PIN_CLR(EPD_DC);
  /* Send Command */
  SPIx_Write(n);
  /* Deselect: Chip Select high */
  PIN_SET(EPD_CS);
}

// following is bogus: there's no MISO connected
uint16_t EPD_IO_ReadData(void) {
  #if(1)
    return 0;
  #else
    /* Reset EPD control line CS */
    PIN_CLR(EPD_CS);
    /* Deselect: Chip Select high */
    PIN_SET(EPD_CS);
    /* Read Data */
    return SPIx_Read();
  #endif
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void EPD_Delay (uint32_t n) {
  LoopDelay(n * 3000);  // some fancy factor to get it roughly in ms
}

void moving_avg_temp(float *avg_temp)
{
	if(i < n)
	{
		temp_sum += temp;
		input[i] = temp;
		i++;
		*avg_temp = temp_sum / i;
	}
	else
	{
		temp_sum = 0;

		for (uint8_t j = n - 1; j > 0; j--) {
			//Shift element of array by one
			input[j] = input[j - 1];
			temp_sum += input[j];
		}
		//New temp will be added to the start of array.
		input[0] = temp;

		temp_sum += temp;
		*avg_temp = temp_sum / n;
	}
}
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  BSP_EPD_Init();	// Initting the EPD display

  // Showcasing
  BSP_EPD_DrawImage(0, 0, 72, 172, (uint8_t*) picture_1);
  BSP_EPD_RefreshDisplay();
  BSP_EPD_Clear(EPD_COLOR_WHITE);
  HAL_Delay(1500);

  // My signature, not neccessary though
  BSP_EPD_DisplayStringAt(0, 42, (unsigned char *)"Embedded system", CENTER_MODE);
  BSP_EPD_DisplayStringAt(0, 39, (unsigned char *)"programming", CENTER_MODE);
  BSP_EPD_DisplayStringAt(0, 35, (unsigned char *)"student Yiman A.u.", CENTER_MODE);
  BSP_EPD_RefreshDisplay();
  BSP_EPD_Clear(EPD_COLOR_WHITE);
  HAL_Delay(3000);

  // DHT sensor configs
  static DHT_sensor sensor = {GPIOA, GPIO_PIN_4, DHT22, GPIO_PULLUP};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  sprintf(msg, "Initializing...");

  // Timer interrupt every 10 seconds
  HAL_TIM_Base_Start_IT(&htim2);

  // Initialize global variables
  temp_sum = 0;
  i = 0, n = 5;

  while (1)
  {
	// Reading data fromt the sensor
	d = DHT_getData(&sensor);

	temp = d.temp;
	hum = d.hum;

	// Check for error
	if(d.temp == -128 && d.temp == -128)
	{
		sprintf(msg, "SENSOR ERROR!");
	}
	// Proceeds normally
	else
	{
		moving_avg_temp(&avg_temp);
	    sprintf(msg, "temp: %2.1fC, hum: %2.1f%%", avg_temp, hum);
	}

    HAL_Delay(2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NFC_MISO_Pin NFC_MOSI_Pin */
  GPIO_InitStruct.Pin = NFC_MISO_Pin|NFC_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
  HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ePD1_MOSI_Pin */
  GPIO_InitStruct.Pin = ePD1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(ePD1_MOSI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MFX_I2C_SDA_Pin */
  GPIO_InitStruct.Pin = MFX_I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(MFX_I2C_SDA_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM2)
	{
		BSP_EPD_DisplayStringAt(0, 40, (unsigned char *)msg, CENTER_MODE);
		BSP_EPD_RefreshDisplay();
		BSP_EPD_Clear(EPD_COLOR_WHITE);
	}
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
