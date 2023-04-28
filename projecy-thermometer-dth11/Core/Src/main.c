#include "main.h"
#include "dht11.h"
#include "stm32l053xx.h"
#include "stm32l0xx_hal_rcc.h"
#include "stm32l0xx_hal_gpio.h"
#include "picture.h"
#include "dht11.h"
#include "utils.h"
#include "gde021a1.h"
#include "stm32l0538_discovery.h"
#include "stm32l0538_discovery_epd.h"

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
GPIO_InitTypeDef GPIO_InitStruct = {0};

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

// ---------------------- M A I N -----------------------------------
uint16_t read_cycle(uint16_t cur_tics, uint8_t neg_tic){
	uint16_t cnt_tics;
 	if (cur_tics < MAX_TICS) cnt_tics = 0;
	if (neg_tic){
		//while (!GPIO_ReadInputDataBit(GPIOA,GPIO_PIN_3)&&(cnt_tics<MAX_TICS)){
		while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3) && (cnt_tics<MAX_TICS)){
			cnt_tics++;
		}
	}
	else {
		//while (GPIO_ReadInputDataBit(GPIOA,GPIO_PIN_3)&&(cnt_tics<MAX_TICS)){
		while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3) && (cnt_tics<MAX_TICS)){
			cnt_tics++;
		}
	}
 	return cnt_tics;
}

uint8_t read_DHT11(uint8_t *buf){
	uint16_t dt[42];
	uint16_t cnt;
	uint8_t i, check_sum;

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//reset DHT11
	//Delay(500);
	HAL_Delay(500);
 	//GPIO_LOW(GPIOA, GPIO_PIN_3);
 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);
	//Delay(20);
	HAL_Delay(20);
 	//GPIO_HIGH(GPIOA, GPIO_PIN_3);
 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);

 	GPIO_InitStruct.Pin = GPIO_PIN_3;
 	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 	GPIO_InitStruct.Pull = GPIO_NOPULL;
 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //start reading
 	cnt = 0;
	for(i=0;i<83 && cnt<MAX_TICS;i++){
		if (i & 1){
			cnt = read_cycle(cnt, 1);
		}
		else {
			cnt = read_cycle(cnt, 0);
			dt[i/2]= cnt;
		}
	}

 	//release line
	//GPIO_HIGH(GPIOA, GPIO_PIN_3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);

	if (cnt>=MAX_TICS) return DHT11_NO_CONN;

	//convert data
 	for(i=2;i<42;i++){
		(*buf) <<= 1;
  	if (dt[i]>20) {
			(*buf)++;
 		}
		if (!((i-1)%8) && (i>2)) {
			buf++;
		}
 	}

	//calculate checksum
	buf -= 5;
	check_sum = 0;
 	for(i=0;i<4;i++){
		check_sum += *buf;
		buf++;
	}

	if (*buf != check_sum) return DHT11_CS_ERROR;

	return DHT11_OK;
	//return check_sum;
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);

int main(void) {
	uint8_t buf[5], res;

	char strDisp[25];

	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize the EPD */
	BSP_EPD_Init();
	//BSP_EPD_RefreshDisplay();
	//BSP_EPD_Clear(EPD_COLOR_WHITE);

	/* Initialize all configured peripherals */
	//MX_GPIO_Init();
	//MX_RTC_Init();

	/* Showcasing */
	BSP_EPD_DrawImage(0, 0, 72, 172, (uint8_t*) picture_1);
	BSP_EPD_RefreshDisplay();
	BSP_EPD_Clear(EPD_COLOR_WHITE);
	HAL_Delay(1000);

	//sprintf(strDisp, "%02d/%02d/%02d %02d:%02d:%02d", sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

	BSP_EPD_DisplayStringAt(0, 45, (unsigned char *)"Introduction to", CENTER_MODE);
	BSP_EPD_DisplayStringAt(0, 42, (unsigned char *)"embedded system", CENTER_MODE);
	BSP_EPD_DisplayStringAt(0, 39, (unsigned char *)"programming", CENTER_MODE);
	BSP_EPD_DisplayStringAt(0, 36, (unsigned char *)"by Yiman A.u.", CENTER_MODE);
	BSP_EPD_RefreshDisplay();
	BSP_EPD_Clear(EPD_COLOR_WHITE);
	HAL_Delay(2000);

	while(1)
	{
		res = read_DHT11(buf);

		if (res==DHT11_OK)
			sprintf(strDisp, "RH=%02d%% t=%dC", buf[0], buf[2]);

		if (res==DHT11_CS_ERROR)
			sprintf(strDisp,"CHECKSUM ERROR");

		if (res==DHT11_NO_CONN)
			sprintf(strDisp,"NOT CONNECTED");

		BSP_EPD_DisplayStringAt(0, 40, (unsigned char *)strDisp, CENTER_MODE);
		BSP_EPD_RefreshDisplay();
		BSP_EPD_Clear(EPD_COLOR_WHITE);

		HAL_Delay(2000);
	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x10;
  sTime.Minutes = 0x1;
  sTime.Seconds = 0x20;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 0x17;
  sDate.Year = 0x23;

  if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PA2 */
//  GPIO_InitStruct.Pin = GPIO_PIN_2;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//  GPIO_HIGH(GPIOA, GPIO_PIN_2);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

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
	  BSP_EPD_DisplayStringAt(0, 40, (unsigned char*)"RTC ERROR OCCURRED", CENTER_MODE);
	  BSP_EPD_RefreshDisplay();
	  BSP_EPD_Clear(EPD_COLOR_WHITE);
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
