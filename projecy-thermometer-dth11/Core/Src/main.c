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

// -------- stuff missing from the stm32XXXxx.h
  // GPIOx_MODER - 2 bits per pin
#define GPIO_Mode_In                         0x00  // GPIO Input Mode
#define GPIO_Mode_Out                        0x01  // GPIO Output Mode
#define GPIO_Mode_AlternateFunction          0x02  // GPIO Alternate function Mode
#define GPIO_Mode_Analog                     0x03  // GPIO Analog Mode

                                                   // to 2.7V/above 2.7V
#define GPIO_Speed_VeryLow                   0x00  //  100kHz/400kHz
#define GPIO_Speed_Low                       0x01  //  600kHz/2MHz
#define GPIO_Speed_Medium                    0x02  //    2MHz/10MHz
#define GPIO_Speed_High                      0x03  //   10MHz/35MHz

#define GPIO_PullNone                        0x00
#define GPIO_PullUp                          0x01
#define GPIO_PullDown                        0x02

#define GPIO_AFRL_AFRL0_0  ((uint32_t)0x00000001)
#define GPIO_AFRL_AFRL1_0  ((uint32_t)0x00000010)
#define GPIO_AFRL_AFRL2_0  ((uint32_t)0x00000100)
#define GPIO_AFRL_AFRL3_0  ((uint32_t)0x00001000)
#define GPIO_AFRL_AFRL4_0  ((uint32_t)0x00010000)
#define GPIO_AFRL_AFRL5_0  ((uint32_t)0x00100000)
#define GPIO_AFRL_AFRL6_0  ((uint32_t)0x01000000)
#define GPIO_AFRL_AFRL7_0  ((uint32_t)0x10000000)
#define GPIO_AFRH_AFRH8_0  ((uint32_t)0x00000001)
#define GPIO_AFRH_AFRH9_0  ((uint32_t)0x00000010)
#define GPIO_AFRH_AFRH10_0 ((uint32_t)0x00000100)
#define GPIO_AFRH_AFRH11_0 ((uint32_t)0x00001000)
#define GPIO_AFRH_AFRH12_0 ((uint32_t)0x00010000)
#define GPIO_AFRH_AFRH13_0 ((uint32_t)0x00100000)
#define GPIO_AFRH_AFRH14_0 ((uint32_t)0x01000000)
#define GPIO_AFRH_AFRH15_0 ((uint32_t)0x10000000)
#define GPIO_AFRx          0x0F  // 4 bits per position

#define GPIO_AlternateFunction_SYS       0   /* default */
#define GPIO_AlternateFunction_TIM21_1   0
#define GPIO_AlternateFunction_SPI1      0
#define GPIO_AlternateFunction_LCD       1
#define GPIO_AlternateFunction_USB       2
#define GPIO_AlternateFunction_TIM2_1    2
#define GPIO_AlternateFunction_TSC       3
#define GPIO_AlternateFunction_USART1    4
#define GPIO_AlternateFunction_USART2    4
#define GPIO_AlternateFunction_USART3    4
#define GPIO_AlternateFunction_TIM2_2    5
#define GPIO_AlternateFunction_TIM21_2   5
#define GPIO_AlternateFunction_TIM22     5
#define GPIO_AlternateFunction_COMP1     7
#define GPIO_AlternateFunction_COMP2     7

// baudrate definitions
#define  SPI_CR1_BR__2          0   // fPCLK/2
#define  SPI_CR1_BR__4          1   // fPCLK/4
#define  SPI_CR1_BR__8          2   // fPCLK/8
#define  SPI_CR1_BR__16         3   // fPCLK/16
#define  SPI_CR1_BR__32         4   // fPCLK/32
#define  SPI_CR1_BR__64         5   // fPCLK/64
#define  SPI_CR1_BR__128        6   // fPCLK/128
#define  SPI_CR1_BR__256        7   // fPCLK/256

// frame format definitions
#define  SPI_CR1_DFF__8         0    // 8 bits
#define  SPI_CR1_DFF__16        1    // 16 bits

// ---- some of the trivial stuff
#define GLUE(a, b) a##b
#define PIN_SET(a) do {GLUE(a, _GPIO_PORT)->BSRR = (1 << GLUE(a, _PIN));} while(0)
#define PIN_CLR(a) do {GLUE(a, _GPIO_PORT)->BSRR = (1 << (GLUE(a, _PIN) + 16));} while(0)
#define PIN_GET(a) (!!(GLUE(a, _GPIO_PORT)->IDR & (1 << GLUE(a, _PIN))))

void LoopDelay(volatile uint32_t n) {
	while(n > 0) n--;
}

#define DELAY_CONSTANT 30000

// --------------------- some board-dependent defines and code
#define LED_RED_GPIO_PORT    GPIOA
#define LED_RED_PIN          5
#define LED_GREEN_GPIO_PORT  GPIOB
#define LED_GREEN_PIN        4

// ------- implement functions to get the epaper stuff working

#define EPD_COLOR_BLACK         0x00
#define EPD_COLOR_DARKGRAY      0x55
#define EPD_COLOR_LIGHTGRAY     0xAA
#define EPD_COLOR_WHITE         0xFF


// enable SPI clock and SPI-related pins -- see init at beginning of main()
#define DISCOVERY_SPIx                          SPI1

#define DISCOVERY_SPIx_SCK_GPIO_PORT            GPIOB
#define DISCOVERY_SPIx_SCK_PIN                  3
#define DISCOVERY_SPIx_MOSI_GPIO_PORT           GPIOB
#define DISCOVERY_SPIx_MOSI_PIN                 5
// no - there's no MISO connected, and PB4 is used for LED on the DISCO
// #define DISCOVERY_SPIx_MISO_PIN                 GPIO_PIN_4                 /* PB.04 */

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


// pins pinit - see init in main()
#define EPD_CS_GPIO_PORT                        GPIOA
#define EPD_CS_PIN                              15

#define EPD_DC_GPIO_PORT                        GPIOB
#define EPD_DC_PIN                              11

#define EPD_RESET_GPIO_PORT                     GPIOB
#define EPD_RESET_PIN                           2

#define EPD_BUSY_GPIO_PORT                      GPIOA
#define EPD_BUSY_PIN                            8

#define EPD_PWR_GPIO_PORT                       GPIOB
#define EPD_PWR_PIN                             10


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
		while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)&&(cnt_tics<MAX_TICS)){
			cnt_tics++;
		}
	}
	else {
		//while (GPIO_ReadInputDataBit(GPIOA,GPIO_PIN_3)&&(cnt_tics<MAX_TICS)){
		while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)&&(cnt_tics<MAX_TICS)){
			cnt_tics++;
		}
	}
 	return cnt_tics;
}

uint8_t read_DHT11(uint8_t *buf){
	uint16_t dt[42];
	uint16_t cnt;
	uint8_t i, check_sum;

	//reset DHT11
	//Delay(500);
	HAL_Delay(500);
 	GPIO_LOW(GPIOA,GPIO_PIN_3);
	//Delay(20);
	HAL_Delay(20);
 	GPIO_HIGH(GPIOA,GPIO_PIN_3);

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
	GPIO_HIGH(GPIOA,GPIO_PIN_3);

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

void Init_ePaper_GPIOs(void)
{
	/*!< Set MSION bit */
	RCC->CR |= (uint32_t)0x00000100;

	/*!< Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], MCOSEL[2:0] and MCOPRE[2:0] bits */
	RCC->CFGR &= (uint32_t) 0x88FF400C;

	/*!< Reset HSION, HSIDIVEN, HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFF6;

	/*!< Reset HSI48ON  bit */
	RCC->CRRCR &= (uint32_t)0xFFFFFFFE;

	/*!< Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/*!< Reset PLLSRC, PLLMUL[3:0] and PLLDIV[1:0] bits */
	RCC->CFGR &= (uint32_t)0xFF02FFFF;

	/*!< Disable all interrupts */
	RCC->CIER = 0x00000000;

	RCC->IOPENR |= 0
			| RCC_IOPENR_GPIOAEN
			| RCC_IOPENR_GPIOBEN
	  ;
	RCC->APB2ENR |= 0
			| RCC_APB2ENR_SPI1EN
	  ;

	GPIOA->MODER = (GPIOA->MODER
			& (~GPIO_MODER_MODE5)      // RED LED
			& (~GPIO_MODER_MODE8)      // EPD_BUSY
			& (~GPIO_MODER_MODE15)     // EPD_CS
	) | (0
			| (GPIO_Mode_Out * GPIO_MODER_MODE5_0)   // RED LED
			| (GPIO_Mode_In  * GPIO_MODER_MODE8_0)   // EPD_BUSY
			| (GPIO_Mode_Out * GPIO_MODER_MODE15_0)  // EPD_CS
	);

	GPIOA->OSPEEDR = (GPIOA->OSPEEDR
			& (~GPIO_OSPEEDER_OSPEED15)    // EPD_CS
	) | (0
			| (GPIO_Speed_High * GPIO_OSPEEDER_OSPEED15_0)  // EPD_CS
	);
	GPIOA->PUPDR = (GPIOA->PUPDR
			& (~GPIO_PUPDR_PUPD8)    // EPD_BUSY
	) | (0
			| (GPIO_PullDown * GPIO_PUPDR_PUPD8_0)  // EPD_BUSY
	);

	GPIOB->MODER = (GPIOB->MODER
			& (~GPIO_MODER_MODE4)      // GREEN LED
			& (~GPIO_MODER_MODE2)      // EPD_RESET
			& (~GPIO_MODER_MODE10)     // EPD_PWR
			& (~GPIO_MODER_MODE11)     // EPD_DC
			& (~GPIO_MODER_MODE3)      // SPIx_SCK
			& (~GPIO_MODER_MODE5)      // SPIx_MOSI
	) | (0
			| (GPIO_Mode_Out * GPIO_MODER_MODE4_0)   // RED LED
			| (GPIO_Mode_Out * GPIO_MODER_MODE2_0)   // EPD_RESET
			| (GPIO_Mode_Out * GPIO_MODER_MODE10_0)  // EPD_PWR
			| (GPIO_Mode_Out * GPIO_MODER_MODE11_0)  // EPD_DC
			| (GPIO_Mode_AlternateFunction * GPIO_MODER_MODE3_0)  // SPIx_SCK
			| (GPIO_Mode_AlternateFunction * GPIO_MODER_MODE5_0)  // SPIx_MOSI
	);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR
			& (~GPIO_OSPEEDER_OSPEED2)     // EPD_RESET
			& (~GPIO_OSPEEDER_OSPEED10)    // EPD_PWR
			& (~GPIO_OSPEEDER_OSPEED11)    // EPD_DC
			& (~GPIO_OSPEEDER_OSPEED3)     // SPIx_SCK
			& (~GPIO_OSPEEDER_OSPEED5)     // SPIx_MOSI
	) | (0
			| (GPIO_Speed_High * GPIO_OSPEEDER_OSPEED2_0)   // EPD_RESET
			| (GPIO_Speed_High * GPIO_OSPEEDER_OSPEED10_0)  // EPD_PWR
			| (GPIO_Speed_High * GPIO_OSPEEDER_OSPEED11_0)  // EPD_DC
			| (GPIO_Speed_High * GPIO_OSPEEDER_OSPEED3_0)   // SPIx_SCK
			| (GPIO_Speed_High * GPIO_OSPEEDER_OSPEED5_0)   // SPIx_MOSI
	);
	GPIOB->PUPDR = (GPIOB->PUPDR
			& (~GPIO_PUPDR_PUPD3)    // SPIx_SCK
			& (~GPIO_PUPDR_PUPD5)    // SPIx_MOSI
	) | (0
			| (GPIO_PullUp   * GPIO_PUPDR_PUPD3_0)  // SPIx_SCK
			| (GPIO_PullDown * GPIO_PUPDR_PUPD5_0)  // SPIx_MOSI
	);
	GPIOB->AFR[0] = (GPIOB->AFR[0]
								& (~ (GPIO_AFRx * GPIO_AFRL_AFRL3_0))    // SPIx_SCK
								& (~ (GPIO_AFRx * GPIO_AFRL_AFRL5_0))    // SPIx_MOSI
	) | (0
			| (GPIO_AlternateFunction_SPI1 * GPIO_AFRL_AFRL3_0)  // SPIx_SCK
			| (GPIO_AlternateFunction_SPI1 * GPIO_AFRL_AFRL5_0)  // SPIx_MOSI
	);
}

int main(void) {
	uint8_t buf[5], res;

	char strDisp[25];

	HAL_Init();

	Init_ePaper_GPIOs();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize the EPD */
	BSP_EPD_Init();
	//BSP_EPD_RefreshDisplay();
	//BSP_EPD_Clear(EPD_COLOR_WHITE);

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	//MX_RTC_Init();

	/* Showcasing */
	BSP_EPD_DrawImage(0, 0, 72, 172, (uint8_t*) picture_1);
	BSP_EPD_RefreshDisplay();
	BSP_EPD_Clear(EPD_COLOR_WHITE);
	HAL_Delay(1000);

	sprintf(strDisp, "%02d/%02d/%02d %02d:%02d:%02d", sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

	BSP_EPD_DisplayStringAt(0, 40, (unsigned char *)strDisp, CENTER_MODE);
	BSP_EPD_RefreshDisplay();
	BSP_EPD_Clear(EPD_COLOR_WHITE);
	HAL_Delay(5000);

	while(1)
	{
		res = read_DHT11(buf);

		if (res==DHT11_OK)
			sprintf(strDisp, "RH=%02d%% t=%dC", buf[0], buf[2]);
		else if (res==DHT11_CS_ERROR)
			sprintf(strDisp,"CHECKSUM ERROR");
		else if (res==DHT11_NO_CONN)
			sprintf(strDisp,"NO CONNECTED");

		BSP_EPD_DisplayStringAt(0, 40, (unsigned char *)strDisp, CENTER_MODE);
		BSP_EPD_RefreshDisplay();
		BSP_EPD_Clear(EPD_COLOR_WHITE);

//		HAL_Delay(2000);
//		BSP_EPD_DisplayStringAt(0,0,"I LOVE STM32", CENTER_MODE);
//		BSP_EPD_RefreshDisplay();
//		BSP_EPD_Clear(EPD_COLOR_WHITE);
//
//		HAL_Delay(2000);
//		BSP_EPD_DisplayStringAt(0,0,"I LOVE C++", CENTER_MODE);
//		BSP_EPD_RefreshDisplay();
//		BSP_EPD_Clear(EPD_COLOR_WHITE);
//
//		HAL_Delay(2000);
//		BSP_EPD_DisplayStringAt(0,0,"45 Celsius", CENTER_MODE);
//		BSP_EPD_RefreshDisplay();
//		BSP_EPD_Clear(EPD_COLOR_WHITE);

		HAL_Delay(100);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
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
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
