/**
  ******************************************************************************
  * @file    stm32l0538_discovery_epd.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   This file contains all the functions prototypes for the 
  *          stm32l0538_discovery_epd.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L0538_DISCOVERY_EPD_H
#define __STM32L0538_DISCOVERY_EPD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gde021a1.h"
#include "fontsepd.h"

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

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L0538_DISCOVERY
  * @{
  */

/** @defgroup STM32L053_DISCOVERY_EPD
  * @{
  */


/** @defgroup STM32L0538_DISCOVERY_EPD_Exported_Types
  * @{
  */
typedef enum
{
  EPD_OK = 0,
  EPD_ERROR = 1,
  EPD_TIMEOUT = 2
} EPD_StatusTypeDef;

/**
  * @brief  Line mode structures definition
  */
typedef enum
{
  CENTER_MODE             = 0x01,    /*!< Center mode */
  RIGHT_MODE              = 0x02,    /*!< Right mode  */
  LEFT_MODE               = 0x03     /*!< Left mode   */
} Text_AlignModeTypdef;

/**
  * @}
  */

/** @defgroup STM32L0538_DISCOVERY_EPD_Exported_Constants
  * @{
  */

/**
  * @brief  EPD color
  */
#define EPD_COLOR_BLACK         0x00
#define EPD_COLOR_DARKGRAY      0x55
#define EPD_COLOR_LIGHTGRAY     0xAA
#define EPD_COLOR_WHITE         0xFF

/**
  * @brief EPD default font
  */
#define EPD_DEFAULT_FONT         Font12

/**
  * @}
  */

/** @defgroup STM32L0538_DISCOVERY_EPD_Exported_Functions
  * @{
  */
uint8_t  BSP_EPD_Init(void);
uint32_t BSP_EPD_GetXSize(void);
uint32_t BSP_EPD_GetYSize(void);

void     BSP_EPD_SetFont(sFONT *pFonts);
sFONT    *BSP_EPD_GetFont(void);

void     BSP_EPD_Clear(uint16_t Color);

void     BSP_EPD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr);
void     BSP_EPD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *pText, Text_AlignModeTypdef mode);
void     BSP_EPD_DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);

void     BSP_EPD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     BSP_EPD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     BSP_EPD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     BSP_EPD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);

void     BSP_EPD_RefreshDisplay(void);

void     BSP_EPD_CloseChargePump(void);

void     BSP_EPD_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L0538_DISCOVERY_EPD_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/*********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
