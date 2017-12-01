/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CDMA_UART_RTS_Pin                       GPIO_PIN_12
#define CDMA_UART_RTS_GPIO_Port                 GPIOG

#define CDMA_UART_TX_Pin                        GPIO_PIN_6
#define CDMA_UART_TX_GPIO_Port                  GPIOC

#define CDMA_UART_RX_Pin                        GPIO_PIN_7
#define CDMA_UART_RX_GPIO_Port                  GPIOC

#define CDMA_UART_CTS_Pin                       GPIO_PIN_13
#define CDMA_UART_CTS_GPIO_Port                 GPIOG

#define SHELL_UART_TX_Pin                       GPIO_PIN_6
#define SHELL_UART_TX_GPIO_Port                 GPIOB

#define SHELL_UART_RX_Pin                       GPIO_PIN_7
#define SHELL_UART_RX_GPIO_Port                 GPIOB

#define GPIO_PIN_MX25_SPI_CS                    GPIO_PIN_1
#define GPIO_PORT_MX25_SPI_CS                   GPIOB

#define GPIO_PIN_KEY0                           GPIO_PIN_1
#define GPIO_PORT_KEY0                          GPIOA
#define IRQn_KEY0                               EXTI1_IRQn

#define GPIO_PIN_KEY1                           GPIO_PIN_3
#define GPIO_PORT_KEY1                          GPIOD
#define IRQn_KEY1                               EXTI3_IRQn

#define GPIO_PIN_2G_MODULE_PWR                  GPIO_PIN_5
#define GPIO_PORT_2G_MODULE_PWR                 GPIOF

#define GPIO_PIN_2G_MODULE_RST                  GPIO_PIN_14
#define GPIO_PORT_2G_MODULE_RST                 GPIOG

#define GPIO_PIN_2G_MODULE_ONOFF                GPIO_PIN_9
#define GPIO_PORT_2G_MODULE_ONOFF               GPIOF

#define GPIO_PIN_LED_PS_G                       GPIO_PIN_6
#define GPIO_PORT_LED_PS_G                      GPIOA

#define GPIO_PIN_LED_PS_R                       GPIO_PIN_7
#define GPIO_PORT_LED_PS_R                      GPIOA

#define GPIO_PIN_LED_SS_G                       GPIO_PIN_10
#define GPIO_PORT_LED_SS_G                      GPIOE

#define GPIO_PIN_LED_SS_B                       GPIO_PIN_11
#define GPIO_PORT_LED_SS_B                      GPIOE

#define GPIO_PIN_LED_SS_R                       GPIO_PIN_12
#define GPIO_PORT_LED_SS_R                      GPIOE

#define GPIO_PIN_SYS_OFF                        GPIO_PIN_2
#define GPIO_PORT_SYS_OFF                       GPIOA

#define GPIO_PIN_BAT_ACOK				        GPIO_PIN_5
#define GPIO_PORT_BAT_ACOK			            GPIOE

#define GPIO_PIN_BAT_CHG				        GPIO_PIN_6
#define GPIO_PORT_BAT_CHG				        GPIOF
#define EXTIx_IRQn_BAT_ACOK    				    EXTI9_5_IRQn

#define GPIO_PIN_BAT_CHG_EN				        GPIO_PIN_4
#define GPIO_PORT_BAT_CHG_EN				    GPIOE

#define GPIO_PIN_BAT_VOL				        GPIO_PIN_5
#define GPIO_PORT_BAT_VOL			            GPIOA

/* Definition for ADCx clock resources */
#define ADCx_BAT                            ADC1
#define ADCx_BAT_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define DMAx_BAT_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_BAT_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define ADCx_BAT_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_BAT_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_BAT_CHANNEL_PIN                GPIO_PIN_5
#define ADCx_BAT_CHANNEL_GPIO_PORT          GPIOA

/* Definition for ADCx's Channel */
#define ADCx_BAT_CHANNEL                    ADC_CHANNEL_5

/* Definition for ADCx's DMA */
#define ADCx_BAT_DMA_CHANNEL                DMA_CHANNEL_0
#define ADCx_BAT_DMA_STREAM                 DMA2_Stream4

/* Definition for ADCx's NVIC */
#define ADCx_BAT_DMA_IRQn                   DMA2_Stream4_IRQn


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
