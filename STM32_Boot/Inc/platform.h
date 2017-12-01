/**
  ******************************************************************************
  * File Name          : platform.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#ifndef __PLATFORM_H
#define __PLATFORM_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32f2xx_hal.h"
#include "main.h"
#include "MX25L1606E.h"
#include <string.h>
#include "CRC.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define FLASH_SPI_CS_Pin            GPIO_PIN_1
#define FLASH_SPI_CS_GPIO_Port      GPIOB
#define LED2_Pin                    GPIO_PIN_12
#define LED2_GPIO_Port              GPIOB
#define FLASH_SPI_SCK_Pin           GPIO_PIN_10
#define FLASH_SPI_SCK_GPIO_Port     GPIOC
#define FLASH_SPI_MISO_Pin          GPIO_PIN_11
#define FLASH_SPI_MISO_GPIO_Port    GPIOC
#define FLASH_SPI_MOSI_Pin          GPIO_PIN_5
#define FLASH_SPI_MOSI_GPIO_Port    GPIOB


/* USER CODE BEGIN Private defines */
#define FLASH_MAX_RETRY_CNT                         (3)
#define FLASH_APP_MAX_LEN                           (512 * 1024)
#define FLASH_APP_START_ADDR                        (0x08000000 + (32 * 1024))
#define EXT_FLASH_FW_BACKUP_START_ADDR              (240 * 1024)
#define EXT_FLASH_FW_INFO_START_ADDR                (516 * 1024)
#define EXT_FLASH_FW_UPDATE_FLAG                    (0x55AA)

#define BOOT_ERROR_CODE_ERASE_FAIL                  (0)
#define BOOT_ERROR_CODE_RETRY_CNT_OVER              (1)
#define BOOT_ERROR_CODE_SPI_TIMEOUT                 (2)

#define STM32_BOOT_VERSION_ADDR                     (0x08001000)

#define FW_DEBUG

/* USER CODE END Private defines */

void Usr_Error_Handler(uint8_t* file, uint32_t line,uint32_t err_code);


/* Exported macro ------------------------------------------------------------*/
#ifdef  USR_ASSERT
  #define usr_para_check(expr) ((expr) ? (void)0 : usr_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void usr_assert_failed(uint8_t* file, uint32_t line);
#else
  #define usr_para_check(expr) ((void)0)
#endif 						/* USR_ASSERT */
  
/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
