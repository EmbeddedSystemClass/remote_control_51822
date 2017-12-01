/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"
#include "platform.h"
#include "main.h"
#include <stdio.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#define PUTCHAR_PROTOTYPE       int fputc(int ch, FILE *f)

typedef  void (*pFunction)(void);

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t SectorError = 0;
const uint8_t bootloaderVersion[4] __attribute__((at(STM32_BOOT_VERSION_ADDR))) = {0x00,0x00,0x01,0x00};// V1.0.0

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

USART_HandleTypeDef husart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_GPIO_DeInit(void);
static void MX_USART1_DeInit(void);
static void MX_USART1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI3_DeInit(void);
void CheckWrite(uint32_t flash_start_addr, uint16_t len);
void ConfigFwInfo(uint8_t u8Flag_update, uint32_t startAddr, uint32_t package_len, uint32_t crc);
void GetFwInfo(FW_UPDATE_INFO_T *p_fw_update_info);
int8_t CheckUpdate(FW_UPDATE_INFO_T *p_fw_update_info);
HAL_StatusTypeDef HAL_FLASH_ProgramPage(uint32_t Address, uint8_t *pBuffer, uint16_t len);
void CopyCodeFromExtFlash(uint32_t startRdAddr,uint32_t len);
int8_t CheckFwCopy(uint32_t u32StartAddr, uint32_t len);
int8_t CopyCode(FW_UPDATE_INFO_T *p_fw_update_info);
void EnterApp(void);
void EnterAppError(uint8_t err_code);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
FW_UPDATE_INFO_T fw_update_info;
/* USER CODE END 0 */

int main(void)
{
    /* MCU Configuration----------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI3_Init();
    #ifdef FW_DEBUG
        MX_USART1_Init();
    #endif

    #ifdef FW_DEBUG
        printf("\r\n\r\nWelcom to Bootloader...\r\n");
    #endif

    GetFwInfo(&fw_update_info);
    if(0 == CheckUpdate(&fw_update_info))
    {
        #ifdef FW_DEBUG
            printf("Start FW Update...\r\n");
        #endif
        HAL_FLASH_Unlock();
        
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        EraseInitStruct.Sector = FLASH_SECTOR_1;
        EraseInitStruct.NbSectors = 10;
        if(HAL_OK == HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError))
        {
            #ifdef FW_DEBUG
                printf("Erase Complete...\r\n");
            #endif
            if(0 == CopyCode(&fw_update_info))
            {
                #ifdef FW_DEBUG
                    printf("FW Copy successfully...\r\n");
                #endif
                HAL_FLASH_Lock(); 
                ConfigFwInfo(0, 0, 0, 0);
                EnterApp();
            }
            else
            {
                #ifdef FW_DEBUG
                    printf("FW Copy Failed...\r\n");
                #endif
                EnterAppError(BOOT_ERROR_CODE_RETRY_CNT_OVER);
            }
        }
        else
        {
            #ifdef FW_DEBUG
                printf("Erase Failed...\r\n");
            #endif
            EnterAppError(BOOT_ERROR_CODE_ERASE_FAIL);
        }
    }
    else
    {
        EnterApp();
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

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
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_SPI3_DeInit(void)
{
  if (HAL_SPI_DeInit(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART5 init function */
#ifdef FW_DEBUG

static void MX_USART1_Init(void)
{

  husart1.Instance = USART1;
  husart1.Init.BaudRate = 9600;
  husart1.Init.WordLength = UART_WORDLENGTH_8B;
  husart1.Init.StopBits = UART_STOPBITS_1;
  husart1.Init.Parity = UART_PARITY_NONE;
  husart1.Init.Mode = UART_MODE_TX_RX;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

static void MX_USART1_DeInit(void)
{
  if (HAL_USART_DeInit(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FLASH_SPI_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FLASH_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

}

static void MX_GPIO_DeInit(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
    while(1)
    {}
  /* USER CODE END Error_Handler */
}

void EnterApp(void)
{
    #ifdef FW_DEBUG
	    printf("EnterApp\r\n");
    #endif
    
    MX_SPI3_DeInit();
    MX_USART1_DeInit();
    MX_GPIO_DeInit();
    
    /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
    if (((*(__IO uint32_t*)FLASH_APP_START_ADDR) & 0x2FFE0000 ) == 0x20000000)
    {

        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*) (FLASH_APP_START_ADDR + 4);
        Jump_To_Application = (pFunction) JumpAddress;

        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) FLASH_APP_START_ADDR);
        Jump_To_Application();
    }
}

void EnterAppError(uint8_t err_code)
{
    #ifdef FW_DEBUG
    printf("EnterApp,Error\r\n");
    #endif
}

/*********************************************************************************
  * Read and print the specification page data
  * @param  flash_start_addr: the start addr
  * @param  len: length
  * @retval none
**********************************************************************************/
void CheckWrite(uint32_t flash_start_addr, uint16_t len)
{
	uint8_t rd_temp[256] = {0};
	uint16_t index;

    /* Check parameters */
	usr_para_check(len <= PAGE_SIZE);

    #ifdef FLASH_DEBUG
	    printf("[FLASH] CheckWrite:\r\n");
    #endif

	CMD_READ(flash_start_addr, rd_temp, len);
	for(index = 0;index < len;index ++)
	{
		printf("0x%02X, ",*(rd_temp + index));
		if(!((index + 1) % 16))
		{
			printf("\r\n");
		}
	}
}

///*********************************************************************************
//  * Backup the specification sector
//  * @param  flash_addr: the start addr of the sector to backup
//  * @retval None
//**********************************************************************************/
//static void FLASH_BackupOneSector(uint32_t flash_addr)
//{
//	uint8_t temp[128] = {0},i;
//    uint32_t write_addr = 0;
//    uint32_t read_addr = 0;

//	/* Check parameters */
//	usr_para_check(!(flash_addr % SECTOR_SIZE));

//	#ifdef FLASH_DEBUG
//		printf("[FLASH] Now,backup the sector @ 0x%08x\r\n",flash_addr);
//	#endif

//    write_addr = EXT_FLASH_FW_BACKUP_START_ADDR;
//    read_addr = flash_addr;

//	for(i = 0;i < 32;i ++)
//	{
//		CMD_READ(read_addr, temp, sizeof(temp));
//		CMD_PageProgram(write_addr, temp, sizeof(temp));
//        write_addr += 128;
//        read_addr += 128;
//	}

//    #ifdef FLASH_DEBUG
//        CheckWrite(FLASH_BACKUP_SECTOR_BASE_ADDR, PAGE_SIZE);
//    #endif
//}

/*******************************************************************************
* Function Name  : ComputeCRC
* Description    : Compute the CRC of the specified flash space.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t ComputeCRC(uint32_t u32StartAddress,uint32_t len)
{
    uint32_t u32Crc_tmp = 0;
    
    u32Crc_tmp = crc32(u32Crc_tmp,(uint8_t *)u32StartAddress, len);

    return u32Crc_tmp;
}

uint32_t EXT_ComputeCRC(uint32_t u32StartAddress,uint32_t len)
{
    uint32_t flashRdStart = 0;
    uint32_t u32Crc_tmp = 0;
    uint8_t  rd_flash[PAGE_SIZE] = {0};
    BOOL bReadComplete = FALSE;
    uint32_t TmpLen = 0;
    
    #ifdef FW_DEBUG
        printf("EXT_ComputeCRC\r\n");
    #endif

    flashRdStart = u32StartAddress;
    TmpLen = len;
    do{
        if(TmpLen > PAGE_SIZE)
        {
            CMD_READ(flashRdStart, rd_flash, PAGE_SIZE);
            u32Crc_tmp = crc32(u32Crc_tmp,rd_flash, PAGE_SIZE);
            TmpLen -= PAGE_SIZE;
            flashRdStart += PAGE_SIZE;
        }
        else
        {
            CMD_READ(flashRdStart, rd_flash, TmpLen);
            u32Crc_tmp = crc32(u32Crc_tmp,rd_flash, TmpLen);
            bReadComplete = TRUE;
        }
    }while(bReadComplete == FALSE);

    return (u32Crc_tmp);
}

void ConfigFwInfo(uint8_t u8Flag_update, uint32_t startAddr, uint32_t package_len, uint32_t crc)
{
    FW_UPDATE_INFO_T fw_update_info;

    if(1 == u8Flag_update)
    {
        fw_update_info.u16FlagFwUpdate = EXT_FLASH_FW_UPDATE_FLAG;
        fw_update_info.u32FwStartAddr = startAddr;
        fw_update_info.u32FwPacketLen = package_len;
        fw_update_info.u32FwFwCrc = crc;
    }
    else
    {
        fw_update_info.u16FlagFwUpdate = 0xFFFF;
        fw_update_info.u32FwStartAddr = 0xFFFF;
        fw_update_info.u32FwPacketLen = 0xFFFF;
        fw_update_info.u32FwFwCrc = 0xFFFF;
    }

    CMD_SE(EXT_FLASH_FW_INFO_START_ADDR);
    CMD_PageProgram(EXT_FLASH_FW_INFO_START_ADDR, (uint8_t *)&fw_update_info, sizeof(FW_UPDATE_INFO_T));
}

void GetFwInfo(FW_UPDATE_INFO_T *p_fw_update_info)
{
    uint8_t tmpArr[PAGE_SIZE] = {0};
    FW_UPDATE_INFO_T *p_fw_update_info_tmp = (FW_UPDATE_INFO_T *)tmpArr;

    CMD_READ(EXT_FLASH_FW_INFO_START_ADDR, tmpArr, PAGE_SIZE);
    p_fw_update_info->u16FlagFwUpdate = p_fw_update_info_tmp->u16FlagFwUpdate;
    p_fw_update_info->u32FwFwCrc = p_fw_update_info_tmp->u32FwFwCrc;
    p_fw_update_info->u32FwPacketLen = p_fw_update_info_tmp->u32FwPacketLen;
    p_fw_update_info->u32FwStartAddr = p_fw_update_info_tmp->u32FwStartAddr;
    
//    #ifdef FW_DEBUG
        printf("p_fw_update_info_tmp->u16FlagFwUpdate = 0x%04x\r\n",p_fw_update_info_tmp->u16FlagFwUpdate);
        printf("p_fw_update_info_tmp->u32FwFwCrc = 0x%08x\r\n",p_fw_update_info_tmp->u32FwFwCrc);
        printf("p_fw_update_info_tmp->u32FwPacketLen = 0x%08x\r\n",p_fw_update_info_tmp->u32FwPacketLen);
        printf("p_fw_update_info_tmp->u32FwStartAddr = 0x%08x\r\n",p_fw_update_info_tmp->u32FwStartAddr);
//    #endif
}

HAL_StatusTypeDef HAL_FLASH_ProgramPage(uint32_t Address, uint8_t *pBuffer, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    for(uint16_t i = 0;i < len;i ++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address + i, *(pBuffer + i));
        if(HAL_OK != status)
        {
            break;
        }
    }

    return (status);
}

void CopyCodeFromExtFlash(uint32_t startRdAddr,uint32_t len)
{
    uint8_t tmpArr[PAGE_SIZE] = {0};
    BOOL bFlag_complete = FALSE;
    uint32_t u32TmpRdAddr = 0;
    uint32_t u32TmpWrAddr = 0;
    uint32_t u32TmpLen = 0;

    u32TmpRdAddr = startRdAddr;
    u32TmpWrAddr = FLASH_APP_START_ADDR;
    u32TmpLen = len;
    do{
        if(u32TmpLen > PAGE_SIZE)
        {
            CMD_READ(u32TmpRdAddr, tmpArr, PAGE_SIZE);
            HAL_FLASH_ProgramPage(u32TmpWrAddr,tmpArr,PAGE_SIZE);
            u32TmpRdAddr += PAGE_SIZE;
            u32TmpWrAddr += PAGE_SIZE;
            u32TmpLen -= PAGE_SIZE;
        }
        else
        {
            CMD_READ(u32TmpRdAddr, tmpArr, u32TmpLen);
            HAL_FLASH_ProgramPage(u32TmpWrAddr,tmpArr,u32TmpLen);
            bFlag_complete = TRUE;
        }
    }while(bFlag_complete == FALSE);
}

int8_t CheckUpdate(FW_UPDATE_INFO_T *p_fw_update_info)
{
    int8_t ret = 0;
    uint32_t crc = 0;

    if(EXT_FLASH_FW_UPDATE_FLAG == p_fw_update_info->u16FlagFwUpdate)
    {
        if(p_fw_update_info->u32FwPacketLen < FLASH_APP_MAX_LEN)
        {
            crc = EXT_ComputeCRC(p_fw_update_info->u32FwStartAddr,p_fw_update_info->u32FwPacketLen);
            #ifdef FW_DEBUG
                printf("EXTFlash,crc = 0%08x\r\n",crc);
            #endif
            if(crc == p_fw_update_info->u32FwFwCrc)
            {
                ret = 0;
            }
            else
            {
                ret = -1;
            }
        }
        else
        {
            ret = -1;
        }
    }
    else
    {
        ret = -1;
    }

    return (ret);
}

int8_t CheckFwCopy(uint32_t u32StartAddr, uint32_t len)
{
    int8_t ret = 0;
    uint32_t crc = 0;

    crc = ComputeCRC(u32StartAddr,len);
    #ifdef FW_DEBUG
        printf("CheckFwCopy,IntFlash,crc = 0x%08x\r\n",crc);
    #endif
    if(crc == fw_update_info.u32FwFwCrc)
    {
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return (ret);
}

int8_t CopyCode(FW_UPDATE_INFO_T *p_fw_update_info)
{
    int8_t ret = 0;
    int8_t retry_cnt = 0;
    
    for(retry_cnt = 0;retry_cnt < FLASH_MAX_RETRY_CNT;retry_cnt ++)
    {
        #ifdef FW_DEBUG
            printf("The %dst timers:\r\n",(retry_cnt + 1));
        #endif
        CopyCodeFromExtFlash(fw_update_info.u32FwStartAddr, fw_update_info.u32FwPacketLen);
        if(0 == CheckFwCopy(FLASH_APP_START_ADDR, fw_update_info.u32FwPacketLen))
        {
            break;
        }
    }
    
    if(retry_cnt >= FLASH_MAX_RETRY_CNT)
    {
        ret = -1;
    }
    else
    {
        ret = 0;
    }
    
    return (ret);
}

void Usr_Error_Handler(uint8_t* file, uint32_t line,uint32_t err_code)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
    printf("Error_Handler: file %s on line %d,error = 0x%08x\r\n", file, line, err_code);
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&husart1, (uint8_t *)&ch, 1, 0xFFFF);
    HAL_USART_Transmit(&husart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
