/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "stm32f2xx_hal.h"
#include "includes.h"

/* USER CODE BEGIN Includes */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;
/* Private function prototypes -----------------------------------------------*/
const char * u8BoxHwVer = "V1.0";
const char * u8BoxFwVer = "V0.0.1";
const uint32_t cstBoxPerSN __attribute__((at(0x08004000))) = 1511000104;

uint32_t u32Tim6IntCnt = 0;
box_info_t box_info;

void SystemClock_Config (void);
static void MX_GPIO_Init (void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
void Error_Handler (void);

/**
 * @brief  main
 * @param  N/A
 * @retval  N/A
 */
int main (void)
{
    SCB->VTOR = 0x08008000;

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init ();

    /* Configure the system clock */
    SystemClock_Config ();

    GetBoxSN(box_info.ucBoxSN, 0x08004000);
	box_info.u8SupportBandMaxNum = 1;
	*(box_info.ptdev_band_info) = &gBLEDevInfo;

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI2_Init();

	CreateServerTask();
    CreateKeyTask();
	CreateBleTask();
	CreateCliTask();
    CreateExtBatteryTask();
    CreatCoAPTask();
    MX_TIM6_Init();
	MX_TIM7_Init();
    bsp_rtc_init();
//    CreateMonitorTask();

    Leds_Init();

    /* Start scheduler */
    vTaskStartScheduler();

    /* Infinite loop */
    while (1)
    {
        printf("OS error\r\n");
    }
}

/**
 * @brief  系统时钟配置函数
 * @param  N/A
 * @retval  N/A
 */
void SystemClock_Config (void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 20;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig (&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_3);

    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{
    uint32_t SystemCoreClock = 0;

    SystemCoreClock = HAL_RCC_GetHCLKFreq();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = (uint32_t) ((SystemCoreClock /2) / 1000) - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 100;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{
    uint32_t SystemCoreClock = 0;

    SystemCoreClock = HAL_RCC_GetHCLKFreq();

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = (uint32_t) ((SystemCoreClock /2) / 1000) - 1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 100;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
}


/* SPI2 init function */
static void MX_SPI2_Init(void)
{

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }

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

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

}

/* USART6 init function */
void MX_USART6_UART_Init(void)
{

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
     PA8   ------> RCC_MCO_1
*/
void MX_GPIO_Init (void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
    __GPIOF_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOE_CLK_ENABLE();
    __GPIOG_CLK_ENABLE();

    /*Configure GPIO pin : PA8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIOG, &GPIO_InitStruct);

    /* Config the LED2 pin*/
    GPIO_InitStruct.Pin = GPIO_LED0_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIO_LED0_IO_PORT, &GPIO_InitStruct);

    /* Config the LED3 pin*/
    GPIO_InitStruct.Pin = GPIO_LED1_PIN;
    HAL_GPIO_Init (GPIO_LED1_IO_PORT, &GPIO_InitStruct);

    /* Config the SYS OFF pin*/
    HAL_GPIO_WritePin(GPIO_PORT_SYS_OFF, GPIO_PIN_SYS_OFF, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_SYS_OFF;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIO_PORT_SYS_OFF, &GPIO_InitStruct);

    /* Config the ACOK pin*/
    GPIO_InitStruct.Pin = GPIO_PIN_BAT_CHG;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIO_PORT_BAT_CHG, &GPIO_InitStruct);

}



void TIM6_Start(void)
{
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
}

void TIM6_Stop(void)
{
    /* Start Channel1 */
    if(HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
}


void TIM7_Start(void)
{
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
}

void TIM7_Stop(void)
{
    /* Start Channel1 */
    if(HAL_TIM_Base_Stop_IT(&htim7) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
}

int GetBoxSN(char *pucDstBoxSn, uint32_t u32Addr)
{
    int ret = 0;
    uint32_or_uint8_u a;
    uint8_t i;

	for(i = 0;i < 4;i ++)
	{
		*(a.arr + i) = *(__IO uint32_t*)(u32Addr + i);
	}

    if(pucDstBoxSn != NULL)
    {
        memset(pucDstBoxSn, '\0', sizeof(pucDstBoxSn));
        sprintf(pucDstBoxSn,"%d",a.l);
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return (ret);
}

void ExtFLASH_SPI_Init(void)
{
    MX_SPI3_Init();
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t u32RxSize = 0;
    uint32_t u32TxedMessage = 0;

    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
    else if(htim->Instance == TIM6)
    {
        u32Tim6IntCnt++;
        if(u32Tim6IntCnt >= (KEY_MAX_CNT + 5))
        {
            TIM6_Stop();
            u32Tim6IntCnt = 0;
            u8KeyPressed = 0;
            u32TxedMessage = KEY0_LONG_INPUT;
            xQueueSendFromISR(xQueue_Key, &u32TxedMessage, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if(htim->Instance == TIM7)
    {
        TIM7_Stop();
        u32RxSize = u16SimUartRxCnt;
        memset(u8SimRxArr,'\0',sizeof(u8SimRxArr));
        memcpy(u8SimRxArr,u8PreSimRxArr,u32RxSize);
        memset(u8PreSimRxArr,'\0',sizeof(u8PreSimRxArr));
        u16SimUartRxCnt = 0;
        xTaskNotifyFromISR(xUart2GRxTaskHandle, u32RxSize, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
    }
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t u32RxSize = 0;

    if (huart->Instance == SHELL_UART)
    {
        ShellCmuUartRxCpltCallBack ();
    }
    else if (huart->Instance == USART6)
    {
        u8PreSimRxArr[u16SimUartRxCnt++] = u8SimUartRxByte;
        TIM7_Stop();
        drv_SimUART_StartRx();
        TIM7_Start();

        // Notify the task
        if(u16SimUartRxCnt >= MAX_LEN_SIM_UART)
        {
            u32RxSize = u16SimUartRxCnt;
            memset(u8SimRxArr,'\0',sizeof(u8SimRxArr));
            memcpy(u8SimRxArr,u8PreSimRxArr,u16SimUartRxCnt);
            memset(u8PreSimRxArr,'\0',sizeof(u8PreSimRxArr));
            u16SimUartRxCnt = 0;
            xTaskNotifyFromISR(xUart2GRxTaskHandle, u32RxSize, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
    }
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
    /* NOTE: This function Should not be modified, when the callback is needed,
    the HAL_UART_TxCpltCallback could be implemented in the user file
    */
    if (huart->Instance == SHELL_UART)
    {
        ShellCmuUartTxCpltCallBack ();
    }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t u32TxedMessage = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BLE_MSG_T           bleTxedMessage;

    switch (GPIO_Pin)
    {
        case GPIO_PIN_KEY0:
            if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_PORT_KEY0, GPIO_PIN_KEY0))
            {
                u32Tim6IntCnt = 0;
                TIM6_Start();
                u8KeyPressed = 1;
            }
            else
            {
                if(1 == u8KeyPressed)
                {
                    TIM6_Stop();
                    u8KeyPressed = 0;
                    if(u32Tim6IntCnt >= KEY_MAX_CNT)
                    {
                        u32Tim6IntCnt = 0;
                        u32TxedMessage = KEY0_LONG_INPUT;
                        xQueueSendFromISR(xQueue_Key, &u32TxedMessage, &xHigherPriorityTaskWoken);
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                }
            }
            break;

        case GPIO_PIN_KEY1:
            u32TxedMessage = KEY1_LONG_INPUT;
            xQueueSendFromISR(xQueue_Key, &u32TxedMessage, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        case BLE_BTREQN_EXIT_IO_PIN:
        {
            /* read data from nRF51822 */
            bleTxedMessage.event = BLE_SPI_READ_EVENT;
            xQueueSendFromISR(xQueue_Ble, &bleTxedMessage, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        break;

        case GPIO_PIN_BAT_ACOK:
            drvBAT_ItCallBack(GPIO_PIN_BAT_ACOK);
            break;

        case GPIO_PIN_BAT_CHG:
            drvBAT_ItCallBack(GPIO_PIN_BAT_CHG);
            break;

        default:
            break;
    }
}

/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : AdcHandle handle
 * @note   This example shows a simple way to report end of conversion, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* AdcHandle)
{
    Bat_ConvCpltCallback();
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler (void)
{
    printf("Error!!!!");
//    led_on(LED0| LED1 | LED2);
    while(1);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    unsigned char cha;
    cha =(unsigned char)ch;

    if (shellDrv.GetAccess () == HAL_OK)
    {

      shellDrv.Write (&cha,1);
      shellDrv.ReleaseAccess ();
    }

    return ch;
}

#ifdef  USR_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void usr_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Error @ file %s on line %d\r\n", file, line);
}
#endif

#ifdef  USR_CHECK
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void usr_res_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Error @ file %s on line %d\r\n", file, line);
}
#endif

void usr_self_check_failed(uint8_t* file, uint32_t line)
{
    printf("Error @ file %s on line %d\r\n", file, line);

    while(1);
}
