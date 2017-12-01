/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @brief   Interrupt Service Routines.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "includes.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef cmuPort;
extern TIM_HandleTypeDef htim1;
/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */
/******************************************************************************/
/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler (void)
{
    while (1)
    {
    }
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler (void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }

}

// hard fault handler in C,
// with stack frame location as input parameter
void hard_fault_handler_c(unsigned int * hardfault_args)
{
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);

	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

	printf ("[Hard fault handler]\r\n");
	printf ("R0 = %0.8x\r\n", stacked_r0);
	printf ("R1 = %0.8x\r\n", stacked_r1);
	printf ("R2 = %0.8x\r\n", stacked_r2);
	printf ("R3 = %0.8x\r\n", stacked_r3);
	printf ("R12 = %0.8x\r\n", stacked_r12);
	printf ("LR = %0.8x\r\n", stacked_lr);
	printf ("PC = %0.8x\r\n", stacked_pc);
	printf ("PSR = %0.8x\r\n", stacked_psr);
	printf ("BFAR = %0.8x\r\n", (*((volatile uint32_t *)(0xE000ED38))));
	printf ("CFSR = %0.8x\r\n", (*((volatile uint32_t *)(0xE000ED28))));
	printf ("HFSR = %0.8x\r\n", (*((volatile uint32_t *)(0xE000ED2C))));
	printf ("DFSR = %0.8x\r\n", (*((volatile uint32_t *)(0xE000ED30))));
	printf ("AFSR = %0.8x\r\n", (*((volatile uint32_t *)(0xE000ED3C))));

	while(1)
	{
	}
}
/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler (void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler (void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler (void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}
/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F2xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f2xx.s).                    */
/******************************************************************************/
/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void RTC_WKUP_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
//  HAL_TIM_IRQHandler(&htim1);
}

/**
 * @brief  This function handles external line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler (void)
{
    HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_0);
}

/**
 * @brief  This function handles external line 0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI1_IRQHandler (void)
{
    HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_1);
}


/**
 * @brief  This function handles external line 2 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI2_IRQHandler (void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
 * @brief  This function handles external line 3 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI3_IRQHandler (void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/**
 * @brief  This function handles external line 3 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI4_IRQHandler (void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
 * @brief  This function handles external line 3 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}

/**
 * @brief  SHELL串口
 */
void USART1_IRQHandler (void)
{
    HAL_UART_IRQHandler (&cmuPort);
}

/**
 * @brief This function handles SPI2 global interrupt.
 */
void SPI2_IRQHandler (void)
{
}

/**
* @brief This function handles RTC alarms A and B interrupt through EXTI line 17.
*/
void RTC_Alarm_IRQHandler(void)
{
    HAL_RTC_AlarmIRQHandler(&hrtc);
}

/**
* @brief This function handles TIM6_DAC_IRQ global interrupt.
*/
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim7);
}

/**
  * @brief  This function handles DMA2_Stream2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(Adc_BatHandle.DMA_Handle);
}

/**
* @brief This function handles DMA2_Stream4 global interrupt.
*/
void DMA2_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(Adc_BatHandle.DMA_Handle);
}

/**
* @brief This function handles DMA2_Stream7 global interrupt.
*/
void DMA2_Stream7_IRQHandler (void)
{
    HAL_DMA_IRQHandler (&hdma_usart1_tx);
}

/**
* @brief This function handles USART6 global interrupt.
*/
void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
