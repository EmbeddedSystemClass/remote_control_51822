/*
 * drv_Uart.c
 *
 *  Created on: 2015年5月4日
 *      Author: yyy
 */
#include "drv_CmuUart.h"
#include "drv_IOInit.h"
#include "includes.h"

SemaphoreHandle_t xRxSemaphore = NULL;
SemaphoreHandle_t xTxMutex = NULL;
SemaphoreHandle_t xRawModeTxSem = NULL;
SemaphoreHandle_t xCmuUartMutex = NULL;
UART_HandleTypeDef *cmuHandle;
DMA_HandleTypeDef hdma_usart1_tx;

/*获取数据*/
static HAL_StatusTypeDef drv_CmuGetByteArray (uint8_t *pBuf,uint32_t length);
//static void drv_CmuUartDeinit (void);
static char drv_CmuGetChar(void);
static char drv_CmuGetCharNonBlocking (void);
static HAL_StatusTypeDef drv_CmuSendData(uint8_t *pTxBuf,uint16_t length);
static HAL_StatusTypeDef drv_CmuUART_Init (UART_HandleTypeDef *uartHandle);
static HAL_StatusTypeDef drv_CmuUartReleaseAccess (void);
static HAL_StatusTypeDef drv_CmuUartGetAccess (void);


SHELL_DrvTypeDef  shellDrv={
    drv_CmuUART_Init,
    drv_CmuSendData,
    drv_CmuGetChar,
    drv_CmuGetCharNonBlocking,
    drv_CmuGetByteArray,
    drv_CmuUartGetAccess,
    drv_CmuUartReleaseAccess,
    NULL

};

/* USART2 init function */
static HAL_StatusTypeDef drv_CmuUART_Init (UART_HandleTypeDef *uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    cmuHandle = uartHandle;

    /* Create the semaphore used to access the UART Tx. */
    xTxMutex = xSemaphoreCreateBinary();
    if (xTxMutex == NULL)
    {
        return HAL_ERROR;
    }
    xSemaphoreGive(xTxMutex);
    
    xCmuUartMutex = xSemaphoreCreateMutex();
    if (xCmuUartMutex == NULL)
    {
        return HAL_ERROR;
    }

    xRawModeTxSem = xSemaphoreCreateBinary();
    if (xRawModeTxSem == NULL)
    {
        return HAL_ERROR;
    }
    
    /* Create the queues used to hold Rx/Tx characters. */
    /* 创建信号量 */
    xRxSemaphore = xSemaphoreCreateBinary();
    if (xRxSemaphore == NULL)
    {
        return HAL_ERROR;
    }
    configASSERT(xTxMutex);
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SHELL_UART_CLK_EN();
    /* Enable USARTx clock */

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin = SHELL_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = SHELL_UART_TX_AF;

    HAL_GPIO_Init (SHELL_UART_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = SHELL_UART_RX_PIN;
    GPIO_InitStruct.Alternate = SHELL_UART_RX_AF;

    HAL_GPIO_Init (SHELL_UART_RX_GPIO_PORT, &GPIO_InitStruct);
    /* Peripheral DMA init*/
    SHELL_UARTDMAx_CLK_ENABLE();
    hdma_usart1_tx.Instance = SHELL_UART_TX_DMA_STREAM;
    hdma_usart1_tx.Init.Channel = SHELL_UART_TX_DMA_CH;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init (&hdma_usart1_tx);

    __HAL_LINKDMA (cmuHandle, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority (SHELL_UART_DMA_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ (SHELL_UART_DMA_TX_IRQn);

    //P579
    uartHandle->Instance = SHELL_UART;
    uartHandle->Init.BaudRate = 115200;
    uartHandle->Init.WordLength = UART_WORDLENGTH_8B;
    uartHandle->Init.StopBits = UART_STOPBITS_1;
    uartHandle->Init.Parity = UART_PARITY_NONE;
    uartHandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartHandle->Init.Mode = UART_MODE_TX_RX;
    if (HAL_UART_DeInit (uartHandle) != HAL_OK)
    {
        usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
        return HAL_ERROR;
    }
    if (HAL_UART_Init (uartHandle) != HAL_OK)
    {
        usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
        return HAL_ERROR;
    }

    /* System interrupt init*/
    HAL_NVIC_SetPriority (SHELL_UART_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ (SHELL_UART_IRQn);
    
    return HAL_OK;
}
/*阻塞方式获取一个数据*/
static char drv_CmuGetChar (void)
{
    uint8_t getChar;
    
    HAL_UART_Receive_IT (cmuHandle, &getChar, 1);
    if(xSemaphoreTake( xRxSemaphore, 10000 ) == pdTRUE)
    {
        return getChar;
    }
    else
    {
        return NULL ;
    }
}
/*实时获取*/
static char drv_CmuGetCharNonBlocking (void)
{
    static uint8_t getChar;

    HAL_UART_Receive_IT (cmuHandle, &getChar, 1);
    if(xSemaphoreTake( xRxSemaphore, 0) == pdTRUE)
    {
        return getChar;
    }
    else
    {
        return NULL;
    }
}

/*获取数据多个数据，阻塞模式*/
static HAL_StatusTypeDef drv_CmuGetByteArray (uint8_t *pBuf,uint32_t length)
{
    return  HAL_UART_Receive (cmuHandle, pBuf, length,10000);/*10秒超时*/
}

/*阻塞发送数据*/
static HAL_StatusTypeDef drv_CmuSendData (uint8_t *pTxBuf, uint16_t length)
{
    if(xSemaphoreTake( xTxMutex, cmdMAX_MUTEX_WAIT) == pdPASS)
    {
        return HAL_UART_Transmit_DMA (cmuHandle, (unsigned char *) pTxBuf, length);
    }
    
    return HAL_ERROR;
}
/*重置串口*/
void drv_CmuUartDeinit (void)
{
  SHELL_UART_CLK_DISABLE();

  /**USART1 GPIO Configuration
   PA9     ------> USART1_TX
   PA10     ------> USART1_RX
   */
  HAL_GPIO_DeInit (SHELL_UART_TX_GPIO_PORT,SHELL_UART_TX_PIN | SHELL_UART_RX_PIN);

  /* Peripheral DMA DeInit*/
  HAL_DMA_DeInit (cmuHandle->hdmatx);

  /* Peripheral interrupt DeInit*/
  HAL_NVIC_DisableIRQ (SHELL_UART_IRQn);
}
/*获取串口控制权*/
HAL_StatusTypeDef
drv_CmuUartGetAccess ()
{
  if ( pdTRUE == xSemaphoreTake(xCmuUartMutex, portMAX_DELAY))
    {
      return HAL_OK;
    }
  return HAL_ERROR;
}
/*释放串口控制权*/
HAL_StatusTypeDef
drv_CmuUartReleaseAccess ()
{
  if ( pdTRUE == xSemaphoreGive(xCmuUartMutex))
    {
      return HAL_OK;
    }
  return HAL_ERROR;
}

/*----------------------------------Interrupt-------------------------------------*/
/*TX中断回掉控制权*/
void
ShellCmuUartTxCpltCallBack ()
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xTxMutex, &xHigherPriorityTaskWoken);
  if (pdTRUE == xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*RX中断回掉控制权*/
void
ShellCmuUartRxCpltCallBack ()
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xRxSemaphore, &xHigherPriorityTaskWoken);
  if (pdTRUE == xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
