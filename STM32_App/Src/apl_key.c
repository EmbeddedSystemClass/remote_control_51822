#include "apl_key.h"
#include "includes.h"
#include "coap.h"

uint8_t u8KeyPressed = 0;
TaskHandle_t xKeyTaskHandle = NULL;
QueueHandle_t xQueue_Key = NULL;
SemaphoreHandle_t xSemaphore_Key = NULL;

void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_NVIC_DisableIRQ (IRQn_KEY0);

    /*按键中断*/
    GPIO_InitStruct.Pin = GPIO_PIN_KEY0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;//松手开始
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init (GPIO_PORT_KEY0, &GPIO_InitStruct);

    /* Enable and set EXTI line 0 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority (IRQn_KEY0, 10, 0);
    HAL_NVIC_EnableIRQ (IRQn_KEY0);

    xSemaphore_Key = xSemaphoreCreateBinary();
    usr_res_assert(xSemaphore_Key != NULL);
}

void vKeyTask(void * pvParameters)
{
    uint32_t u32RxData = 0;

    Key_Init();
    WDT_Register(WDT_KEY_TASK_ID);

    /* Infinit loop */
    while(1)
    {
        if(pdTRUE == xQueueReceive(xQueue_Key, &u32RxData, 200/portTICK_PERIOD_MS))
        {
            switch(u32RxData)
            {
                case KEY0_SHORT_INPUT:
                    printf("KEY0_SHORT_INPUT\r\n");
                    break;

                case KEY0_LONG_INPUT:
                    WDT_Feed(WDT_KEY_TASK_ID);
					printf("KEY0_LONG_INPUT\r\n");
                    if(pdTRUE == xSemaphoreTake(xSemaphore_Key, 10))
                    {
                        printf("Do long press hansler\r\n");
                        //CoAP_NotifyResourceObservers(&CoAP_Res);
                        BLE_MSG_T           bleTxedMessage;
                        bleTxedMessage.event = BLE_START_SCAN_EVENT;
                        xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);
                        xSemaphoreGive(xSemaphore_Key);
                    }
                    break;

                case KEY1_SHORT_INPUT:
                    WDT_Feed(WDT_KEY_TASK_ID);
                    printf("KEY1_SHORT_INPUT\r\n");
                    break;

                case KEY1_LONG_INPUT:
                    WDT_Feed(WDT_KEY_TASK_ID);
                    RTC_CalendarShow();
                    break;

                default:
                    break;
            }
        }
        else
        {
            WDT_Feed(WDT_KEY_TASK_ID);
        }
    }
}

// Function that creates a task.
void CreateKeyTask(void)
{
    xTaskCreate( vKeyTask, "KeyTask", configMINIMAL_STACK_SIZE * 10, NULL, tskKEY_PRIORITY, &xKeyTaskHandle );
    configASSERT( xKeyTaskHandle );

	// Create a queue capable of containing 10 uint32_t values.
	xQueue_Key = xQueueCreate( 10, sizeof( uint32_t ) );
	configASSERT( xQueue_Key );
}



