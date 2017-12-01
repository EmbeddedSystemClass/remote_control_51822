#include "apl_bat.h"
#include "includes.h"

TaskHandle_t xBatTaskHandle = NULL;
battery_t battery;
TimerHandle_t xBatLedTimers = NULL;
TimerHandle_t xBatDetectTimers = NULL;
QueueHandle_t xBatQueue;

/* ADC handler declaration */
ADC_HandleTypeDef Adc_BatHandle;
/* Variable used to get converted value */
uint16_t uhADCxConvertedValue[BAT_VOL_BUFFER_LENGTH] = {0};
float fExtBatVolBuffer[BAT_VOL_ALARM_NUMBER] = {0};
uint8_t u8LowVoltAlarm_Flag = 0;
uint8_t u8ExtPowerPlugged_Flag = 0;


void Bat_ChargeEnable(void);

int8_t vStartDetectTimer(void)
{
    int8_t ret = 0;

    if( xTimerStart( xBatDetectTimers, 0 ) != pdPASS )
    {
        ret = -1;
    }

    return (ret);
}

void vBatDetectTimerCallback( TimerHandle_t pxTimer )
{
    uint32_t ulVar = BAT_DETECT_START_EVENT;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendToBackFromISR(xBatQueue, &ulVar, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Bat_ConvCpltCallback(void)
{
    uint32_t ulVar = BAT_DETECT_END_EVENT;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendToBackFromISR(xBatQueue, &ulVar, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Bat_ACOK_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Config the CHARGE_EN pin*/
    GPIO_InitStruct.Pin = GPIO_PIN_BAT_CHG_EN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIO_PORT_BAT_CHG_EN, &GPIO_InitStruct);
    Bat_ChargeEnable();

    /* Config the ACOK pin*/
    GPIO_InitStruct.Pin = GPIO_PIN_BAT_ACOK;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIO_PORT_BAT_ACOK, &GPIO_InitStruct);

    /* Config the ACOK pin*/
    GPIO_InitStruct.Pin = GPIO_PIN_BAT_CHG;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIO_PORT_BAT_CHG, &GPIO_InitStruct);

    /* Enable and set EXTI line 5 Interrupt to the lowest priority */
    HAL_NVIC_DisableIRQ (EXTIx_IRQn_BAT_ACOK);
    HAL_NVIC_SetPriority (EXTIx_IRQn_BAT_ACOK, 10, 0);
    HAL_NVIC_EnableIRQ (EXTIx_IRQn_BAT_ACOK);
}

void Bat_VOL_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Config the ACOK pin*/
    GPIO_InitStruct.Pin = GPIO_PIN_BAT_VOL;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIO_PORT_BAT_VOL, &GPIO_InitStruct);
}

void Bat_ChargeEnable(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_BAT_CHG_EN, GPIO_PIN_BAT_CHG_EN, GPIO_PIN_SET);
}

void Bat_ChargeDisable(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_BAT_CHG_EN, GPIO_PIN_BAT_CHG_EN, GPIO_PIN_RESET);
}


int BatVolDetectInit(void)
{
    ADC_ChannelConfTypeDef sConfig;

    /*##-1- Configure the ADC peripheral #######################################*/
    Adc_BatHandle.Instance = ADCx_BAT;
    Adc_BatHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    Adc_BatHandle.Init.Resolution = ADC_RESOLUTION_12B;
    Adc_BatHandle.Init.ScanConvMode = DISABLE;
    Adc_BatHandle.Init.ContinuousConvMode = ENABLE;
    Adc_BatHandle.Init.DiscontinuousConvMode = DISABLE;
    Adc_BatHandle.Init.NbrOfDiscConversion = 0;
    Adc_BatHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    Adc_BatHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    Adc_BatHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    Adc_BatHandle.Init.NbrOfConversion = 1;
    Adc_BatHandle.Init.DMAContinuousRequests = ENABLE;
    Adc_BatHandle.Init.EOCSelection = DISABLE;
    if(HAL_ADC_Init(&Adc_BatHandle) != HAL_OK)
    {
        /* Initialization Error */
        usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
    }

    /*##-2- Configure ADC regular channel ######################################*/
    sConfig.Channel = ADCx_BAT_CHANNEL;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    sConfig.Offset = 0;
    if(HAL_ADC_ConfigChannel(&Adc_BatHandle, &sConfig) != HAL_OK)
    {
        /* Channel Configuration Error */
        usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
    }

    return 0;
}

float Bat_ComputeVol(uint16_t *pu16Buffer,uint8_t u8Len)
{
    uint16_t u16Tmp = 0;
    float fTemp = 0;

    for(int i = 0;i < u8Len;i++)
    {
        u16Tmp += *(pu16Buffer + i);
    }

    fTemp = (u16Tmp * BAT_VREF_VOL * 2) / (4095 * u8Len);

    #ifdef BAT_DEBUG
        printf("Batvol = %0.4f\r\n",fTemp);
    #endif
    return (fTemp);
}

int8_t Bat_CheckAlarm(float fVolTmp)
{
    static uint8_t u8BATVolDetectNum = 0;
    uint8_t i = 0;

    for(i = (BAT_VOL_ALARM_NUMBER - 1);i >= 1;i--)
    {
        fExtBatVolBuffer[i] = fExtBatVolBuffer[i - 1];
    }
    fExtBatVolBuffer[0] = fVolTmp;

    if(u8BATVolDetectNum < BAT_VOL_ALARM_NUMBER)
    {
        u8BATVolDetectNum++;
    }
    else
    {
        for(i = 0;i < BAT_VOL_ALARM_NUMBER;i++)
        {
            if(BAT_VREF_VOL <= *(fExtBatVolBuffer + i))
            {
                break;
            }
        }

        if(i >= BAT_VOL_ALARM_NUMBER)
        {
            return (-1);
        }
    }

    return 0;
}

void drvBAT_ItCallBack(uint16_t GPIO_Pin)
{
    uint32_t ulVar = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(GPIO_Pin == GPIO_PIN_BAT_ACOK)
    {
        if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_PORT_BAT_ACOK, GPIO_PIN_BAT_ACOK))
        {
            ulVar = BAT_EXTSUPPLY_PLUGGED_EVENT;
            xQueueSendToBackFromISR(xBatQueue, &ulVar, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        else
        {
            ulVar = BAT_EXTSUPPLY_UNPLUGGED_EVENT;
            xQueueSendToBackFromISR(xBatQueue, &ulVar, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_BAT_CHG)
    {
        if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_PORT_BAT_CHG, GPIO_PIN_BAT_CHG))
        {
            ulVar = BAT_CHARGE_START_EVENT;
            xQueueSendToBackFromISR(xBatQueue, &ulVar, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        else
        {
            ulVar = BAT_CHARGE_STOP_EVENT;
            xQueueSendToBackFromISR(xBatQueue, &ulVar, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

}

void vExtBatteryTask(void * pvParameters)
{
    uint32_t u32RxData = 0;
    HAL_StatusTypeDef status = HAL_OK;

    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_PORT_BAT_CHG, GPIO_PIN_BAT_CHG))
    {
        drvLed_PS_OrangeFlashStart();
    }
    else
    {
        drvLed_PS_GreenFlashStart();
    }
    Bat_ACOK_Init();
    BatVolDetectInit();
    vStartDetectTimer();
    WDT_Register(WDT_BAT_TASK_ID);

    /* Task loop */
    while(1)
    {
        if(pdTRUE == xQueueReceive(xBatQueue, &u32RxData, 10))
        {
            switch(u32RxData)
            {
                case BAT_EXTSUPPLY_PLUGGED_EVENT:
                    WDT_Feed(WDT_BAT_TASK_ID);
                    printf("BAT_EXTSUPPLY_PLUGGED_EVENT\r\n");
                    u8ExtPowerPlugged_Flag = 1;
                    break;

                case BAT_EXTSUPPLY_UNPLUGGED_EVENT:
                    WDT_Feed(WDT_BAT_TASK_ID);
                    printf("BAT_EXTSUPPLY_UNPLUGGED_EVENT\r\n");
                    u8ExtPowerPlugged_Flag = 0;
                    drvLed_PS_GreenDeLight();
                    break;

                case BAT_CHARGE_START_EVENT:
                    WDT_Feed(WDT_BAT_TASK_ID);
                    printf("Now start charging ...\r\n");
                    drvLed_PS_GreenFlashStop();
                    u8LowVoltAlarm_Flag = 0;
                    drvLed_PS_RedFlashStop();
                    drvLed_PS_OrangeFlashStart();
                    break;

                case BAT_CHARGE_STOP_EVENT:
                    WDT_Feed(WDT_BAT_TASK_ID);
                    printf("Now stop charging,%d ...\r\n",u8ExtPowerPlugged_Flag);
                    drvLed_PS_OrangeFlashStop();
                    if(u8ExtPowerPlugged_Flag)
                    {
                        // Full charge
                        drvLed_PS_GreenLight();
                    }
                    else
                    {
                        drvLed_PS_GreenFlashStart();
                    }
                    break;

                case BAT_DETECT_START_EVENT:
                    WDT_Feed(WDT_BAT_TASK_ID);
                    /* Start Conversation */
                    status = HAL_ADC_Start_DMA(&Adc_BatHandle, (uint32_t*)&uhADCxConvertedValue, BAT_VOL_BUFFER_LENGTH);
                    if(status != HAL_OK)
                    {
                        usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
                    }
                    break;

                case BAT_DETECT_END_EVENT:
                    HAL_ADC_Stop_DMA(&Adc_BatHandle);
                    WDT_Feed(WDT_BAT_TASK_ID);
                    if(-1 == Bat_CheckAlarm(Bat_ComputeVol(uhADCxConvertedValue, BAT_VOL_BUFFER_LENGTH)))
                    {
                        if(!u8LowVoltAlarm_Flag)
                        {
                            u8LowVoltAlarm_Flag = 1;
                            drvLed_PS_RedFlashStart();
                        }
                    }
                    else
                    {
                        if(u8LowVoltAlarm_Flag)
                        {
                            u8LowVoltAlarm_Flag = 0;
                            drvLed_PS_RedFlashStop();
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

// Function that creates a task.
void CreateExtBatteryTask( void )
{
    xTaskCreate( vExtBatteryTask, "BatTask", configMINIMAL_STACK_SIZE * 10, NULL, tskBATTERY_PRIORITY, &xBatTaskHandle );
    configASSERT( xBatTaskHandle );

	// Create a queue capable of containing 10 uint32_t values.
	xBatQueue = xQueueCreate( 10, sizeof( uint32_t ) );
    configASSERT( xBatQueue );

    xBatDetectTimers = xTimerCreate( "BatDetectTimers",
                               (8000/portTICK_PERIOD_MS),
                               pdTRUE,
                               NULL,
                               vBatDetectTimerCallback);
    configASSERT( xBatDetectTimers );
}


