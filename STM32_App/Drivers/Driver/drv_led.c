#include "drv_led.h"
#include "includes.h"


TimerHandle_t xPSRedLedFlashTimer = NULL;
TimerHandle_t xPSGreenLedFlashTimer = NULL;
TimerHandle_t xPSOrangeLedFlashTimer = NULL;

TimerHandle_t xSSGreenLedFlashTimer = NULL;


// PS Green Led
void drvLed_PS_GreenLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G, GPIO_PIN_RESET);
}

void drvLed_PS_GreenDeLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G, GPIO_PIN_SET);
}

void drvLed_PS_GreenFlash(void)
{
    HAL_GPIO_TogglePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G);
}

void drvLed_PS_GreenFlashStart(void)
{
    xTimerStart(xPSGreenLedFlashTimer, 0);
    drvLed_PS_GreenLight();
}

void drvLed_PS_GreenFlashStop(void)
{
    xTimerStop(xPSGreenLedFlashTimer, 0);
    drvLed_PS_GreenDeLight();
}


// PS Red Led
void drvLed_PS_RedLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R, GPIO_PIN_RESET);
}

void drvLed_PS_RedDeLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R, GPIO_PIN_SET);
}

void drvLed_PS_RedFlash(void)
{
    HAL_GPIO_TogglePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R);
}

void drvLed_PS_RedFlashStart(void)
{
    xTimerStart(xPSRedLedFlashTimer, 0);
    drvLed_PS_RedLight();
}

void drvLed_PS_RedFlashStop(void)
{
    xTimerStop(xPSRedLedFlashTimer, 0);
    drvLed_PS_RedDeLight();
}

// PS Orange Led
void drvLed_PS_OrangeLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R, GPIO_PIN_RESET);
}

void drvLed_PS_OrangeDeLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R, GPIO_PIN_SET);
}

void drvLed_PS_OrangeFlashStart(void)
{
    xTimerStart(xPSOrangeLedFlashTimer, 0);
    drvLed_PS_OrangeLight();
}

void drvLed_PS_OrangeFlashStop(void)
{
    xTimerStop(xPSOrangeLedFlashTimer, 0);
    drvLed_PS_OrangeDeLight();
}

void drvLed_PS_OrangeFlash(void)
{
    HAL_GPIO_TogglePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R);
    HAL_GPIO_TogglePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G);
}

// SS Green Led
void drvLed_SS_GreenLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_SS_R, GPIO_PIN_LED_SS_R, GPIO_PIN_RESET);
}

void drvLed_SS_GreenDeLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_SS_R, GPIO_PIN_LED_SS_R, GPIO_PIN_SET);
}

void drvLed_SS_GreenFlashStart(void)
{
    xTimerStart(xSSGreenLedFlashTimer, 0);
    drvLed_SS_GreenLight();
}

void drvLed_SS_GreenFlashStop(void)
{
    xTimerStop(xSSGreenLedFlashTimer, 0);
    drvLed_SS_GreenDeLight();
}

void drvLed_SS_GreenFlash(void)
{
    HAL_GPIO_TogglePin(GPIO_PORT_LED_SS_R, GPIO_PIN_LED_SS_R);
}


// PS Blue Led
void drvLed_BlueLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R, GPIO_PIN_RESET);
}

void drvLed_BlueDeLight(void)
{
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R, GPIO_PIN_RESET);
}

// Timer Handler
void vPSGreenLedFlashTimerCallback( TimerHandle_t pxTimer )
{
    drvLed_PS_GreenFlash();
}

void vPSRedLedFlashTimerCallback( TimerHandle_t pxTimer )
{
    drvLed_PS_RedFlash();
}

void vPSOrangeLedFlashTimerCallback( TimerHandle_t pxTimer )
{
    drvLed_PS_OrangeFlash();
}


void Leds_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /*Configure GPIO pins : LED_PS_G */
    GPIO_InitStruct.Pin = GPIO_PIN_LED_PS_G;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_PORT_LED_PS_G, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_PS_R */
    GPIO_InitStruct.Pin = GPIO_PIN_LED_PS_R;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_PORT_LED_PS_R, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_PS_R */
    GPIO_InitStruct.Pin = GPIO_PIN_LED_SS_B;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_PORT_LED_SS_B, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_PS_R */
    GPIO_InitStruct.Pin = GPIO_PIN_LED_SS_G;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_PORT_LED_SS_G, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_PS_R */
    GPIO_InitStruct.Pin = GPIO_PIN_LED_SS_R;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_PORT_LED_SS_R, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_G, GPIO_PIN_LED_PS_G, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_PS_R, GPIO_PIN_LED_PS_R, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_SS_B, GPIO_PIN_LED_SS_B, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_SS_G, GPIO_PIN_LED_SS_G, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_LED_SS_R, GPIO_PIN_LED_SS_R, GPIO_PIN_SET);

    xPSGreenLedFlashTimer = xTimerCreate( "xPSGreenLedFlashTimer",(1000/portTICK_PERIOD_MS),
                                    pdTRUE,NULL,vPSGreenLedFlashTimerCallback);
    configASSERT( xPSGreenLedFlashTimer );

    xPSRedLedFlashTimer = xTimerCreate( "PSRedLedFlashTimer",(1000/portTICK_PERIOD_MS),
                                    pdTRUE,NULL,vPSRedLedFlashTimerCallback);
    configASSERT( xPSRedLedFlashTimer );

    xPSOrangeLedFlashTimer = xTimerCreate( "PSOrangeLedFlashTimer",(1000/portTICK_PERIOD_MS),
                                    pdTRUE,NULL,vPSOrangeLedFlashTimerCallback);
    configASSERT( xPSOrangeLedFlashTimer );

}


