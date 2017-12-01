#include "drv_sys.h"
#include "includes.h"

HAL_StatusTypeDef drvSys_PwrOff(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    printf("drvSys_PwrOff\r\n");

    HAL_GPIO_WritePin(GPIO_PORT_SYS_OFF, GPIO_PIN_SYS_OFF, GPIO_PIN_SET);
    vTaskDelay(100/portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(GPIO_PORT_SYS_OFF, GPIO_PIN_SYS_OFF, GPIO_PIN_RESET);

    return (status);
}
