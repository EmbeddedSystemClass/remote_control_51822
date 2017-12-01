#ifndef __DRV_LED_H_
#define __DRV_LED_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

void Leds_Init(void);
void drvLed_PS_GreenFlashStart(void);
void drvLed_PS_GreenFlashStop(void);
void drvLed_PS_RedFlashStart(void);
void drvLed_PS_RedFlashStop(void);
void drvLed_PS_OrangeLight(void);
void drvLed_PS_OrangeDeLight(void);
void drvLed_PS_OrangeFlashStart(void);
void drvLed_PS_OrangeFlashStop(void);

void drvLed_GreenFlashStart(void);
void drvLed_BlueLight(void);
void drvLed_BlueDeLight(void);
void drvLed_PS_GreenLight(void);
void drvLed_PS_GreenDeLight(void);


#endif /* __DRV_RTC_H_ */

/***************************** END OF FILE *****************************/

