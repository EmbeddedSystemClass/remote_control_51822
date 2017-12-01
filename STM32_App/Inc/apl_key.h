#ifndef _APL_KEY_H_
#define _APL_KEY_H_

#include "includes.h"

#define KEY_MAX_CNT                     (10)

#define KEY0_SHORT_INPUT                (1 << 0)
#define KEY0_LONG_INPUT                 (1 << 1)
#define KEY1_SHORT_INPUT                (1 << 2)
#define KEY1_LONG_INPUT                 (1 << 3)

extern uint8_t u8KeyPressed;
extern QueueHandle_t xQueue_Key;
extern SemaphoreHandle_t xSemaphore_Key;


void Key_Init(void);
void CreateKeyTask(void);
void mid_Coap_Init(void);


#endif  /* End of #ifndef _APL_KEY_H_ */

