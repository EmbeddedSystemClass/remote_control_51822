#ifndef _BATTERY_H_
#define _BATTERY_H_

#include "stm32f2xx_hal.h"
#include "includes.h"

#define BAT_EXTSUPPLY_PLUGGED_EVENT              (1 << 0)
#define BAT_EXTSUPPLY_UNPLUGGED_EVENT            (1 << 1)
#define BAT_CHARGE_START_EVENT                  (1 << 2)
#define BAT_CHARGE_STOP_EVENT                   (1 << 3)
#define BAT_DETECT_START_EVENT                  (1 << 4)
#define BAT_DETECT_END_EVENT                    (1 << 5)

#define BAT_VOL_BUFFER_LENGTH                   (5)
#define BAT_VOL_ALARM_NUMBER                    (3)

#define BAT_VREF_VOL                            (3.00)

#define BAT_ARLARM_VOL                          (3.3)



typedef enum {BAT_NORMAL, BAT_CHARGING} bat_state_e;

typedef struct{
        bat_state_e enBatState;
        float fBatVolt;
} battery_t;

extern battery_t battery;
extern QueueHandle_t xBatQueue;
extern ADC_HandleTypeDef Adc_BatHandle;

void CreateExtBatteryTask( void );
void drvBAT_ItCallBack(uint16_t GPIO_Pin);
int BatVolDetect(void);
void Bat_ConvCpltCallback(void);
int8_t vStartDetectTimer(void);

#endif  /* End of #ifndef _BATTERY_H_ */

