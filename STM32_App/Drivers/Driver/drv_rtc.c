/**
  ****************************************************************************************
  * @file    bsp_rtc.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-9-18
  * @brief   bsp_rtc.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "drv_rtc.h"
#include "includes.h"

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;         //RTC handler declaration
static void RTC_CalendarConfig(void);
static RTC_Wakeup_Callback m_pRTC_Wakup_Callback;
static RTC_AlarmA_Callback m_pRTC_AlarmA_Callback;
static RTC_AlarmB_Callback m_pRTC_AlarmB_Callback;


/* Private function prototypes -----------------------------------------------*/
static void Calendar_WeekDayNum(const RTC_DateTypeDef* date_s, week_str_typedef *week_s);



/**
  * @brief  bsp_rtc_init
  * @note   bsp rtc init
  * @param  None
  * @retval HAL status
  */
HAL_StatusTypeDef bsp_rtc_init(void)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    /*##-1- Configure the RTC peripheral #######################################*/
    /* Configure RTC prescaler and RTC data registers */
    /* RTC configured as follows:
        - Hour Format    = Format 24
        - Asynch Prediv  = Value according to source clock
        - Synch Prediv   = Value according to source clock
        - OutPut         = Output Disable
        - OutPutPolarity = High Polarity
        - OutPutType     = Open Drain
    */
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
    }

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
    }

    /*##-2- Check if Data stored in BackUp register1: No Need to reconfigure RTC#*/
    /* Read the Back Up Register 1 Data */
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
    {
        /* Configure RTC Calendar */
        RTC_CalendarConfig();
    }
    else
    {
        /* Check if the Power On Reset flag is set */
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
        {

        }
        /* Check if Pin Reset flag is set */
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
        {

        }
        /* Clear source Reset Flag */
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }

    return ret;
}
/**
  * @brief  get the current time and date.
  * @param  date_s:  pointer to buffer
  * @param  rtc_time: pointer to buffer
  * @retval status
  */
HAL_StatusTypeDef calendar_get(RTC_DateTypeDef *date_s,RTC_TimeTypeDef *rtc_time)
{
    RTC_DateTypeDef         sdatestructureget;
    RTC_TimeTypeDef         stimestructureget;
    HAL_StatusTypeDef       ret = HAL_ERROR;

    /* Get the RTC current Time */
    ret = HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
    usr_res_assert(ret == HAL_OK);

    /* Get the RTC current Date */
    ret = HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
    usr_res_assert(ret == HAL_OK);

    date_s->Year = sdatestructureget.Year;
    date_s->Month = sdatestructureget.Month;
    date_s->Date = sdatestructureget.Date;
    date_s->WeekDay = sdatestructureget.WeekDay;

    rtc_time->Hours = stimestructureget.Hours;
    rtc_time->Minutes = stimestructureget.Minutes;
    rtc_time->Seconds = stimestructureget.Seconds;

    return ret;
}

/**
  * @brief  set the current time and date.
  * @param  date_s:  pointer to buffer
  * @param  rtc_time: pointer to buffer
  * @retval status
  */
HAL_StatusTypeDef calendar_set(RTC_DateTypeDef *date_s,RTC_TimeTypeDef *rtc_time)
{
    RTC_DateTypeDef         sdatestructure;
    RTC_TimeTypeDef         stimestructure;
    week_str_typedef        m_rtc_week;
    HAL_StatusTypeDef       ret = HAL_ERROR;

    /* Convent the date to week */
    Calendar_WeekDayNum(date_s, &m_rtc_week);

    /* Get Date */
    sdatestructure.Year = date_s->Year;
    sdatestructure.Month = date_s->Month;
    sdatestructure.Date = date_s->Date;
    sdatestructure.WeekDay = m_rtc_week.u8WkDayNum;

    /* Set Date */
    ret = HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BIN);
    usr_res_assert(ret == HAL_OK);

    /* Get Time */
    stimestructure.Hours = rtc_time->Hours;
    stimestructure.Minutes = rtc_time->Minutes;
    stimestructure.Seconds = rtc_time->Seconds;
    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    /* Set Time */
    ret = HAL_RTC_SetTime(&hrtc,&stimestructure,RTC_FORMAT_BIN);
    usr_res_assert(ret == HAL_OK);

    return ret;
}
/**
  * @brief  Wake Up Timer callback.
  * @param  hrtc: RTC handle
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
//    printf("\r\nHi!\r\n");
    if(m_pRTC_Wakup_Callback != NULL)
    {
        m_pRTC_Wakup_Callback();
    }
}
/**
  * @brief  calendar_rtc_period_wakeup_init()
  * @brief  init the RTC to period wakeup, resolution = 1s/(2^15)
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef RTC_Period_Wakeup_Init(uint32_t u32SecCnt, RTC_Wakeup_Callback pCallBack)
{
    HAL_StatusTypeDef       ret = HAL_ERROR;

    /*Init the callback function*/
    m_pRTC_Wakup_Callback = pCallBack;

    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    /*Configure the NVIC for RTC Wake up ###################################*/
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

    /* Set the wake up time */
    ret = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,u32SecCnt,RTC_WAKEUPCLOCK_RTCCLK_DIV2);

    return ret;
}
/**
  * @brief  calendar_rtc_period_wakeup_deinit()
  * @brief  deinit the RTC to period wakeup
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef RTC_Period_Wakeup_Deinit(void)
{
    HAL_StatusTypeDef       ret = HAL_ERROR;

    /*Reset the callback function*/
    m_pRTC_Wakup_Callback = NULL;

    ret = (HAL_StatusTypeDef)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    return ret;
}

/**
  * @brief  AlarmA callback
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
//    printf("RTC AlarmA\r\n");
    if(m_pRTC_AlarmA_Callback!=NULL)
    {
        m_pRTC_AlarmA_Callback();
    }
}

/**
  * @brief  set the alarm A.
  * @param  Alarm_HH,Alarm_MM,Alarm_SS
  * @retval status
  */
HAL_StatusTypeDef rtc_set_alarm_A(uint8_t Alarm_HH,uint8_t Alarm_MM,uint8_t Alarm_SS,RTC_AlarmA_Callback pCallBack)
{
    HAL_StatusTypeDef       ret = HAL_ERROR;
    RTC_AlarmTypeDef        salarmstructure;

    /*Init the callback function*/
    m_pRTC_AlarmA_Callback = pCallBack;

    /* Disable the Alarm */
    HAL_RTC_DeactivateAlarm(&hrtc,RTC_ALARM_A);

    /* Configure the RTC Alarm peripheral #################################*/
    /* Set Alarm , RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
    salarmstructure.Alarm = RTC_ALARM_A;
    salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
    salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_PM;
    salarmstructure.AlarmTime.Hours = Alarm_HH;
    salarmstructure.AlarmTime.Minutes = Alarm_MM;
    salarmstructure.AlarmTime.Seconds = Alarm_SS;
    ret = HAL_RTC_SetAlarm_IT(&hrtc,&salarmstructure,RTC_FORMAT_BIN);

    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

    return ret;
}
/**
  * @brief  disable the alarm A.
  * @param  None
  * @retval status
  */
HAL_StatusTypeDef rtc_disable_alarm_A(void)
{
    HAL_StatusTypeDef       ret = HAL_ERROR;

    /*Reset the callback function*/
    m_pRTC_AlarmA_Callback = NULL;

    /* Disable the Alarm */
    ret = HAL_RTC_DeactivateAlarm(&hrtc,RTC_ALARM_A);

    return ret;
}
/**
  * @brief  AlarmB callback
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
//    printf("RTC AlarmB\r\n");
    if(m_pRTC_AlarmB_Callback!=NULL)
    {
        m_pRTC_AlarmB_Callback();
    }
}

/**
  * @brief  set the alarm B.
  * @param  Alarm_HH,Alarm_MM,Alarm_SS
  * @retval status
  */
HAL_StatusTypeDef rtc_set_alarm_B(uint8_t Alarm_HH,uint8_t Alarm_MM,uint8_t Alarm_SS,RTC_AlarmB_Callback pCallBack)
{
    HAL_StatusTypeDef       ret = HAL_ERROR;
    RTC_AlarmTypeDef        salarmstructure;

    /*Init the callback function*/
    m_pRTC_AlarmB_Callback = pCallBack;

    /* Disable the Alarm */
    HAL_RTC_DeactivateAlarm(&hrtc,RTC_ALARM_B);

    /* Configure the RTC Alarm peripheral #################################*/
    /* Set Alarm , RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
    salarmstructure.Alarm = RTC_ALARM_B;
    salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
    salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
    salarmstructure.AlarmTime.Hours = Alarm_HH;
    salarmstructure.AlarmTime.Minutes = Alarm_MM;
    salarmstructure.AlarmTime.Seconds = Alarm_SS;
    ret = HAL_RTC_SetAlarm_IT(&hrtc,&salarmstructure,RTC_FORMAT_BIN);

    return ret;
}
/**
  * @brief  disable the alarm B.
  * @param  None
  * @retval status
  */
HAL_StatusTypeDef rtc_disable_alarm_B(void)
{
    HAL_StatusTypeDef       ret = HAL_ERROR;

    /*Reset the callback function*/
    m_pRTC_AlarmB_Callback = NULL;

    /* Disable the Alarm */
    ret = HAL_RTC_DeactivateAlarm(&hrtc,RTC_ALARM_B);

    return ret;
}

/**
* @brief  将时、分、秒时间转换为unix时间
  * @param  date
  * @retval None
  */
uint32_t GetTick(int YY,int MM,int DD,int HH,int MMin,int SS)
{
    struct tm stm;
    uint32_t tim = 0;

    memset(&stm,0,sizeof(stm));

    stm.tm_year=YY-1900;
    stm.tm_mon=MM-1;
    stm.tm_mday=DD;
    stm.tm_hour=HH;
    stm.tm_min=MMin;
    stm.tm_sec=SS;

    tim = mktime(&stm);

    return (tim - 28800);  //转换为北京时间
}

uint32_t CovertTimetoUnix(void)
{
	uint32_t date=0;

	RTC_DateTypeDef     date_s;      //RTC 日期
	RTC_TimeTypeDef     rtc_time;    //RTC 时间

    calendar_get(&date_s,&rtc_time); //获取当前时间

	date = GetTick(date_s.Year + 2000, date_s.Month, date_s.Date, rtc_time.Hours,rtc_time.Minutes,rtc_time.Seconds);

	return date;
}
/**
* @brief  将unix时间转换为时分秒
  * @param  tim
  * @retval None
  */
void OutputGMTIME(long tim,RTC_DateTypeDef *date_s,RTC_TimeTypeDef *rtc_time)
{
	time_t t=0;
	struct tm *p;

	t = tim + 28800; //转换为北京时间
	p = localtime(&t);

    date_s->Year = p->tm_year+1900;
    date_s->Month = p->tm_mon+1;
    date_s->Date = p->tm_mday;

    rtc_time->Hours = p->tm_hour;
    rtc_time->Minutes = p->tm_min;
    rtc_time->Seconds = p->tm_sec;
}
/**
  * @brief  将当前日期转换为32位数据标示
  * @param  None
  * @retval date
  */
uint32_t CovernDateto32(void)
{
	uint32_t date=0;

	RTC_DateTypeDef     date_s;      //RTC 日期
	RTC_TimeTypeDef     rtc_time;    //RTC 时间

    calendar_get(&date_s,&rtc_time); //获取当前时间

	date = (uint32_t)(((date_s.Year+2000) << 16) | (date_s.Month << 8) | (date_s.Date));

	return date;
}
/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */
static void RTC_CalendarConfig(void)
{
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;
    HAL_StatusTypeDef ret = HAL_ERROR;

    /*##-1- Configure the Date #################################################*/
    /* Set Date: Tuesday February 18th 2014 */
    sdatestructure.Year = 0x14;
    sdatestructure.Month = RTC_MONTH_FEBRUARY;
    sdatestructure.Date = 0x18;
    sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;

    ret = HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD);
    usr_res_assert(ret == HAL_OK);

    /*##-2- Configure the Time #################################################*/
    /* Set Time: 02:00:00 */
    stimestructure.Hours = 0x02;
    stimestructure.Minutes = 0x00;
    stimestructure.Seconds = 0x00;
    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    ret = HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD);
    usr_res_assert(ret == HAL_OK);

    /*##-3- Writes a data in a RTC Backup data Register1 #######################*/
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
}
/**
  * @brief  Determines the week number, the day number and the week day number.
  * @param  None
  * @retval None
  */
static void Calendar_WeekDayNum(const RTC_DateTypeDef* date_s, week_str_typedef *week_s)
{
  uint32_t a = 0, b = 0, c = 0, s = 0, e = 0, f = 0, g = 0, d = 0;
  int32_t n = 0;
  if (date_s->Month < 3)
  {
    a = date_s->Year - 1;
  }
  else
  {
    a = date_s->Year;
  }

  b = (a / 4) - (a / 100) + (a / 400);
  c = ((a - 1) / 4) - ((a - 1) / 100) + ((a - 1) / 400);
  s = b - c;
  if (date_s->Month < 3)
  {
    e = 0;
    f =  date_s->Date - 1 + 31 * (date_s->Month - 1);
  }
  else
  {
    e = s + 1;
    f = date_s->Date + (153 * (date_s->Month - 3) + 2) / 5 + 58 + s;
  }
  g = (a + b) % 7;
  d = (f + g - e) % 7;
  n = f + 3 - d;
  if (n < 0)
  {
    week_s->u8WkNum = 53 - ((g - s) / 5);
  }
  else if (n > (364 + s))
  {
    week_s->u8WkNum = 1;
  }
  else
  {
    week_s->u8WkNum = (n / 7) + 1;
  }
  week_s->u8WkDayNum = d + 1;
  week_s->u8DayNum = f + 1;
}

HAL_StatusTypeDef RTC_validate(time_t unix_time)
{
    HAL_StatusTypeDef status = HAL_OK;
    struct tm * g_SysTime;
    RTC_DateTypeDef date;
    RTC_TimeTypeDef rtc_time;

    g_SysTime = localtime (&unix_time);
    printf("current time: %d-%d-%d,%d:%d:%d\r\n",g_SysTime->tm_year + 1900,
                                                        g_SysTime->tm_mon + 1,
                                                        g_SysTime->tm_mday,
                                                        g_SysTime->tm_hour + 8,
                                                        g_SysTime->tm_min,
                                                        g_SysTime->tm_sec);
    date.Year = g_SysTime->tm_year + 1900 - 2000;
    date.Month = g_SysTime->tm_mon + 1;
    date.Date = g_SysTime->tm_mday;
    date.WeekDay = g_SysTime->tm_wday;
    rtc_time.Hours = g_SysTime->tm_hour + 8;
    rtc_time.Minutes = g_SysTime->tm_min;
    rtc_time.Seconds = g_SysTime->tm_sec;
    status = calendar_set(&date, &rtc_time);

    return (status);
}

/**
  * @brief  Display the current time and date.
  * @param  showtime : pointer to buffer
  * @param  showdate : pointer to buffer
  * @retval None
  */
void RTC_CalendarShow(void)
{
    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;

    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

    /* Display date Format : mm-dd-yy,time Format : hh:mm:ss */
//    printf("%02d-%02d-%02d,%02d:%02d:%02d\r\n",
//                            sdatestructureget.Month,
//                            sdatestructureget.Date,
//                            2000 + sdatestructureget.Year,
//                            stimestructureget.Hours,
//                            stimestructureget.Minutes,
//                            stimestructureget.Seconds);
}

void App_RTC_Wakup_Callback(void)
{
    /*uint32_t ulVar = KEY1_LONG_INPUT;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendToBackFromISR(xQueue_Key, &ulVar, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);*/
}

void App_RTC_AlarmA_Callback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSemaphore_FW, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  get the rtc in unix time
  * @param[out]  uint32_t *tim
  * @retval None
  */
void get_time_inUnix(uint32_t *tim)
{
    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;
    struct tm stm;

    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

    stm.tm_year = sdatestructureget.Year + 2000 - 1900;
    stm.tm_mon = sdatestructureget.Month - 1;
    stm.tm_mday = sdatestructureget.Date;
    stm.tm_hour = stimestructureget.Hours-8;
    stm.tm_min = stimestructureget.Minutes;
    stm.tm_sec = stimestructureget.Seconds;

    *tim = mktime(&stm);
}

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

