/**
  ****************************************************************************************
  * @file    bsp_spis.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-14
  * @brief   bsp spi slave
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp_spis.h"

#define SPIS_INSTANCE   1 /**< SPIS instance index. */

static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
static void spis_event_handler(nrf_drv_spis_event_t event);

static uint8_t       m_tx_buf[MAX_SPI_LEN] = {0};   /**< TX buffer. */
static uint8_t       m_rx_buf[MAX_SPI_LEN] = {0};   /**< RX buffer. */
static uint8_t       m_length = MAX_SPI_LEN;        /**< Transfer length. */

/**
  * @brief  bsp_spi_slave_config  
  * @param  None
  * @retval None
  */
void bsp_spi_slave_config(void)
{
    /* REQN(request spi master to read data) Pin config */
    nrf_gpio_cfg_output(BT_REQN);
    nrf_gpio_pins_set(BT_REQN);    
    
    // Enable the constant latency sub power mode to minimize the time it takes
    // for the SPIS peripheral to become active after the CSN line is asserted
    // (when the CPU is in sleep mode).
    NRF_POWER->TASKS_CONSTLAT = 1;

    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = SPIS_CSN_PIN;
    spis_config.miso_pin              = SPIS_MISO_PIN;
    spis_config.mosi_pin              = SPIS_MOSI_PIN;
    spis_config.sck_pin               = SPIS_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler)); 

    nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length);    
}
/**
  * @brief  bsp_spi_slave_transmit  
  * @param[in]  uint8_t *data,uint8_t len
  * @retval status
  */
ret_code_t bsp_spi_slave_transmit(uint8_t *data,uint8_t len)
{
    ret_code_t          status = NRF_ERROR_NULL;
    
    if(len > MAX_SPI_LEN)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    
    memset(m_tx_buf,0,MAX_SPI_LEN);
    memcpy(m_tx_buf,data,len);
    
    status = nrf_drv_spis_buffers_set(&spis, m_tx_buf, len, m_rx_buf, m_length);
    
    /* notice the spi master to read the data out */
    nrf_gpio_pin_clear(BT_REQN);
    nrf_gpio_pin_set(BT_REQN);
    nrf_gpio_pin_clear(BT_REQN);
    
    return status;
}

/**
  * @brief  spis_event_handler  
  * @param  event
  * @retval None
  */
static void spis_event_handler(nrf_drv_spis_event_t event)
{
//    uint8_t         i = 0;
    APP_CMU_MANAGE_MSG_T    cmuMsg;
    int32_t                 err_code = NRF_ERROR_NULL;
    
    if(event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        if(event.rx_amount > 0) //接收到数据完成
        {
//            printf("Rx_len%d:",event.rx_amount);
//            for(i=0;i<event.rx_amount;i++)
//            {
//                printf("0x%x,",m_rx_buf[i]);
//            }
//            printf("\r\n");
            
            cmuMsg.eventID = EVENT_APP_CMU_SPI_RX;
            cmuMsg.len = (uint8_t)(event.rx_amount);
            memcpy(g_data_cmubuf,m_rx_buf,cmuMsg.len);
            cmuMsg.p_data = g_data_cmubuf;
            err_code = app_sched_event_put(&cmuMsg,sizeof(cmuMsg),app_cmu_manage_task_handler);
            APP_ERROR_CHECK(err_code);             
            
            memset(m_rx_buf,0,MAX_SPI_LEN);
        }
        
        if(event.tx_amount > 0) //发送数据完成
        {
            memset(m_tx_buf,0,MAX_SPI_LEN);           
        }
        APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, MAX_SPI_LEN, m_rx_buf, MAX_SPI_LEN)); 
    }
}






/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/


