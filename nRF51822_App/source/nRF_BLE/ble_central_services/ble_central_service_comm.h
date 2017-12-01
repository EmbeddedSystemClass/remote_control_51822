/**
  ****************************************************************************************
  * @file    ble_central_service_comm.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-9
  * @brief   common define of ble central
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_CENTRAL_SERVICE_COMM_H_
#define __BLE_CENTRAL_SERVICE_COMM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/**@brief Structure containing the handles related to the Service found on the peer. */
typedef struct
{
    uint16_t cccd_handle;       /**< Handle of the CCCD of the characteristic. */
    uint16_t char_handle;       /**< Handle of the characteristic as provided by the SoftDevice. */    
}char_db_t;

/**
 * @defgroup bas_c_enums Enumerations
 * @{
 */
typedef enum
{
    READ_REQ, /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ /**< Type identifying that this tx_message is a write request. */
} TX_REQUEST_T;

/**
 * @brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                     gattc_value[20]; /**< The message to write. */
    ble_gattc_write_params_t    gattc_params; /**< The GATTC parameters for this message. */
} write_params_t;

/**
 * @brief Structure for Bonding service
 */
typedef struct
{
    char_db_t           pwdWriteCharW;
    char_db_t           pwdResultCharR;
}bonding_service_t;

/**
 * @brief Structure for device information manage service
 */
typedef struct
{
    char_db_t           syncTimeCharW;
    char_db_t           nameModifyCharW;
    char_db_t           versionGetCharR;
    char_db_t           batLevelGetCharR;
}devinfo_manage_service_t;

/**
 * @brief Structure for monitor template service
 */
typedef struct
{
    char_db_t           monitorTemplateSetCharW;
    char_db_t           monitorTemplateSetResultCharR;
}monitor_template_service_t;

/**
 * @brief Structure for data sync service
 */
typedef struct
{
    char_db_t           syncDataSwitchCharW;
    char_db_t           syncDataCharR;
    char_db_t           syncDataDoneCharW;
}sync_data_service_t;


#endif /* __BLE_CENTRAL_SERVICE_COMM_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



