#ifndef __CRC16_H
#define __CRC16_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

uint32_t crc32(uint32_t crc,uint8_t *buffer, uint32_t size);
uint16_t crc32_To_crc16(uint32_t crc32);
uint8_t CRC_8bytes_Calculate(uint32_t crc_init,uint8_t *buffer, uint32_t size) ;

#endif /* __CRC_H */


