/******************************************************************************
 *
 * COPYRIGHT:
 *   Copyright (c)  2005-2050   GnsTime  Inc.    All rights reserved.
 *
 *   This is unpublished proprietary source code of GnsTime Inc.
 *   The copyright notice above does not evidence any actual or intended
 *   publication of such source code.
 *
 * FILE NAME:
 *   ymodem.c
 * DESCRIPTION:
 *   FATFS *fs, fatfs;
 * HISTORY:
 *   2013年8月10日        Arvin         Create/Update
 *   2016年3月13日        Arvin         Update  更新使用的ymode，原Ymode存在文件大小问题
 *
 *****************************************************************************/

/** @addtogroup FileSystem
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "ymodem.h"
#include "includes.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define CRC16_F
extern uint8_t fileName[];
extern uint8_t yModeBuf[1024];
uint8_t aPacketData[PACKET_1K_SIZE + PACKET_DATA_INDEX + PACKET_TRAILER_SIZE];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Receive byte from sender
 * @param  c: Character
 * @param  timeout: Timeout
 * @retval 0: Byte received
 *        -1: Timeout
 */
static MID_STATUS_TYPE_T Receive_Byte (uint8_t *c, uint32_t timeout)
{
    *c = mid_CliGetChar ();
    if (*c != NULL)
        return MID_OK;
    return MID_ERROR;
}

/**
 * @brief  Send a byte
 * @param  c: Character
 * @retval 0: Byte sent
 */
static uint32_t Send_Byte (uint8_t c)
{
    mid_CliSendData (&c, 1);

    return 0;
}

/**
 * @brief  Receive a packet from sender
 * @param  data
 * @param  length
 * @param  timeout
 *     0: end of transmission
 *    -1: abort by sender
 *    >0: packet length
 * @retval 0: normally return
 *        -1: timeout or packet error
 *         1: abort by user
 */
static MID_STATUS_TYPE_T Receive_Packet (uint8_t *p_data, uint32_t *p_length, uint32_t timeout)
{
    uint32_t crc;
    uint32_t packet_size = 0;
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint8_t char1;

    *p_length = 0;
    status = Receive_Byte (&char1, timeout);
    if (status == MID_OK)
    {
        switch (char1)
        {
            case SOH:
                packet_size = PACKET_SIZE;
                break;

            case STX:
                packet_size = PACKET_1K_SIZE;
                break;

            case EOT:
                status = MID_OK;
                break;

            case CA:
                if ((Receive_Byte (&char1, timeout) == MID_OK) && (char1 == CA))
                {
                    packet_size = 2;
                }
                else
                {
                    status = MID_ERROR;
                }
                break;

            case ABORT1:
            case ABORT2:
                status = MID_BUSY;
                break;

            default:
                status = MID_ERROR;
                break;
        }

        *p_data = char1;
        if (packet_size >= PACKET_SIZE)
        {
            status = mid_CliGetChars (&p_data[PACKET_NUMBER_INDEX],packet_size + PACKET_OVERHEAD_SIZE);
            /* Simple packet sanity check */
            if (status == MID_OK)
            {
                if (p_data[PACKET_NUMBER_INDEX] != ((p_data[PACKET_CNUMBER_INDEX]) ^ NEGATIVE_BYTE))
                {
                    packet_size = 0;
                    status = MID_ERROR;
                }
                else
                {
                    /* Check packet CRC */
                    crc = p_data[packet_size + PACKET_DATA_INDEX] << 8;
                    crc += p_data[packet_size + PACKET_DATA_INDEX + 1];
                    if (Cal_CRC16 (&p_data[PACKET_DATA_INDEX], packet_size) != crc)
                    {
                        packet_size = 0;
                        status = MID_ERROR;
                    }
                }
            }
            else
            {
                packet_size = 0;
            }
        }
    }

    *p_length = packet_size;

    return status;
}

/**
 * @brief  Receive a file using the ymodem protocol.
 * @param  buf: Address of the first byte.
 * @retval The size of the file.
 */
int32_t Ymodem_Receive (uint8_t *buf, FIL *pFile)
{
    MID_STATUS_TYPE_T result = MID_OK;
    uint32_t i, packet_length, session_done = 0, file_done, errors = 0,session_begin = 0;
    uint32_t filesize;
    uint8_t *file_ptr;
    char file_size[FILE_SIZE_LENGTH];
    uint32_t packets_received;
    uint32_t byteswritten1;
    uint8_t aFileName[FILE_NAME_LENGTH_YM];
    uint32_t writeLeft = 0;
    uint32_t writeLength = 0;
    FRESULT res = FR_OK;

    while ((session_done == 0) && (result == MID_OK))
    {
        packets_received = 0;
        file_done = 0;
        while ((file_done == 0) && (result == MID_OK))
        {
            switch (Receive_Packet (aPacketData, &packet_length, NAK_TIMEOUT))
            {
                case MID_OK:
                    errors = 0;
                    switch (packet_length)
                    {
                        case 2:
                            /* Abort by sender */
                            Send_Byte (Y_ACK);
                            result = MID_ERROR;
                            break;

                        case 0:
                            /* End of transmission */
                            Send_Byte (Y_ACK);
                            file_done = 1;
                            f_close (pFile);
                            break;

                        default:
                            /* Normal packet */
                            if (packets_received == 0)
                            {
                                /* File name packet */
                                if (aPacketData[PACKET_DATA_INDEX] != 0)
                                {
                                    /* File name extraction */
                                    i = 0;
                                    file_ptr = aPacketData + PACKET_DATA_INDEX;
                                    while ((*file_ptr != 0) && (i < FILE_NAME_LENGTH_YM))
                                    {
                                        aFileName[i++] = *file_ptr++;
                                    }

                                    /* File size extraction */
                                    aFileName[i++] = '\0';
                                    i = 0;
                                    file_ptr++;
                                    while ((*file_ptr != ' ') && (i < FILE_SIZE_LENGTH))
                                    {
                                        file_size[i++] = *file_ptr++;
                                    }
                                    file_size[i] = '\0';/*原为file_size[i++] = '\0'，但如果*file_ptr != ' '没有空格，则会溢出*/
                                    //Str2Int (file_size, (int32_t *)&filesize);
                                    filesize = atoi(file_size);
                                    if ((f_open (pFile, (char *) aFileName,FA_CREATE_NEW | FA_WRITE) != FR_OK))
                                    {
                                        Send_Byte (CA);
                                        Send_Byte (CA);
                                        return MID_ERROR;
                                    }

                                    writeLeft = filesize;
                                    Send_Byte (Y_ACK);
                                    Send_Byte (CRC16);
                                }
                                /* File header packet is empty, end session */
                                else
                                {
                                    Send_Byte (Y_ACK);
                                    file_done = 1;
                                    session_done = 1;
                                    break;
                                }
                            }
                            else /* Data packet */
                            {
                                writeLength = writeLeft >= packet_length ? packet_length : writeLeft;
                                res = f_write (pFile, &aPacketData[PACKET_DATA_INDEX],writeLength, (void *) &byteswritten1);
                                /* Write received data in Flash */
                                if ((res != FR_OK) || (byteswritten1 != writeLength))
                                {
                                    /* An error occurred while writing to Flash memory */
                                    /* End session */
                                    Send_Byte (CA);
                                    Send_Byte (CA);
                                    return MID_ERROR;
                                }
                                else
                                {
                                    writeLeft = (writeLeft >= packet_length) ? (writeLeft - writeLength) : 0;
                                    res = f_lseek (pFile, pFile->fsize);
                                    if (res != FR_OK)
                                    {
                                        /* file Read or EOF Error */
                                        /* End session */
                                        Send_Byte (CA);
                                        Send_Byte (CA);
                                        return MID_ERROR;
                                    }

                                    Send_Byte (Y_ACK);
                                }
                            }

                            packets_received++;
                            session_begin = 1;
                            break;

                }
                break;

            case MID_BUSY: /* Abort actually */
                Send_Byte (CA);
                Send_Byte (CA);
                result = MID_ERROR;
                break;

            default:
                if (session_begin > 0)
                {
                    errors++;
                }

                if (errors > MAX_ERRORS)
                {
                    /* Abort communication */
                    Send_Byte (CA);
                    Send_Byte (CA);
                }
                else
                {
                    Send_Byte (CRC16); /* Ask for a packet */
                }
                break;
            }
        }
    }

    return filesize;
}

/**
 * @brief  check response using the ymodem protocol
 * @param  buf: Address of the first byte
 * @retval The size of the file
 */
int32_t Ymodem_CheckResponse (uint8_t c)
{
  return 0;
}

/**
 * @brief  Prepare the first block
 * @param  timeout
 *     0: end of transmission
 * @retval None
 */
void Ymodem_PrepareIntialPacket (uint8_t *data, const uint8_t* fileName,uint32_t length)
{
    uint32_t i = 0, j = 0;
    uint8_t file_ptr[10] = { 0 };

    /* Make first three packet */
    data[PACKET_START_INDEX] = SOH;
    data[PACKET_NUMBER_INDEX] = 0x00;
    data[PACKET_CNUMBER_INDEX] = 0xff;

    /* fileName packet has valid data */
    for (i = 0; (fileName[i] != '\0') && (i < FILE_NAME_LENGTH_YM); i++)
    {
        data[i + PACKET_DATA_INDEX] = fileName[i];
    }

    data[i + PACKET_DATA_INDEX] = 0x00;

    /* file size written */
    Int2Str (file_ptr, length);
    i = i + PACKET_DATA_INDEX + 1;
    while (file_ptr[j] != '\0')
    {
      data[i++] = file_ptr[j++];
    }

  /* padding with zeros */
  for (j = i; j < PACKET_SIZE + PACKET_DATA_INDEX; j++)
    {
      data[j] = 0;
    }

}

/**
 * @brief  Prepare the data packet
 * @param  timeout
 *     0: end of transmission
 * @retval None
 */
MID_STATUS_TYPE_T Ymodem_PreparePacket (FIL *pFile, uint8_t *data, uint8_t pktNo,
		      uint32_t sizeBlk)
{
  uint32_t i, size, packetSize;
  uint32_t bytesread1 = 0;
 static uint32_t readData=0;/*已经读取的数据数量*/
 static uint32_t  readPkgNo=0;/*数据包*/
  /* Make first three packet */
  packetSize = sizeBlk >= PACKET_1K_SIZE ? PACKET_1K_SIZE : PACKET_SIZE;
  size = sizeBlk < packetSize ? sizeBlk : packetSize;
  if (packetSize == PACKET_1K_SIZE)
    {
      data[PACKET_START_INDEX] = STX;
    }
  else
    {
      data[PACKET_START_INDEX] = SOH;
    }
  data[PACKET_NUMBER_INDEX] = pktNo;
  data[PACKET_CNUMBER_INDEX] = (~pktNo);

  /*读取数据内容*/

  if (f_read (pFile, &data[PACKET_DATA_INDEX], size, (UINT*) &bytesread1)
      != FR_OK)
    {
      /* file Read or EOF Error */
      return MID_ERROR;
    }

  if (bytesread1 != size)
    {
      return MID_ERROR;
    }

  /*记录数据读取了多少*/
      if(pktNo==1)/*第一包*/
        {
  	readData=0;
  	readPkgNo=pktNo;
        }
      else if(readPkgNo!=pktNo)/*不是重复读取*/
        {
  	readPkgNo=pktNo;/*更新记录*/
  	readData+=bytesread1;/*更新已经读取的数据数量*/
        }
      else/*重复读取数据*/
	{

	}
  if (f_lseek (pFile, size+readData) != FR_OK)
    {
      /*file Read or EOF Error */
      return MID_ERROR;
    }

  if (size <= packetSize)
    {
      for (i = size + PACKET_DATA_INDEX; i < packetSize + PACKET_DATA_INDEX;
	  i++)
	{
	  data[i] = 0x1A; /* EOF (0x1A) or 0x00 */
	}
    }
  return MID_OK;
}

/**
 * @brief  Update CRC16 for input byte
 * @param  CRC input value
 * @param  input byte
 * @retval None
 */
uint16_t
UpdateCRC16 (uint16_t crcIn, uint8_t byte)
{
  uint32_t crc = crcIn;
  uint32_t in = byte | 0x100;

  do
    {
      crc <<= 1;
      in <<= 1;
      if (in & 0x100)
	++crc;
      if (crc & 0x10000)
	crc ^= 0x1021;
    }

  while (!(in & 0x10000));

  return crc & 0xffffu;
}

/**
 * @brief  Cal CRC16 for YModem Packet
 * @param  data
 * @param  length
 * @retval None
 */
uint16_t Cal_CRC16 (const uint8_t* data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t* dataEnd = data + size;

  while (data < dataEnd)
  {
    crc = UpdateCRC16 (crc, *data++);
  }

  crc = UpdateCRC16 (crc, 0);
  crc = UpdateCRC16 (crc, 0);

  return crc & 0xffffu;
}

/**
 * @brief  Cal Check sum for YModem Packet
 * @param  data
 * @param  length
 * @retval None
 */
uint8_t CalChecksum (const uint8_t* data, uint32_t size)
{
  uint32_t sum = 0;
  const uint8_t* dataEnd = data + size;

  while (data < dataEnd)
  {
    sum += *data++;
  }

  return (sum & 0xffu);
}

/**
 * @brief  Transmit a data packet using the ymodem protocol
 * @param  data
 * @param  length
 * @retval None
 */
void Ymodem_SendPacket (uint8_t *data, uint16_t length)
{
  mid_CliSendData (data, length);
}

/**
 * @brief  Transmit a file using the ymodem protocol
 * @param  buf: Address of the first byte
 * @retval The size of the file
 */
uint8_t Ymodem_Transmit (FIL *pFile, const uint8_t* p_file_name, uint32_t file_size)
{

  uint32_t errors = 0, ack_recpt = 0, size = 0, pkt_size = 0;
  MID_STATUS_TYPE_T result = MID_OK;
  uint32_t blk_number = 1;
  uint8_t a_rx_ctrl[2];
  uint32_t i = 0;

#ifdef CRC16_F
  uint32_t temp_crc;
#else /* CRC16_F */
  uint8_t temp_chksum;
#endif /* CRC16_F */

    /* Prepare first block - header */
    Ymodem_PrepareIntialPacket (aPacketData, p_file_name, file_size);

    while ((!ack_recpt) && (result == MID_OK))
    {
      /* Send Packet */
      /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F
      temp_crc = Cal_CRC16 (&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
      aPacketData[PACKET_SIZE + PACKET_HEADER_SIZE + 1] = (temp_crc >> 8)
	  & 0xFF;
      aPacketData[PACKET_SIZE + PACKET_HEADER_SIZE + 2] = (temp_crc & 0xFF);
      Ymodem_SendPacket (&aPacketData[PACKET_START_INDEX],
      PACKET_SIZE + PACKET_HEADER_SIZE + PACKET_TRAILER_SIZE);
#else /* CRC16_F */
      temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX],
	  pkt_size);
      &aPacketData[pkt_size + PACKET_HEADER_SIZE]=temp_chksum;
      Ymodem_SendPacket (&aPacketData[PACKET_START_INDEX],
	  pkt_size + PACKET_HEADER_SIZE+1);
#endif /* CRC16_F */

      /* Wait for Ack and 'C' */
      if (Receive_Byte (&a_rx_ctrl[0], NAK_TIMEOUT) == 0)
	{
	  if (a_rx_ctrl[0] == Y_ACK)
	    {
	      /* Packet transferred correctly */
	      ack_recpt = 1;
	    }
	  else if (a_rx_ctrl[0] == CA)
	    {
	      if ((Receive_Byte (&a_rx_ctrl[0], NAK_TIMEOUT) == MID_OK)
		  && (a_rx_ctrl[0] == CA))
		{
		  vTaskDelay (5);
//		  __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
		  result = MID_ERROR;
		}
	    }
	}
      else
	{
	  errors++;
	}

        if (errors >= MAX_ERRORS)
        {
            result = MID_ERROR;
        }
    }

    size = file_size;

    /* Here 1024 bytes length is used to send the packets */
    while ((size) && (result == MID_OK))
    {

      /* Prepare next packet */
      Ymodem_PreparePacket (pFile, aPacketData, blk_number, size);
      ack_recpt = 0;
      a_rx_ctrl[0] = 0;
      errors = 0;

      /* Resend packet if NAK for few times else end of communication */
      while ((!ack_recpt) && (result == MID_OK))
	{
	  /* Send next packet */
	  if (size >= PACKET_1K_SIZE)
	    {
	      pkt_size = PACKET_1K_SIZE;
	    }
	  else
	    {
	      pkt_size = PACKET_SIZE;
	    }
	  /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F
	  temp_crc = Cal_CRC16 (&aPacketData[PACKET_DATA_INDEX], pkt_size);
	  aPacketData[pkt_size + PACKET_HEADER_SIZE + 1] = (temp_crc >> 8);
	  aPacketData[pkt_size + PACKET_HEADER_SIZE + 2] = (temp_crc & 0xFF);
	  Ymodem_SendPacket (
	      &aPacketData[PACKET_START_INDEX],
	      pkt_size + PACKET_HEADER_SIZE + PACKET_TRAILER_SIZE);
#else /* CRC16_F */
	  temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX],
	      pkt_size);
	  &aPacketData[pkt_size + PACKET_HEADER_SIZE]=temp_chksum;
	  Ymodem_SendPacket (&aPacketData[PACKET_START_INDEX],
	      pkt_size + PACKET_HEADER_SIZE+1);
#endif /* CRC16_F */

	  /* Wait for Ack */
	  if ((Receive_Byte (&a_rx_ctrl[0], NAK_TIMEOUT) == MID_OK)
	      && (a_rx_ctrl[0] == Y_ACK))
	    {
	      ack_recpt = 1;
	      if (size > pkt_size)
		{
		  size -= pkt_size;
		  blk_number++;
		}
	      else
		{

		  size = 0;
		}
	    }
	  else
	    {
	      errors++;
	    }

	  /* Resend packet if NAK  for a count of 10 else end of communication */
	  if (errors >= MAX_ERRORS)
	    {
	      result = MID_ERROR;
	    }
	}
    }

  /* Sending End Of Transmission char */
  ack_recpt = 0;
  a_rx_ctrl[0] = 0x00;
  errors = 0;
  while ((!ack_recpt) && (result == MID_OK))
    {
      Send_Byte (EOT);

      /* Wait for Ack */
      if (Receive_Byte (&a_rx_ctrl[0], NAK_TIMEOUT) == MID_OK)
	{
	  if (a_rx_ctrl[0] == Y_ACK)
	    {
	      ack_recpt = 1;
	    }
	  else if (a_rx_ctrl[0] == CA)
	    {
	      if ((Receive_Byte (&a_rx_ctrl[0], NAK_TIMEOUT) == MID_OK)
		  && (a_rx_ctrl[0] == CA))
		{
		  HAL_Delay (2);
//		  __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
		  result = MID_ERROR;
		}
	    }
	}
      else
	{
	  errors++;
	}

      if (errors >= MAX_ERRORS)
	{
	  result = MID_ERROR;
	}
    }

  /* Empty packet sent - some terminal emulators need this to close session */
  if (result == MID_OK)
    {
      /* Preparing an empty packet */
      aPacketData[PACKET_START_INDEX] = SOH;
      aPacketData[PACKET_NUMBER_INDEX] = 0;
      aPacketData[PACKET_CNUMBER_INDEX] = 0xFF;
      for (i = PACKET_DATA_INDEX; i < (PACKET_SIZE + PACKET_DATA_INDEX); i++)
	{
	  aPacketData[i] = 0x00;
	}

      /* Send Packet */
      /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F
      temp_crc = Cal_CRC16 (&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
      aPacketData[PACKET_SIZE + PACKET_HEADER_SIZE + 1] = (temp_crc >> 8);
      aPacketData[PACKET_SIZE + PACKET_HEADER_SIZE + 2] = (temp_crc & 0xFF);
      Ymodem_SendPacket (&aPacketData[PACKET_START_INDEX],
      PACKET_SIZE + PACKET_HEADER_SIZE + PACKET_TRAILER_SIZE);
#else /* CRC16_F */
      temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX],
	  pkt_size);
      &aPacketData[pkt_size + PACKET_HEADER_SIZE]=temp_chksum;
      Ymodem_SendPacket (&aPacketData[PACKET_START_INDEX],
	  pkt_size + PACKET_HEADER_SIZE+1);
#endif /* CRC16_F */

      /* Wait for Ack and 'C' */
      if (Receive_Byte (&a_rx_ctrl[0], NAK_TIMEOUT) == MID_OK)
	{
	  if (a_rx_ctrl[0] == CA)
	    {
	      HAL_Delay (2);
//	      __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
	      result = MID_ERROR;
	    }
	}
    }

  return result; /* file transmitted successfully */

}

/**
 * @}
 */

