#include "mid_net.h"
#include "includes.h"

char ucBandMonitorTemplateArr[BOX_CONNECT_MAX_NUM][MAX_MONITOR_TEMPLATE_LEN];
uint8_t u8ArrTmp[MAX_LEN_PER_PKG + 2] = {0};
FIL fil;

MID_STATUS_TYPE_T mid_TransmitFileToServer(char *pucFileName)
{
    MID_STATUS_TYPE_T status = MID_OK;
    MID_STATUS_TYPE_T status_tmp = MID_OK;
    uint32_t u32TotelLen = 0;
    uint16_t u16ToReadLen = 0;
    uint32_t u32ReadLen = 0;
    int8_t i8Status = 0;
    uint32_t u32TxedPkgCnt = 0;
	uint8_t u8Offset = 0;

    printf("Now,transmitting the file %s....\r\n",pucFileName);
    if(FR_OK == f_open(&fil, pucFileName, FA_READ | FA_OPEN_EXISTING))
    {
        u32TotelLen = fil.fsize;
		u8Offset = sprintf((char *)u8ArrTmp, "syncData=");
        do{
            u16ToReadLen = ((u32TotelLen > MAX_LEN_PER_PKG) ? MAX_LEN_PER_PKG : u32TotelLen);
            if(FR_OK == f_read(&fil, u8ArrTmp + u8Offset, u16ToReadLen, &u32ReadLen))
            {
                if(u32TotelLen > MAX_LEN_PER_PKG)
                {
                    status_tmp = midCoAP_TransmitDataToServer(u8ArrTmp, u16ToReadLen, u32TxedPkgCnt++, TRUE);
                }
                else
                {
                    status_tmp = midCoAP_TransmitDataToServer(u8ArrTmp, u16ToReadLen, u32TxedPkgCnt++, FALSE);
                }

                if(HAL_OK != status_tmp)
                {
                    i8Status = -1;
                }
                else
                {
                    u32TotelLen = ((u32TotelLen > u16ToReadLen) ? (u32TotelLen - u16ToReadLen) : 0);
                }
            }
        }while((u32TotelLen >0) && (i8Status == 0));

        if(u32TotelLen > 0)
        {
            status = MID_ERROR;
        }
        else
        {
            status = MID_OK;
        }

        f_close(&fil);
    }
    else
    {
        status = MID_PARA_ERROR;
    }

    return (status);
}


MID_STATUS_TYPE_T mid_TransmitBandSycDataToServer(char *pucFileName, uint32_t u32BandSn)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
	FRESULT fil_res = FR_OK;
	int8_t i8Status = 0;
	uint32_t u32TotelLen = 0;
	uint32_t u32TotelLenTmp = 0;
	uint16_t u16ToReadLen = 0;
    char ucUriTmp[80];
	char cBoxSnTmp[BOX_SN_MAX_LEN];
	uint32_t u32Crc_tmp = 0;
	uint16_t u16Crc_tmp = 0;
	uint32_t u32TxedPkgCnt = 0;
	uint32_t u32ReadOutLen = 0;
	uint8_t u8CrcArr[2];

	printf("Now,transmitting the file %s....\r\n",pucFileName);
    if(FR_OK == f_open(&fil, pucFileName, FA_READ | FA_OPEN_EXISTING))
    {
        // u32TotelLen = fil.fsize;
		u32TotelLen = 50 * 1024;
		u32TotelLenTmp = u32TotelLen;
		memset(ucUriTmp, '\0', sizeof(ucUriTmp));
		memcpy(cBoxSnTmp, box_info.ucBoxSN, BOX_SN_MAX_LEN);
		sprintf((char *)ucUriTmp, "assistants_bands_syncData?assistantSn=%s&bandSn=%d",cBoxSnTmp,u32BandSn);
		do{
			printf("u32TotelLen = %d\r\n",u32TotelLen);
			u16ToReadLen = ((u32TotelLen > MAX_LEN_PER_PKG) ? MAX_LEN_PER_PKG : u32TotelLen);
			fil_res = f_read(&fil, u8ArrTmp, u16ToReadLen, &u32ReadOutLen);
			if(FR_OK == fil_res)
			{
				u32Crc_tmp = crc32(u32Crc_tmp,u8ArrTmp, u16ToReadLen);
				if(u32TotelLen > MAX_LEN_PER_PKG)
				{
					status = midCoAP_TransmitMonitorDataToServer(ucUriTmp,u8ArrTmp, MAX_LEN_PER_PKG, u32TxedPkgCnt++, TRUE, u32TotelLenTmp);
					if(HAL_OK != status)
					{
						i8Status = -1;
					}
					else
					{
						u32TotelLen = ((u32TotelLen > u16ToReadLen) ? (u32TotelLen - u16ToReadLen) : 0);
					}
				}
				else					// The last packsge
				{
					u16Crc_tmp = crc32_To_crc16(u32Crc_tmp);
					printf("u16Crc_tmp = 0x%04x\r\n",u16Crc_tmp);
					u8CrcArr[0] = u16Crc_tmp >> 8;
					u8CrcArr[1] = (uint8_t)u16Crc_tmp;
					memcpy(u8ArrTmp + u16ToReadLen, u8CrcArr, 2);
					if((u32TotelLen + 2) <= MAX_LEN_PER_PKG)
					{
						status = midCoAP_TransmitMonitorDataToServer(ucUriTmp,u8ArrTmp, u16ToReadLen + 2, u32TxedPkgCnt++, FALSE, u32TotelLenTmp);
						if(HAL_OK != status)
						{
							i8Status = -1;
						}
						else
						{
							u32TotelLen = ((u32TotelLen > u16ToReadLen) ? (u32TotelLen - u16ToReadLen) : 0);
						}
					}
					else
					{
						if(1 == ((u32TotelLen + 2) - MAX_LEN_PER_PKG))
						{
							status = midCoAP_TransmitMonitorDataToServer(ucUriTmp,u8ArrTmp, MAX_LEN_PER_PKG, u32TxedPkgCnt++, TRUE, u32TotelLenTmp);
							if(HAL_OK != status)
							{
								i8Status = -1;
							}
							else
							{
								status = midCoAP_TransmitMonitorDataToServer(ucUriTmp,(u8ArrTmp + MAX_LEN_PER_PKG), 1, u32TxedPkgCnt++, FALSE, u32TotelLenTmp);
								if(HAL_OK == status)
								{
									u32TotelLen = 0;
								}
								else
								{
									i8Status = -1;
								}
							}
						}
						else
						{
							status = midCoAP_TransmitMonitorDataToServer(ucUriTmp,u8ArrTmp, MAX_LEN_PER_PKG, u32TxedPkgCnt++, TRUE, u32TotelLenTmp);
							if(HAL_OK != status)
							{
								i8Status = -1;
							}
							else
							{
								status = midCoAP_TransmitMonitorDataToServer(ucUriTmp,(u8ArrTmp + MAX_LEN_PER_PKG), 2, u32TxedPkgCnt++, FALSE, u32TotelLenTmp);
								if(HAL_OK == status)
								{
									u32TotelLen = 0;
								}
								else
								{
									i8Status = -1;
								}
							}
						}
					}
				}
			}
			else
			{
				i8Status = -1;
			}
		}while((u32TotelLen >0) && (i8Status == 0));

        if(u32TotelLen > 0)
        {
            status = MID_ERROR;
        }
        else
        {
			status = MID_OK;
        }

        f_close(&fil);
    }
    else
    {
        status = MID_PARA_ERROR;
    }

    return (status);
}

MID_STATUS_TYPE_T mid_GetBondingBandInfo(void)
{
	MID_STATUS_TYPE_T status = MID_ERROR;
	uint8_t u8BandNum = 0;
	uint32_t u32BandSnArr[BOX_CONNECT_MAX_NUM];

	memset(u32BandSnArr, 0, BOX_CONNECT_MAX_NUM * sizeof(uint32_t));
	if(MID_OK == midCoAP_GetParedBandInfo(&u8BandNum))
	{
		if((u8BandNum > 0) && (u8BandNum <= box_info.u8SupportBandMaxNum))
		{
            box_info.u8BandNum = u8BandNum;
		}

	    status = MID_OK;
	}

	return (status);
}

MID_STATUS_TYPE_T mid_GetMonitorTemplate(void)
{
	MID_STATUS_TYPE_T status = MID_ERROR;
    uint8_t i = 0;

	for(i = 0;i < box_info.u8SupportBandMaxNum;i ++)
	{
		memset((char *)(ucBandMonitorTemplateArr + i), '\0', sizeof(MAX_MONITOR_TEMPLATE_LEN));
		if((*(box_info.ptdev_band_info + i))->sn)
		{
			if(MID_OK == midCoAP_GetMonitorTemplate((char *)(ucBandMonitorTemplateArr + i), (*(box_info.ptdev_band_info + i)) ->sn))
			{
				(*(box_info.ptdev_band_info + i)) ->monitorTemplate.pbuf = (uint8_t *)ucBandMonitorTemplateArr + i;
				(*(box_info.ptdev_band_info + i)) ->monitorTemplate.len = strlen((char *)(ucBandMonitorTemplateArr + i));
			}
            else
            {
                return MID_ERROR;
            }
		}
        else
        {
            break;
        }
	}

    status = MID_OK;

    return (status);
}

MID_STATUS_TYPE_T mid_SyncFileToServer(void)
{
	MID_STATUS_TYPE_T status = MID_ERROR;
    uint8_t i = 0;

	for(i = 0;i < box_info.u8SupportBandMaxNum;i ++)
	{
		memset((char *)(ucBandMonitorTemplateArr + i), '\0', sizeof(MAX_MONITOR_TEMPLATE_LEN));
		if((*(box_info.ptdev_band_info + i))->sn)
		{
			if(MID_OK == midCoAP_GetMonitorTemplate((char *)(ucBandMonitorTemplateArr + i), (*(box_info.ptdev_band_info + i)) ->sn))
			{
				(*(box_info.ptdev_band_info + i)) ->monitorTemplate.pbuf = (uint8_t *)ucBandMonitorTemplateArr + i;
				(*(box_info.ptdev_band_info + i)) ->monitorTemplate.len = strlen((char *)(ucBandMonitorTemplateArr + i));
			}
            else
            {
                return MID_ERROR;
            }
		}
        else
        {
            break;
        }
	}

    status = MID_OK;

    return (status);
}


MID_STATUS_TYPE_T mid_ServerInit(void)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
	uint8_t u8ServerState = 0;
	uint32_t u32Time;

    switch(box_info.server_setup)
    {
        case SERVER_SETUP_INIT:
			if(MID_OK == midCoAP_GetServerAvailable(&u8ServerState))
			{
				if(u8ServerState == 1)
				{
					box_info.server_setup = SERVER_SETUP_AVAILABLE;
				}
			}
            break;

		case SERVER_SETUP_AVAILABLE:
			if(MID_OK == midCoAP_PutAssistantInfo())
			{
				box_info.server_setup = SERVER_SETUP_BASEINFO;
			}
            break;

		case SERVER_SETUP_BASEINFO:
			if(MID_OK == midCoAP_GetRealServerIP(box_info.ucIpAddr, box_info.ucPort))
			{
				box_info.server_setup = SERVER_SETUP_CONFIGIP;
			}
            break;

		case SERVER_SETUP_CONFIGIP:
			if(MID_OK == midCoAP_GetSysTime(&u32Time))
			{
				time_t unix_time = (time_t)u32Time;

                RTC_validate(unix_time);
                RTC_Period_Wakeup_Init(32768/2, App_RTC_Wakup_Callback);
				box_info.server_setup = SERVER_SETUP_TIME;
			}
            break;

		case SERVER_SETUP_TIME:
			if(MID_OK == mid_GetBondingBandInfo())
			{
				box_info.server_setup = SERVER_SETUP_MONITORTEMP;
			}
            break;

		case SERVER_SETUP_MONITORTEMP:
			if(MID_OK == mid_GetMonitorTemplate())
			{
				box_info.server_setup = SERVER_SETUP;
			}
            break;

		case SERVER_SETUP:
			status = MID_OK;
            break;

        default:
            break;
    }

    return (status);
}

