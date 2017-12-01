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
 *   drv_Sim900.c
 * DESCRIPTION:
 *   This file contains the op code of sim900 & 900A
 *    basicly AT cmd of sim900
 *    using usart1 on armfly board
 *    sim900 work in non transparent mode
 * HISTORY:
 *   2016年1月11日        Arvin         Create/Update
 *
 *****************************************************************************/
#include "drv_cdma.h"
#include "includes.h"

char ucSIMTxCmdBuffer[3200] = {'\0'};
cdma_stat_e cdma_stat;

char ucIpAddr[16] = "182.139.182.209";
char ucPort[6] = "5683";
char *pucLocalPort = "15000";

HAL_StatusTypeDef drv_SimHwInit (void);
HAL_StatusTypeDef drv_SimHwON (void);
HAL_StatusTypeDef drv_SimHwOFF (void);
HAL_StatusTypeDef drv_SimHwReset(void);
HAL_StatusTypeDef drv_SimSWInit(void);
HAL_StatusTypeDef drvSim_CheckATState(void);
HAL_StatusTypeDef drv_SimSetEcho(uint8_t tmp);
HAL_StatusTypeDef drv_SimSetUart(void);
HAL_StatusTypeDef drv_SimStopDial(void);
HAL_StatusTypeDef drv_SimQueryEsn(char * pucESN);
HAL_StatusTypeDef drvSim_CheckIPCallState(void);
HAL_StatusTypeDef drvSim_CREG(void);


HAL_StatusTypeDef drvSim_SyncATState(void)
{
    uint8_t u8Cnt = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    printf("\r\ndrvSim_SyncATState.....................\r\n");
    while((HAL_OK != drvSim_CheckATState()) && (u8Cnt < 20))
    {
        vTaskDelay(200/portTICK_PERIOD_MS);
        u8Cnt++;
    }

    if(u8Cnt >= 20)
    {
        status = HAL_ERROR;
    }
    else
    {
        status = HAL_OK;
    }

    return (status);
}

HAL_StatusTypeDef drv_SimHwInit (void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    HAL_StatusTypeDef status = HAL_ERROR;

    __GPIOF_CLK_ENABLE();
    __GPIOG_CLK_ENABLE();

    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_PWR, GPIO_PIN_2G_MODULE_PWR, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_ONOFF, GPIO_PIN_2G_MODULE_ONOFF, GPIO_PIN_SET);
    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_RST, GPIO_PIN_2G_MODULE_RST, GPIO_PIN_RESET);
    cdma_stat = CDMA_OFF;

    // SIM_PWR_2G_PIN
    GPIO_InitStruct.Pin = GPIO_PIN_2G_MODULE_PWR;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init (GPIO_PORT_2G_MODULE_PWR, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2G_MODULE_RST;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init (GPIO_PORT_2G_MODULE_RST, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2G_MODULE_ONOFF;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init (GPIO_PORT_2G_MODULE_ONOFF, &GPIO_InitStruct);

    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_PWR, GPIO_PIN_2G_MODULE_PWR, GPIO_PIN_RESET);
    vTaskDelay(200/portTICK_PERIOD_MS);
	HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_PWR, GPIO_PIN_2G_MODULE_PWR, GPIO_PIN_SET);
    drv_SimHwON_OFF();
    drv_SimHwReset();


    return (status);
}

HAL_StatusTypeDef drv_SimHwON_OFF (void)
{
    HAL_StatusTypeDef status = HAL_OK;
    GPIO_InitTypeDef GPIO_InitStruct;

    printf("drv_SimHwON_OFF\r\n");

    /* Make the ONOFF pin high level */
    GPIO_InitStruct.Pin = GPIO_PIN_2G_MODULE_ONOFF;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init (GPIO_PORT_2G_MODULE_ONOFF, &GPIO_InitStruct);
    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_ONOFF, GPIO_PIN_2G_MODULE_ONOFF, GPIO_PIN_RESET);
    vTaskDelay(4000/portTICK_PERIOD_MS);

    /* Make the ONOFF pin low level */
    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_ONOFF, GPIO_PIN_2G_MODULE_ONOFF, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_2G_MODULE_ONOFF;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init (GPIO_PORT_2G_MODULE_ONOFF, &GPIO_InitStruct);

    return (status);
}

HAL_StatusTypeDef drv_SimHwReset(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    printf("drv_SimHwReset\r\n");
    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_RST, GPIO_PIN_2G_MODULE_RST, GPIO_PIN_SET);
    vTaskDelay(100/portTICK_PERIOD_MS);

    HAL_GPIO_WritePin (GPIO_PORT_2G_MODULE_RST, GPIO_PIN_2G_MODULE_RST, GPIO_PIN_RESET);
    vTaskDelay(100/portTICK_PERIOD_MS);

    return (status);
}

HAL_StatusTypeDef drv_SimSWInit(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t u16RetryCnt = 0;

    printf("\r\ndrv_SimSWInit\r\n");

    do{
        drv_SimHwON_OFF();
        sim800c.pfInterfaceInit();
        while((drvSim_CheckATState() != HAL_OK) && (u16RetryCnt++ < 50))
        {
            vTaskDelay(100/portMAX_DELAY);
        }

        if(u16RetryCnt < 50)
        {
            status = HAL_OK;
        }
        else
        {
            u16RetryCnt = 0;
            status = HAL_ERROR;
        }
    }while(status != HAL_OK);

    drv_SimSetEcho(0);
    drv_SimSetUart();
    drv_SimSetAccoutPass();
    drv_SimStartDial();

    return (status);
}

HAL_StatusTypeDef drvSim_Init(void)
{
    HAL_StatusTypeDef status = HAL_BUSY;
    uint16_t u16RetryCnt = 0;

    switch(cdma_stat)
    {
        case CDMA_OFF:
            do{
                drv_SimHwON_OFF();
                sim800c.pfInterfaceInit();
                while((drvSim_CheckATState() != HAL_OK) && (u16RetryCnt++ < 50))
                {
                    vTaskDelay(300/portMAX_DELAY);
                }

                if(u16RetryCnt < 50)
                {
                    status = HAL_OK;
                }
                else
                {
                    u16RetryCnt = 0;
                    status = HAL_ERROR;
                }
            }while(status != HAL_OK);

            cdma_stat = CDMA_ON;
            status = HAL_BUSY;
            break;

        case CDMA_ON:
            if(HAL_OK == drv_SimSetEcho(0))
            {
                cdma_stat = CDMA_SET_ECHO;
            }
            break;

        case CDMA_SET_ECHO:
            if(HAL_OK == drv_SimSetUart())
            {
                cdma_stat = CDMA_SET_UART;
            }
            break;

        case CDMA_SET_UART:
            if(HAL_OK == drv_SimSetAccoutPass())
            {
                cdma_stat = CDMA_SET_ACCOUNT;
            }
            break;

        case CDMA_SET_ACCOUNT:
            drv_SimStartDial();
            // if(HAL_OK == drv_SimStartDial())
            {
                status = HAL_ERROR;
                cdma_stat = CDMA_BUILD_CONN;
            }
            break;

        case CDMA_BUILD_CONN:
            if(HAL_OK == drv_SimConnectServer(1, 15000, ucIpAddr, ucPort, TRUE))
            {
                cdma_stat = CDMA_WAIT_CONN;
            }
            break;

        case CDMA_WAIT_CONN:
            if(HAL_OK == drvSim_GetTcpStatus())
            {
                if(TRUE == sim800c_stat.bOnLine)
                {
                    cdma_stat = CDMA_CONNECTED;
                    status = HAL_OK;
                }
                else
                {
                    vTaskDelay(500/portTICK_PERIOD_MS);
                    status = HAL_ERROR;
                }
            }
            break;

        default:
            break;
    }

    return (status);
}


uint8_t drvSim_ServerIsConnect(void)
{
    if(cdma_stat == CDMA_CONNECTED)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

HAL_StatusTypeDef drvSim_CheckIPCallState(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char *cmd = "AT^IPCALL?\r\n";
    char ucRespArr[20] = {0};
    uint16_t u8RespLen = 0;

    sim800c.pfInterfaceTransmit((char *)cmd, strlen(cmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 500))
    {
        if(NULL != strstr((char *)ucRespArr,"^IPCALL:"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}


HAL_StatusTypeDef drvSim_CheckATState(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char *cmd = "AT\r\n";
    char ucRespArr[20] = {0};
    uint16_t u8RespLen = 0;

    printf("\r\ndrvSim_CheckATState\r\n");
    sim800c.pfInterfaceTransmit((char *)cmd, strlen(cmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}

HAL_StatusTypeDef drv_SimSetUart(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char *pucCmd = "AT+URCHAN=2\r\n";
    char ucRespArr[20] = {'\0'};
    uint16_t u8RespLen = 0;

    sim800c.pfInterfaceTransmit(pucCmd, strlen(pucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}


HAL_StatusTypeDef drv_SimSetEcho(uint8_t tmp)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[20] = {'\0'};
    char ucRespArr[20] = {'\0'};
    uint16_t u8RespLen = 0;

    if(!tmp)
    {
        sprintf(ucCmd,"%s","ATE0\r\n");
    }
    else
    {
        sprintf(ucCmd,"%s","ATE1\r\n");
    }

    sim800c.pfInterfaceTransmit(ucCmd, strlen(ucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}

HAL_StatusTypeDef drvSim_CREG(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char *pucCmd = "AT+CREG?\r\n";
    char ucRespArr[20] = {'\0'};
    uint16_t u8RespLen = 0;

    sim800c.pfInterfaceTransmit(pucCmd, strlen(pucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}


HAL_StatusTypeDef drv_SimSetAccoutPass(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[100] = {'\0'};
    char ucRespArr[100] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    u32Len += sprintf(ucCmd, "AT+VSIPACCNT=");
    ucCmd[u32Len++] = 0x22;
    u32Len += sprintf(ucCmd + u32Len, "ctnet");
    ucCmd[u32Len++] = 0x22;
    ucCmd[u32Len++] = ',';
    ucCmd[u32Len++] = 0x22;
    u32Len += sprintf(ucCmd + u32Len, "ctnet@mycdma.cn");
    ucCmd[u32Len++] = 0x22;
    ucCmd[u32Len++] = ',';
    ucCmd[u32Len++] = 0x22;
    u32Len += sprintf(ucCmd + u32Len, "vnet.mobi");
    ucCmd[u32Len++] = 0x22;
    u32Len += sprintf(ucCmd + u32Len, "\r\n");
    sim800c.pfInterfaceTransmit(ucCmd, strlen(ucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}

HAL_StatusTypeDef drv_SimStartDial(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char *pucCmd = "AT^IPCALL=1\r\n";
    char ucRespArr[20] = {'\0'};
    uint16_t u8RespLen = 0;

    memset(ucRespArr, '\0', sizeof(ucRespArr));
    sim800c.pfInterfaceTransmit(pucCmd, strlen(pucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            if(NULL != strstr((char *)ucRespArr,"^IPCALL:"))
            {
                state = HAL_OK;
            }
            else
            {
                if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen,10000))
                {
                    if(NULL != strstr((char *)ucRespArr,"^IPCALL:"))
                    {
                        state = HAL_OK;
                    }
                    else
                    {
                        state = HAL_ERROR;
                    }
                }
                else
                {
                    state = HAL_TIMEOUT;
                }
            }
        }
        else if(NULL != strstr((char *)ucRespArr,"+CME ERROR:3"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}

HAL_StatusTypeDef drv_SimStopDial(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char *pucCmd = "AT^IPCALL=0\r\n";
    char ucRespArr[20] = {'\0'};
    uint16_t u8RespLen = 0;

    sim800c.pfInterfaceTransmit(pucCmd, strlen(pucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen,10000))
            {
                state = HAL_OK;
            }
        }
    }

    return (state);
}

HAL_StatusTypeDef drv_SimConnectServer(uint8_t u8Socket, uint32_t u32LocalPort, char *pucIpAddr, char *pucPort, bool bUdpServer)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[100] = {'\0'};
    char ucRespArr[100] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    u32Len += sprintf(ucCmd, "AT^IPOPEN=%d,%d,\"%.15s\",%.5s,%d\r\n",
                    u8Socket,u32LocalPort,pucIpAddr,pucPort,((bUdpServer == TRUE) ? 1 : 0));
    sim800c.pfInterfaceTransmit(ucCmd, u32Len);
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 500))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            if(NULL == strstr((char *)ucRespArr,"^IPOPEN:1,1"))
            {
                if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen,10000))
                {
                    if(NULL != strstr((char *)ucRespArr,"^IPOPEN:1,1"))
                    {
                        state = HAL_OK;
                    }
                    else
                    {
                        state = HAL_ERROR;
                    }
                }
                else
                {
                    state = HAL_TIMEOUT;
                }
            }
            else
            {
                state = HAL_OK;
            }
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}

HAL_StatusTypeDef drv_SimQueryEsn(char * pucESN)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[100] = {'\0'};
    char ucRespArr[100] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    u32Len += sprintf(ucCmd, "AT+GSN\r\n");
    sim800c.pfInterfaceTransmit(ucCmd, u32Len);
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"+GSN:"))
        {
            printf("%s", (ucRespArr + 2));
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}

HAL_StatusTypeDef drv_SimQueryVersion(char * pucVersion)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[100] = {'\0'};
    char ucRespArr[100] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    u32Len += sprintf(ucCmd, "AT+CGMR\r\n");
    sim800c.pfInterfaceTransmit(ucCmd, u32Len);
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"+CGMR:"))
        {
            printf("%s", (ucRespArr + 2));
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}


HAL_StatusTypeDef drvSim_EntryRawMode(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[100] = {'\0'};
    char ucRespArr[100] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    u32Len += sprintf(ucCmd, "AT^IPENTRS=1");
    sim800c.pfInterfaceTransmit(ucCmd, strlen(ucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}

HAL_StatusTypeDef drvSim_ExitRawMode(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd = '+';
    char ucRespArr[100] = {'\0'};
    uint16_t u8RespLen = 0;

    printf("drvSim_ExitRawMode\r\n");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    sim800c.pfInterfaceTransmit(&ucCmd, 1);
    vTaskDelay(300/portTICK_PERIOD_MS);
    sim800c.pfInterfaceTransmit(&ucCmd, 1);
    vTaskDelay(300/portTICK_PERIOD_MS);
    sim800c.pfInterfaceTransmit(&ucCmd, 1);
    vTaskDelay(300/portTICK_PERIOD_MS);
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}


/*HAL_StatusTypeDef drvSim_LookUpDns(char *pucUrl)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[100] = {'\0'};
    char ucRespArr[100] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    u32Len += sprintf(ucCmd, "AT^DNSLOOKUP=\"%s\"\r\n",pucUrl);
    printf("\r\n%s",ucCmd);
    sim800c.pfInterfaceTransmit(ucCmd, strlen(ucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        printf("%s",ucRespArr);
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {

            state = HAL_OK;
        }
        else
        {
            state = HAL_ERROR;
        }
    }

    return (state);
}*/


HAL_StatusTypeDef drvSim_GetTcpStatus(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[30] = {'\0'};
    char ucRespArr[100] = {'\0'};
    char ucRespStateArr[5] = {'\0'};
    uint8_t u8OnLine = 0;
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    u32Len += sprintf(ucCmd, "AT+VTCPSTATUS?\r\n");
    sim800c.pfInterfaceTransmit(ucCmd, strlen(ucCmd));
	//printf();
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 1000))
    {
    	printf("GetTcpStatus, %s\r\n",ucRespArr);
        if(NULL != strstr((char *)(ucRespArr + 2),"+VTCPSTATUS:1,"))
        {
            sscanf((char *)(ucRespArr + 2), "%*[^,],%s",ucRespStateArr);
            u8OnLine = atoi(ucRespStateArr);
            if(!u8OnLine)
            {
                sim800c_stat.bOnLine = TRUE;
            }
            else
            {
                sim800c_stat.bOnLine = FALSE;
            }

            state = HAL_OK;
        }
    }

    return (state);
}

HAL_StatusTypeDef drvSim_SendTcpDataToServer(uint8_t * pucBuffer, int16_t u16Size)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucRespArr[30] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;
    uint16_t u16TxedData = 0;
    char ucRespLenArr[5] = {'\0'};

    memset(ucSIMTxCmdBuffer, 0,sizeof(ucSIMTxCmdBuffer));
    u32Len += sprintf(ucSIMTxCmdBuffer, "AT^IPSEND=1,\"");
    M2Mhtos((ucSIMTxCmdBuffer + u32Len), (char *)pucBuffer, u16Size);
    u32Len += u16Size * 2;
    u32Len += sprintf(ucSIMTxCmdBuffer + u32Len, "\"\r\n");
    sim800c.pfInterfaceTransmit(ucSIMTxCmdBuffer, strlen(ucSIMTxCmdBuffer));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 10000))
    {
        if(u8RespLen > 6)
        {
            if(NULL != strstr((char *)ucRespArr,"OK"))
            {
                if(NULL != strstr((char *)ucRespArr,"^IPSEND:1,"))
                {
                    sscanf(ucRespArr, "%*[^,],%s", ucRespLenArr);
                    u16TxedData = atoi(ucRespLenArr);
                    if(u16TxedData == u16Size)
                    {
                        state = HAL_OK;
                    }
                }
            }
        }
        else
        {
            if(NULL != strstr((char *)ucRespArr,"OK"))
            {
                if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen,10000))
                {
                    if(NULL != strstr((char *)ucRespArr,"^IPSEND:1,"))
                    {
                        sscanf(ucRespArr, "%*[^,],%s", ucRespLenArr);
                        u16TxedData = atoi(ucRespLenArr);
                        if(u16TxedData == u16Size)
                        {
                            state = HAL_OK;
                        }
                    }
                }
            }
        }
    }

    return (state);
}

HAL_StatusTypeDef drvSim_SendTcpDataToServer_Ext(uint8_t * pucBuffer, int16_t u16Size)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    uint32_t u32Len = 0;

    memset(ucSIMTxCmdBuffer, 0,sizeof(ucSIMTxCmdBuffer));
    u32Len += sprintf(ucSIMTxCmdBuffer, "AT^IPSEND=1,");
    ucSIMTxCmdBuffer[u32Len++] = 0x22;
    M2Mhtos((ucSIMTxCmdBuffer + u32Len), (char *)pucBuffer, u16Size);
    u32Len += u16Size * 2;
    ucSIMTxCmdBuffer[u32Len++] = 0x22;
    u32Len += sprintf(ucSIMTxCmdBuffer + u32Len, "\r\n");
    if(HAL_OK == sim800c.pfInterfaceTransmit(ucSIMTxCmdBuffer, strlen(ucSIMTxCmdBuffer)))
    {
        state = HAL_OK;
    }

    return (state);
}

HAL_StatusTypeDef drvSim_SendRawDataToServer(uint8_t * pucBuffer, int16_t u16Size)
{
    HAL_StatusTypeDef state = HAL_ERROR;

    sim800c.pfInterfaceTransmit((char *)pucBuffer, u16Size);

    return (state);
}


HAL_StatusTypeDef drvSim_GetTcpDataFromServer(uint8_t * pucBuffer, uint16_t *pu16Size)
{
    HAL_StatusTypeDef state = HAL_ERROR;

    state = sim800c.pfInterfaceReceive((char *)pucBuffer, pu16Size, 10000);

    return (state);
}

HAL_StatusTypeDef drvSim_SendUdpDataToServer(uint8_t * pucBuffer, int16_t u16Size, char * pucServerIpAddr, char *pucPort)
{
    HAL_StatusTypeDef state = HAL_OK;
    char ucRespArr[30] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;
    uint16_t u16TxedData = 0;
    char ucRespLenArr[5] = {'\0'};

    memset(ucSIMTxCmdBuffer, 0,sizeof(ucSIMTxCmdBuffer));
    u32Len += sprintf(ucSIMTxCmdBuffer, "AT^IPSENDUDP=1,\"");
    M2Mhtos((ucSIMTxCmdBuffer + u32Len), (char *)pucBuffer, u16Size);
    u32Len += u16Size * 2;
    u32Len += sprintf(ucSIMTxCmdBuffer + u32Len, "\",\"%.15s\",%s\r\n",pucServerIpAddr,pucPort);
    // printf("%s",ucSIMTxCmdBuffer);
    sim800c.pfInterfaceTransmit(ucSIMTxCmdBuffer, u32Len);
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 30000))
    {
        if(u8RespLen > 6)
        {
            if(NULL != strstr((char *)ucRespArr,"OK"))
            {
                if(NULL != strstr((char *)ucRespArr,"^IPSEND:1,"))
                {
                    sscanf(ucRespArr, "%*[^,],%s", ucRespLenArr);
                    u16TxedData = atoi(ucRespLenArr);
                    if(u16TxedData == u16Size)
                    {
                        state = HAL_OK;
                    }
                }
            }
        }
        else
        {
            if(NULL != strstr((char *)ucRespArr,"OK"))
            {
                if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen,10000))
                {
                    if(NULL != strstr((char *)ucRespArr,"^IPSEND:1,"))
                    {
                        sscanf(ucRespArr, "%*[^,],%s", ucRespLenArr);
                        u16TxedData = atoi(ucRespLenArr);
                        if(u16TxedData == u16Size)
                        {
                            state = HAL_OK;
                        }
                    }
                }
            }
        }
    }

    return (state);
}

HAL_StatusTypeDef drvSim_CloseIPConnect(void)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    char ucCmd[100] = {'\0'};
    char ucRespArr[100] = {'\0'};
    uint32_t u32Len = 0;
    uint16_t u8RespLen = 0;

    printf("\r\ndrvSim_CloseIPConnect\r\n");
    u32Len += sprintf(ucCmd, "AT^IPCLOSE=1\r\n");
    printf("Txed: %s",ucCmd);

    sim800c.pfInterfaceTransmit(ucCmd, strlen(ucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u8RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            printf("ATState,OK\r\n");
            state = HAL_OK;
        }
        else
        {
            printf("ATState,ERROR\r\n");
        }
    }

    return (state);
}

/*    *
 simple function to transfer rssi 2 signal DB number
 return: -115~-52
 */
int8_t drv_SimRssi2Db (uint8_t rssi)
{
    int8_t db = 0;

    if (rssi == 0)
        db = -115;
    else if (rssi == 1)
        db = -111;
    else if (rssi >= 2 && rssi <= 30)
        db = 2 * rssi - 114;
    else if (rssi >= 31 && rssi < 99)
        db = -52;
    else
        db = 0;

    return (db);
}

HAL_StatusTypeDef drvSim_GetSigStrength (uint8_t *pu8Rssi, int8_t *pi8Rssi)
{
    HAL_StatusTypeDef state = HAL_ERROR;
    uint8_t SQ = 0;
    int8_t i8SQdb = 0;
    char *ucCmd = "AT+CSQ\r\n";
    char ucRespArr[50] = {'\0'};
    uint16_t u16RespLen = 0;
    char pucArr[5] = {'\0'};

    /*send SQ cmd*/
    sim800c.pfInterfaceTransmit(ucCmd, strlen(ucCmd));
    if(HAL_OK == sim800c.pfInterfaceReceive(ucRespArr, &u16RespLen, 200))
    {
        if(NULL != strstr((char *)ucRespArr,"OK"))
        {
            if (strncmp((char*) (ucRespArr + 2), "+CSQ:", 5) == 0)
            {
                sscanf((char*) (ucRespArr + 2),"%*[^:]:%[^,]s",pucArr);
                SQ = atoi(pucArr);
                i8SQdb = drv_SimRssi2Db (SQ);
                if(pi8Rssi != NULL)
                {
                    *pi8Rssi = i8SQdb;
                }

                if(pu8Rssi != NULL)
                {
                    *pu8Rssi = SQ;
                }
                printf("i8SQdb = %d dBm\r\n",i8SQdb);
            }

            state = HAL_OK;
        }
        else
        {
            printf("ATState,ERROR\r\n");
        }
    }

    return state;
}


