/*
 * drv_Qspi.c
 *
 *  Created on: 2015年7月30日
 *      Author: Arvin
 * @version V1.0.0
 * @date    25-June-2015
 * @brief   This file includes a standard driver for the N25Q128A QSPI
 *          memory mounted on STM32746G-Discovery board.
 @verbatim
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 [..]
 (#) This driver is used to drive the N25Q128A QSPI external


 (#) This driver need a specific component driver (N25Q128A) to be included with.

 (#) Initialization steps:
 (++) Initialize the QPSI external memory using the drv_QspiInit() function. This
 function includes the MSP layer hardware resources initialization and the
 QSPI interface with the external memory.

 (#) QSPI memory operations
 (++) QSPI memory can be accessed with read/write operations once it is
 initialized.
 Read/write operation can be performed with AHB access using the functions
 drv_QspiRead()/drv_QspiWrite().
 (++) The function drv_QspiGetInfo() returns the configuration of the QSPI memory.
 (see the QSPI memory data sheet)
 (++) Perform erase block operation using the function drv_QspiErase_Block() and by
 specifying the block address. You can perform an erase operation of the whole
 chip by calling the function drv_QspiErase_Chip().
 (++) The function drv_QspiGetStatus() returns the current status of the QSPI memory.
 (see the QSPI memory data sheet)

 */

/* Includes ------------------------------------------------------------------*/
#include "drv_FlashSpi.h"
#include "includes.h"

static SPI_HandleTypeDef hFlashSpi;

#define EN_FLASH_SPI_CMU()      	 HAL_GPIO_WritePin(SPI_ROM_NSS_PORT,SPI_ROM_NSS_PIN,GPIO_PIN_RESET)
#define DISABLE_FLASH_SPI_CMU()      HAL_GPIO_WritePin(SPI_ROM_NSS_PORT,SPI_ROM_NSS_PIN,GPIO_PIN_SET)
#define SPI_CMU_TIME_OUT      		    500 			//ms
#define W25Q128_ID					    0x00EF4018
#define MX25L12835F_ID				    0x00c22018

HAL_StatusTypeDef drv_FlashEraseSector (uint32_t add);
static HAL_StatusTypeDef drv_FlashWriteEn (void);

static SemaphoreHandle_t xCmuFlashMutex = NULL; //lock for sim900 OP

/*获取NORFLASH控制权*/
HAL_StatusTypeDef drv_FlashGetAccess (void)
{
    if(pdTRUE == xSemaphoreTake(xCmuFlashMutex, portMAX_DELAY))
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/*释放NORFLASH控制权*/
HAL_StatusTypeDef drv_FlashReleaseAccess (void)
{
    if(pdTRUE == xSemaphoreGive(xCmuFlashMutex))
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
 * @brief  Initializes the QSPI interface.
 * @retval QSPI memory status
 */

HAL_StatusTypeDef drv_FlashSpiInit (void)
{
    uint32_t id=0;
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_COMInitTypeDef spiFlashInit;

    xCmuFlashMutex= xSemaphoreCreateMutex();
    if( xCmuFlashMutex==NULL)
    {
        return HAL_ERROR;
    }
    GPIO_InitStruct.Pin = SPI_ROM_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init (SPI_ROM_NSS_PORT, &GPIO_InitStruct);

    spiFlashInit.CLKPhase=SPI_PHASE_1EDGE;
    spiFlashInit.CLKPolarity=SPI_POLARITY_LOW;
    spiFlashInit.DataSize=SPI_DATASIZE_8BIT;
    spiFlashInit.Mode=SPI_MODE_MASTER;
    Spi3Drv.Init(&spiFlashInit);

    id = drv_FlashGetId();//

    if((id == MX25L12835F_ID) 
    || (id == W25Q128_ID))
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
    
    
}

/**
 * @brief  ENABLE program flash.
 * @param  None
 * @retval None
 */
static HAL_StatusTypeDef drv_FlashWriteEn(void)
{
    uint8_t addBuf = WRITE_ENABLE_CMD;
    HAL_StatusTypeDef ret = HAL_OK;

    EN_FLASH_SPI_CMU();

    if(Spi3Drv.Write (&addBuf, 1) != HAL_OK)
    {
        ret = HAL_ERROR;
    }
    else
    {
        ret = HAL_OK;
    }

    DISABLE_FLASH_SPI_CMU();

    return (ret);
}
/**
 * @brief  erase a 4kb sector
 * @param  add: pointer to the first address of sector .
 * @param  NumByteToWrite: Number of bytes to write.
 * @retval None
 */
HAL_StatusTypeDef CMD_SE( uint32_t flash_address )
{
    uint8_t addBuf[3] = {0};

	#ifdef FLASH_DEBUG
		printf("\r\nCMD_SE,flash_address = 0x%08x\r\n",flash_address);
	#endif

    // Check flash address
    if( flash_address > N25Q128A_FLASH_SIZE )
		return HAL_ERROR;

    // Check flash is busy or not
    if( drv_FlashIsBusy() )
		return HAL_ERROR;
    
    // Setting Write Enable Latch bit
    if (drv_FlashWriteEn () != HAL_OK) /* 发送写使能命令 */
    {
        return HAL_ERROR;
    }
    
    EN_FLASH_SPI_CMU(); /* 使能片选 */
    
    addBuf[0] = SUBSECTOR_ERASE_CMD;/* 发送PAGE prog命令(地址自动增加编程) */
    if (Spi3Drv.Write (&addBuf[0], 1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    addBuf[0] = (flash_address & 0xFF0000) >> 16;
    addBuf[1] = (flash_address & 0xFF00) >> 8;
    addBuf[2] = flash_address & 0xFF;
    if (Spi3Drv.Write (addBuf, 3) != HAL_OK)/* 发送扇区地址的高8bit */
    {
        return HAL_ERROR;
    }

    DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */

    do{
        vTaskDelay (100);/*等待100ms 确定器件的4kb 擦出完成*/
    }while (drv_FlashIsBusy ());
  
    return HAL_OK; 
}



HAL_StatusTypeDef drv_FlashEraseSector (uint32_t add)
{
//    HAL_StatusTypeDef msg = HAL_ERROR;
    return CMD_SE(add);
}

/*
 * W25Q128 id=0xEF17
 *
 * */
void InsertDummyCycle( uint8_t dummy_num)
{
	uint16_t i = 0;

	for(i = 0;i < dummy_num;i ++)
	{
		Spi3Drv.WriteRead (0xFF);
	}
}

uint8_t CMD_ReadID( uint32_t *Identification )
{
    uint32_t temp;
    uint8_t  gDataBuffer[3] = {0};
	uint8_t i;
    
	// Chip select go low to start a flash command
    EN_FLASH_SPI_CMU();
    
    Spi3Drv.WriteRead (0x9F);
    // Send command
	//MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_RDID, &rx_temp);
    
    InsertDummyCycle(3);

    // Get manufacturer identification, device identification
	for(i = 0;i < 3;i ++)
	{
		gDataBuffer[i] = Spi3Drv.WriteRead(0);
	}

	// Chip select go high to end a command
    DISABLE_FLASH_SPI_CMU();;

    // Store identification
    temp =  gDataBuffer[0];
    temp =  (temp << 8) | gDataBuffer[1];
    *Identification =  (temp << 8) | gDataBuffer[2];

    return HAL_OK;
}

uint32_t drv_FlashGetId (void)
{
    uint32_t flashId = 0;

    if(HAL_OK != CMD_ReadID(&flashId))
    {
        flashId = 0;
    }

    return flashId;
}

/**
 * @brief  Check the flash is busy.
 * @param  n/a
 * @retval   flash busy? TRUE:FALSE
 */
bool drv_FlashIsBusy(void)
{
    EN_FLASH_SPI_CMU(); /* 使能片选 */
    Spi3Drv.WriteRead (READ_STATUS_REG_CMD);
    if (Spi3Drv.WriteRead (DUMMY_BYTE) & W25Q128F_STATUS1_BUSY)
    {
        DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */
        
        return TRUE;
    }
    DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */
    
    return FALSE;
}

uint8_t CMD_RDSR0( uint8_t *StatusReg )
{
    uint8_t gDataBuffer;

    EN_FLASH_SPI_CMU();							// Chip select go low to start a flash command

    // Send command
	Spi3Drv.WriteRead(READ_STATUS_REG_CMD);
	gDataBuffer =Spi3Drv.WriteRead(0);

    DISABLE_FLASH_SPI_CMU();							// Chip select go high to end a flash command

    *StatusReg = gDataBuffer;

    return TRUE;
}

bool IsFlashBusy0( void )
{
    uint8_t  gDataBuffer;

    CMD_RDSR0( &gDataBuffer );
    if( (gDataBuffer & W25Q128F_STATUS1_BUSY)  == W25Q128F_STATUS1_BUSY )
    {
    	#ifdef FLASH_DEBUG
			//printf("[FLASH] Busy,0x%02x...\r\n",gDataBuffer);
		#endif
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
/**
 * @brief  drv_FlashWritePage 按整页（256 bytes）的方式写入数据,需要调用前，自行擦出写入目前的数据
 * @param  pBuffer: pointer to the buffer containing the data to be written to the Flash.
 * @param  WriteAddr: Flash's internal address to write to.
 * @param  length: Number of bytes to write.,
 * @retval None
 */
HAL_StatusTypeDef drv_FlashWritePageForNoFafts (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length)
{
    uint32_t j = 0;
    uint8_t addBuf[3] = { 0 };

    while (drv_FlashIsBusy ())
    {
        vTaskDelay (1);/*等待1ms 确定器件的4kb 擦出完成*/
    }

    for (j = 0; j < length / 256; j++)
	{
        if (drv_FlashWriteEn () != HAL_OK) /* 发送写使能命令 */
	    {
            return HAL_ERROR;
	    }

        EN_FLASH_SPI_CMU(); /* 使能片选 */
        addBuf[0] = PAGE_PROG_CMD;/* 发送PAGE prog命令(地址自动增加编程) */
        if (Spi3Drv.Write  (&addBuf[0], 1) != HAL_OK)
        {
            return HAL_ERROR;
        }
        
        addBuf[0] = (writeAddr & 0xFF0000) >> 16;
        addBuf[1] = (writeAddr & 0xFF00) >> 8;
        addBuf[2] = writeAddr & 0xFF;
        if (Spi3Drv.Write  (addBuf, 3) != HAL_OK)/* 发送扇区地址的高8bit */
        {
            return HAL_ERROR;
        }

        if (Spi3Drv.Write  ((pBuffer + j * 256), 256) != HAL_OK)/* 发送数据 */
	    {
            return HAL_ERROR;
	    }

        DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */

        while (drv_FlashIsBusy ())
	    {
            vTaskDelay (2);/*等待1ms 确认写操作完成*/
	    }

        writeAddr += 256;
	}

    /* 进入写保护状态 */
    EN_FLASH_SPI_CMU();

    addBuf[0] = WRITE_DISABLE_CMD;
    Spi3Drv.Write  (&addBuf[0], 1);
    DISABLE_FLASH_SPI_CMU();

    return HAL_OK;
}
/**
 * @brief  drv_FlashWritePage 按整页（256 bytes）的方式写入数据,擦出按照扇区的方式.此方式适用于文件系统，
 * 写入长度必须是扇区大小的倍数。
 * @param  pBuffer: pointer to the buffer containing the data to be written to the Flash.
 * @param  WriteAddr: Flash's internal address to write to.
 * @param  length: Number of bytes to write.,
 * @retval None
 */
HAL_StatusTypeDef CMD_PageProgram( uint32_t flash_address, uint8_t *source_address, uint32_t byte_length )
{
    uint8_t addBuf[3] = { 0 };

    /* Check parameters */
    if( flash_address > N25Q128A_FLASH_SIZE )
		return HAL_ERROR;
    if( source_address == NULL)
		return HAL_ERROR;
    if( byte_length> N25Q128A_PAGE_SIZE)
		return HAL_ERROR; 

	#ifdef FLASH_DEBUG
		printf("CMD_PageProgram,flash_address = 0x%08x,byte_length = 0x%08x\r\n",flash_address,byte_length);
	#endif
    while (drv_FlashIsBusy ())
    {
        vTaskDelay (1);/*等待1ms 确定器件的4kb 擦出完成*/
    }
    // Setting Write Enable Latch bit
    if (drv_FlashWriteEn () != HAL_OK) /* 发送写使能命令 */
    {
        return HAL_ERROR;
    }

    // Chip select go low to start a flash command
    EN_FLASH_SPI_CMU();
    addBuf[0] = PAGE_PROG_CMD;/* 发送PAGE prog命令(地址自动增加编程) */
    if (Spi3Drv.Write  (&addBuf[0], 1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    addBuf[0] = (flash_address & 0xFF0000) >> 16;
    addBuf[1] = (flash_address & 0xFF00) >> 8;
    addBuf[2] = flash_address & 0xFF;
    if (Spi3Drv.Write  (addBuf, 3) != HAL_OK)/* 发送扇区地址的高8bit */
    {
        return HAL_ERROR;
    }

    if (Spi3Drv.Write  (source_address, 256) != HAL_OK)/* 发送数据 */
    {
        return HAL_ERROR;
    }

    DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */

    while (drv_FlashIsBusy ())
    {
        vTaskDelay (2 / portTICK_PERIOD_MS);/*等待1ms 确认写操作完成*/
    }

    return HAL_OK;
}

HAL_StatusTypeDef drv_FlashWritePage (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length)
{
    uint32_t i = 0, j = 0;

    if ((length % N25Q128A_SUBSECTOR_SIZE) != 0) /* 整个扇区都改写 */
    {
        return HAL_ERROR;/*此驱动用于FATFS文件系统，定义扇区大小为4096*/
    }
    
    while (drv_FlashIsBusy ())
    {
        vTaskDelay (1);/*等待1ms 确定器件的4kb 擦出完成*/
    }

    for (i = 0; i < (length >>12) ; i++)/*有几个扇区*/
    {
        if (drv_FlashEraseSector(writeAddr + i * 4096) != HAL_OK) /* 擦出扇区 */
        {
            return HAL_ERROR;
        }
    }

    for (j = 0; j < length / 256; j++)
    {
        if(CMD_PageProgram(writeAddr,pBuffer,N25Q128A_PAGE_SIZE)!=HAL_OK)
        {
            return HAL_ERROR;
        }

        pBuffer += N25Q128A_PAGE_SIZE;
        writeAddr += N25Q128A_PAGE_SIZE;
    }

    return HAL_OK;
}

/**
 * @brief  Reads a block of data from the Flash.
 * @param  pBuffer: pointer to the buffer that receives the data read from the Flash.
 * @param  readAddr: Flash internal address to read from.
 * @param  length: number of bytes to read from the Flash.
 * @retval None
 */
 HAL_StatusTypeDef CMD_READ( uint32_t flash_address, uint8_t *target_address, uint32_t byte_length )
{
    uint8_t addBuf[3] = { 0 };

	#ifdef FLASH_DEBUG
		printf("CMD_READ,flash_address = 0x%08x,byte_length = 0x%08x (%d)\r\n",flash_address,byte_length,byte_length);
	#endif

    // Check flash address
    if( flash_address > N25Q128A_FLASH_SIZE ) return HAL_ERROR;
    
    if(byte_length >= N25Q128A_PAGE_SIZE)
    {
        byte_length = N25Q128A_PAGE_SIZE;
    }

    while (drv_FlashIsBusy ())
    {
        vTaskDelay (1);/*等待1ms 确定器件的4kb 擦出完成*/
    }
     

    // Chip select go low to start a flash command
    EN_FLASH_SPI_CMU();
    
    addBuf[0] = READ_CMD;
    if (Spi3Drv.Write(&addBuf[0],1) != HAL_OK) /* 发送读命令 */
    {
        return HAL_ERROR;
    }
    addBuf[0] = (flash_address & 0xFF0000) >> 16;
    addBuf[1] = (flash_address & 0xFF00) >> 8;
    addBuf[2] = flash_address & 0xFF;
    if (Spi3Drv.Write  (addBuf, 3) != HAL_OK)/* 发送扇区地址的高8bit */
    {
        return HAL_ERROR;
    }

    if (Spi3Drv.Read (target_address, byte_length) != HAL_OK)/* 读一个字节并存储到pBuf，读完后指针自加1 */
    {
        return HAL_ERROR;
    }
    DISABLE_FLASH_SPI_CMU();

    return HAL_OK;
}

HAL_StatusTypeDef drv_FlashRead (uint8_t * pBuffer, uint32_t readAddr, uint32_t length)
{
    uint32_t j = 0;
    
    if ((length % N25Q128A_SUBSECTOR_SIZE) != 0) /* 整个扇区都改写 */
    {
        return HAL_ERROR;/*此驱动用于FATFS文件系统，定义扇区大小为4096*/
    }
    
    while (drv_FlashIsBusy ())
    {
        vTaskDelay (1);/*等待1ms 确定器件的4kb 擦出完成*/
    }
     
    for (j = 0; j < length / N25Q128A_PAGE_SIZE; j++)
	{
        if(CMD_READ(readAddr,pBuffer,N25Q128A_PAGE_SIZE)!=HAL_OK)
	    {
            return HAL_ERROR;
	    }
        
        pBuffer += N25Q128A_PAGE_SIZE;
        readAddr += N25Q128A_PAGE_SIZE;
    }
    
    return HAL_OK;
}
//#endif

// Return Message
extern SPI_HandleTypeDef hEPgSpi;


#define    CLK_PERIOD                20// unit: ns
#define    Min_Cycle_Per_Inst        12// cycle count of one instruction
#define    One_Loop_Inst             8// instruction count of one loop (estimate)

#define    tSE              		200000000      // 200ms
#define    tPP              		3000000        // 3ms

#define    SectorEraseCycleTime        tSE / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#define    PageProgramCycleTime        tPP / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)

#define    SIO              		0
#define    DIO              		1
#define FlashSize           N25Q128A_FLASH_SIZE
#define MX25_CS_Low()  EN_FLASH_SPI_CMU()
#define MX25_CS_High   DISABLE_FLASH_SPI_CMU
#define hspi3           hEPgSpi
#define FLASH_CMD_WREN  WRITE_ENABLE_CMD
#define FLASH_CMD_SE    SUBSECTOR_ERASE_CMD
#define FLASH_CMD_RDSR  READ_STATUS_REG_CMD
#define FLASH_WIP_MASK  WRITE_STATUS_REG_CMD
#define FLASH_CMD_PP    PAGE_PROG_CMD
#define FLASH_CMD_READ  READ_CMD


#define usr_para_check(expr) ((void)0)

ReturnMsg MX25_SPI_ReadWriteByte(SPI_HandleTypeDef *hspi,uint8_t tx_temp,uint8_t * rx_temp)
{
//	uint16_t 	u16TimeOut;
	ReturnMsg 	mx25_state = FlashOperationSuccess;
    HAL_StatusTypeDef state = HAL_OK;

    * rx_temp =Spi3Drv.WriteRead(tx_temp);
    
    //state = HAL_SPI_TransmitReceive(hspi, &tx_temp, rx_temp, 1, 100);
    state =HAL_OK;
    if(HAL_OK != state)
    {
        mx25_state = FlashWriteRegFailed;
    }
    else
    {
        mx25_state = FlashOperationSuccess;
    }

    /* Return the operation state from the SPI bus */
    return mx25_state;
}

 ReturnMsg CMD_RDSR( uint8_t *StatusReg )
{
    uint8_t gDataBuffer;
	uint8_t rx_temp;

    MX25_CS_Low();							// Chip select go low to start a flash command

    // Send command
	MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_RDSR, &rx_temp);
	MX25_SPI_ReadWriteByte(&hspi3, 0, &gDataBuffer);

    MX25_CS_High();							// Chip select go high to end a flash command

    *StatusReg = gDataBuffer;

    return FlashOperationSuccess;
}

bool IsFlashBusy( void )
{
    uint8_t  gDataBuffer;

    CMD_RDSR( &gDataBuffer );
    if( (gDataBuffer & FLASH_WIP_MASK)  == FLASH_WIP_MASK )
    {
    	#ifdef FLASH_DEBUG
			//printf("[FLASH] Busy,0x%02x...\r\n",gDataBuffer);
		#endif
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

bool WaitFlashReady( uint32_t ExpectTime )
{
//    uint32_t temp = 0;
    while( IsFlashBusy() )
    {
//        if( temp > ExpectTime )
//        {
//        	#ifdef FLASH_DEBUG
//				printf("[FLASH] Flash no Ready...\r\n");
//			#endif
//            return FALSE;
//        }
//        temp ++;
        
    }

	return TRUE;
}


void SendFlashAddr( uint32_t flash_address, uint8_t io_mode, bool addr_4byte_mode )
{
	uint8_t rx_temp;

    /* Check flash is 3-byte or 4-byte mode.
       4-byte mode: Send 4-byte address (A31-A0)
       3-byte mode: Send 3-byte address (A23-A0) */
    if( addr_4byte_mode == TRUE )
	{
		MX25_SPI_ReadWriteByte(&hspi3, (flash_address >> 24), &rx_temp);	// A31-A24
    }

    /* A23-A0 */
	MX25_SPI_ReadWriteByte(&hspi3, (flash_address >> 16), &rx_temp);
	MX25_SPI_ReadWriteByte(&hspi3, (flash_address >> 8), &rx_temp);
	MX25_SPI_ReadWriteByte(&hspi3, (flash_address), &rx_temp);
}

ReturnMsg CMD_WREN( void )
{
    uint8_t rx_temp;
	ReturnMsg msg;

    MX25_CS_Low();							// Chip select go low to start a flash command
    
    // Write Enable command = 0x06, Setting Write Enable Latch Bit
	msg = MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_WREN, &rx_temp);
	if(msg != FlashOperationSuccess)
	{
        #ifdef FLASH_DEBUG
            printf("ERROR,0x%02x\r\n",msg);
        #endif
	}

    // Chip select go high to end a flash command
    MX25_CS_High();

    return FlashOperationSuccess;
}

//SE
ReturnMsg CMD_SECTOR( uint32_t flash_address )
{
	uint8_t  rd_tmp;
	ReturnMsg msg = FlashOperationSuccess;

	#ifdef FLASH_DEBUG
		printf("\r\nCMD_SE,flash_address = 0x%08x\r\n",flash_address);
	#endif

    // Check flash address
    if( flash_address > FlashSize )
		return FlashAddressInvalid;

    // Check flash is busy or not
    if( IsFlashBusy() )
		return FlashIsBusy;

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    MX25_CS_Low();

    //Write Sector Erase command = 0x20;
	msg = MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_SE, &rd_tmp);
	if(msg == FlashOperationSuccess)
	{
		SendFlashAddr( flash_address, SIO, FALSE);
	}

    // Chip select go high to end a flash command
    MX25_CS_High();
    
    if(WaitFlashReady( SectorEraseCycleTime ))
    {
        msg = FlashOperationSuccess;
    }
    else
    {
        msg = FlashTimeOut;
    }

	return msg;
}

ReturnMsg CMD_PageProg( uint32_t flash_address, void *source_address, uint32_t byte_length )
{
    uint32_t index;
    uint8_t  rd_tmp;

    /* Check parameters */
	usr_para_check(flash_address <= FlashSize);
	usr_para_check(source_address != NULL);
	usr_para_check(byte_length <= N25Q128A_PAGE_SIZE);

	#ifdef FLASH_DEBUG
		printf("CMD_PageProgram,flash_address = 0x%08x,byte_length = 0x%08x\r\n",flash_address,byte_length);
	#endif

    // Check flash address
    if( flash_address > FlashSize )
	{
		return FlashAddressInvalid;
	}

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    MX25_CS_Low();

    // Write Page Program command
	MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_PP, &rd_tmp);
    SendFlashAddr(flash_address, SIO, FALSE);

    // Set a loop to down load whole page data into flash's buffer
    // Note: only last 256 byte ( or 32 byte ) will be programmed
    for( index = 0; index < byte_length; index ++ )
    {
		MX25_SPI_ReadWriteByte(&hspi3, *((uint8_t *)source_address + index), &rd_tmp);
    }

    // Chip select go high to end a flash command
    MX25_CS_High();

    if( WaitFlashReady(PageProgramCycleTime) )
	{
	    return FlashOperationSuccess;
	}
    else
	{
	    return FlashTimeOut;
	}
}

ReturnMsg CMD_READ_PAGE( uint32_t flash_address, uint8_t *target_address, uint32_t byte_length )
{
    uint32_t index;
    uint8_t  rx_tmp;

	#ifdef FLASH_DEBUG
		printf("CMD_READ,flash_address = 0x%08x,byte_length = 0x%08x (%d)\r\n",flash_address,byte_length,byte_length);
	#endif
	if(byte_length >= N25Q128A_PAGE_SIZE)
	{
		byte_length = N25Q128A_PAGE_SIZE;
	}

    // Check flash address
    if( flash_address > FlashSize ) return FlashAddressInvalid;

    // Chip select go low to start a flash command
    MX25_CS_Low();

    // Write READ command and address
	MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_READ, &rx_tmp);
    SendFlashAddr(flash_address, SIO, FALSE);

    // Set a loop to read data into buffer
    for( index=0; index < byte_length; index ++)
    {
        // Read data one byte at a time
		MX25_SPI_ReadWriteByte(&hspi3, 0, target_address + index);
    }

    // Chip select go high to end a flash command
    MX25_CS_High();

    return FlashOperationSuccess;
}

HAL_StatusTypeDef drv_FlashReadSector (uint8_t * pBuffer, uint32_t readAddr, uint32_t length)
{
    uint32_t j=0;
    uint32_t offset=0;
    
    if ((length % N25Q128A_SUBSECTOR_SIZE) != 0) /* 整个扇区都改写 */
    {
        return HAL_ERROR;/*此驱动用于FATFS文件系统，定义扇区大小为4096*/
    }

    for (j = 0; j < length / 256; j++)
	{
        if(CMD_READ(readAddr,pBuffer+offset,N25Q128A_PAGE_SIZE)!=HAL_OK)
	    {
	      return HAL_ERROR;
	    }
        
        offset+=N25Q128A_PAGE_SIZE;
        readAddr+=N25Q128A_PAGE_SIZE;
    }

    return HAL_OK;
}

HAL_StatusTypeDef drv_FlashWriteSector (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length)
{
    uint32_t i = 0, j = 0;
    uint32_t offset =0;

    if ((length % N25Q128A_SUBSECTOR_SIZE) != 0) /* 整个扇区都改写 */
    {
        return HAL_ERROR;/*此驱动用于FATFS文件系统，定义扇区大小为4096*/
    }

    for (i = 0; i < (length >>12) ; i++)/*有几个扇区*/
    {
        if (CMD_SE (writeAddr + i * 4096) != HAL_OK) /* 擦出扇区 */
        {
            return HAL_ERROR;
        }
    }
     
    for (j = 0; j < length / 256; j++)
	{
        if(CMD_PageProgram(writeAddr,pBuffer,N25Q128A_PAGE_SIZE) != HAL_OK)
	    {
            return HAL_ERROR;
	    }
        
        offset += N25Q128A_PAGE_SIZE;
        writeAddr += N25Q128A_PAGE_SIZE;
    }
    
    return HAL_OK;
}
