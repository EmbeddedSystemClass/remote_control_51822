/*
 * COPYRIGHT (c) 2010-2014 MACRONIX INTERNATIONAL CO., LTD
 * SPI Flash Low Level Driver (LLD) Sample Code
 *
 * SPI interface command set
 *
 * $Id: MX25_CMD.c,v 1.29 2013/08/12 02:56:37 mxclldb1 Exp $
 */

#include "MX25L1606E.h"
#include "platform.h"
#include "stm32f2xx_hal.h"

extern SPI_HandleTypeDef hspi3;

/*
 --Common functions
 */

/*
 * Function:       Wait_Flash_WarmUp
 * Arguments:      None.
 * Description:    Wait some time until flash read / write enable.
 * Return Message: None.
 */
void Wait_Flash_WarmUp()
{
    uint32 time_cnt = FlashFullAccessTime;
    while( time_cnt > 0 )
    {
        time_cnt--;
    }
}

/*
 * Function:       CS_Low, CS_High
 * Arguments:      None.
 * Description:    Chip select go low / high.
 * Return Message: None.
 */
void MX25_CS_Low(void)
{
    HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_RESET);        // CS = 0
}

void MX25_CS_High(void)
{
	HAL_GPIO_WritePin(FLASH_SPI_CS_GPIO_Port, FLASH_SPI_CS_Pin, GPIO_PIN_SET);			// CS = 1
}


/*******************************************************************************
* @brief  Sends a byte through the SPI interface and return the byte
* @param  SPIx: To select the SPIx/I2Sx peripheral
* @param  temp: Data to be transmitted.l
* @retval The value of the received data.
*******************************************************************************/
ReturnMsg MX25_SPI_ReadWriteByte(SPI_HandleTypeDef *hspi,uint8_t tx_temp,uint8_t * rx_temp)
{
	ReturnMsg 	mx25_state = FlashOperationSuccess;
    HAL_StatusTypeDef state = HAL_OK;
    
    state = HAL_SPI_TransmitReceive(hspi, &tx_temp, rx_temp, 1, 100);
    if(HAL_OK != state)
    {
        mx25_state = FlashWriteRegFailed;
        EnterAppError(BOOT_ERROR_CODE_ERASE_FAIL);
        Usr_Error_Handler(__FILE__, __LINE__, state);
    }
    else
    {
        mx25_state = FlashOperationSuccess;
    }

    /* Return the operation state from the SPI bus */
    return mx25_state;
}

/*
 * Function:       InsertDummyCycle
 * Arguments:      dummy_cycle, number of dummy clock cycle
 * Description:    Insert dummy cycle of SCLK
 * Return Message: None.
 */
void InsertDummyCycle( uint8 dummy_num)
{
	uint16_t i = 0;
	uint8_t rx_temp;

	for(i = 0;i < dummy_num;i ++)
	{
		MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_DUMMY, &rx_temp);
	}
}

/*
 * Function:       WaitFlashReady
 * Arguments:      ExpectTime, expected time-out value of flash operations.
 *                 No use at non-synchronous IO mode.
 * Description:    Synchronous IO:
 *                 If flash is ready return TRUE.
 *                 If flash is time-out return FALSE.
 *                 Non-synchronous IO:
 *                 Always return TRUE
 * Return Message: TRUE, FALSE
 */
BOOL WaitFlashReady( uint32 ExpectTime )
{
    uint32 temp = 0;
    while( IsFlashBusy() )
    {
        if( temp > ExpectTime )
        {
        	#ifdef FLASH_DEBUG
				printf("[FLASH] Flash no Ready...\r\n");
			#endif
            return FALSE;
        }
        temp ++;
    }

	return TRUE;
}

/*
 * Function:       WaitRYBYReady
 * Arguments:      ExpectTime, expected time-out value of flash operations.
 *                 No use at non-synchronous IO mode.
 * Description:    Synchronous IO:
 *                 If flash is ready return TRUE.
 *                 If flash is time-out return FALSE.
 *                 Non-synchronous IO:
 *                 Always return TRUE
 * Return Message: TRUE, FALSE
 */
BOOL WaitRYBYReady( uint32 ExpectTime )
{
#ifndef NON_SYNCHRONOUS_IO
    uint32 temp = 0;
#ifdef GPIO_SPI
    while( SO == 0 )
#else
    // Insert your code for waiting RYBY (SO) pin ready
#endif
    {
        if( temp > ExpectTime )
        {
            return FALSE;
        }
        temp = temp + 1;
    }
    return TRUE;

#else
    return TRUE;
#endif
}

/*
 * Function:       IsFlashBusy
 * Arguments:      None.
 * Description:    Check status register WIP bit.
 *                 If  WIP bit = 1: return TRUE ( Busy )
 *                             = 0: return FALSE ( Ready ).
 * Return Message: TRUE, FALSE
 */
BOOL IsFlashBusy( void )
{
    uint8  gDataBuffer;

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

/*
 * Function:       IsFlashQIO
 * Arguments:      None.
 * Description:    If flash QE bit = 1: return TRUE
 *                                 = 0: return FALSE.
 * Return Message: TRUE, FALSE
 */
BOOL IsFlashQIO( void )
{
#ifdef FLASH_NO_QE_BIT
    return TRUE;
#else
    uint8  gDataBuffer;
    CMD_RDSR( &gDataBuffer );
    if( (gDataBuffer & FLASH_QE_MASK) == FLASH_QE_MASK )
        return TRUE;
    else
        return FALSE;
#endif
}
/*
 * Function:       IsFlash4Byte
 * Arguments:      None
 * Description:    Check flash address is 3-byte or 4-byte.
 *                 If flash 4BYTE bit = 1: return TRUE
 *                                    = 0: return FALSE.
 * Return Message: TRUE, FALSE
 */
BOOL IsFlash4Byte( void )
{
#ifdef FLASH_CMD_RDSCUR
    #ifdef FLASH_4BYTE_ONLY
        return TRUE;
    #elif FLASH_3BYTE_ONLY
        return FALSE;
    #else
        uint8  gDataBuffer;
        CMD_RDSCUR( &gDataBuffer );
        if( (gDataBuffer & FLASH_4BYTE_MASK) == FLASH_4BYTE_MASK )
            return TRUE;
        else
            return FALSE;
    #endif
#else
    return FALSE;
#endif
}
/*
 * Function:       SendFlashAddr
 * Arguments:      flash_address, 32 bit flash memory address
 *                 io_mode, I/O mode to transfer address
 *                 addr_4byte_mode,
 * Description:    Send flash address with 3-byte or 4-byte mode.
 * Return Message: None
 */
void SendFlashAddr( uint32 flash_address, uint8 io_mode, BOOL addr_4byte_mode )
{
	uint8 rx_temp;

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

/*
 * Function:       GetDummyCycle
 * Arguments:      default_cycle, default dummy cycle
 *                 fsptr, pointer of flash status structure
 * Description:    Get dummy cycle for different condition
 *                 default_cycle: Byte3 | Byte2 | Byte1 | Byte0
 *                      DC 1 bit:   x       1       x       0
 *                      DC 2 bit:   11      10      01      00
 *                 Note: the variable dummy cycle only support
                         in some product.
 * Return Message: Dummy cycle value
 */
uint8 GetDummyCycle( uint32 default_cycle )
{
#ifdef FLASH_CMD_RDCR
    uint8 gDataBuffer;
    uint8 dummy_cycle = default_cycle;
    CMD_RDCR( &gDataBuffer );
    #ifdef SUPPORT_CR_DC
        // product support 1-bit dummy cycle configuration
        if( (gDataBuffer & FLASH_DC_MASK) == FLASH_DC_MASK )
            dummy_cycle = default_cycle >> 16;
        else
            dummy_cycle = default_cycle;
    #elif SUPPORT_CR_DC_2bit
        // product support 2-bit dummy cycle configuration
        switch( gDataBuffer & FLASH_DC_2BIT_MASK ){
            case 0x00:
                dummy_cycle = default_cycle;
            break;
            case 0x40:
                dummy_cycle = default_cycle >> 8;
            break;
            case 0x80:
                dummy_cycle = default_cycle >> 16;
            break;
            case 0xC0:
                dummy_cycle = default_cycle >> 24;
            break;
        }
    #else
         // configuration register not support dummy configuration
         dummy_cycle = default_cycle;
    #endif
    return dummy_cycle;
#else
    // default case: return default dummy cycle
    return default_cycle;
#endif
}



/*
 * ID Command
 */

/*
 * Function:       CMD_RDID
 * Arguments:      Identification, 32 bit buffer to store id
 * Description:    The RDID instruction is to read the manufacturer ID
 *                 of 1-byte and followed by Device ID of 2-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_ReadID( uint32 *Identification )
{
    uint32 temp;
    uint8  gDataBuffer[3];
	uint8 rx_temp;
	uint8 i;

	/* Check parameters*/
	usr_para_check(Identification != NULL);

	// Chip select go low to start a flash command
    MX25_CS_Low();

    // Send command
	MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_RDID, &rx_temp);
    InsertDummyCycle(3);

    // Get manufacturer identification, device identification
	for(i = 0;i < 3;i ++)
	{
		MX25_SPI_ReadWriteByte(&hspi3, 0, gDataBuffer + i);
	}

	// Chip select go high to end a command
    MX25_CS_High();

    // Store identification
    temp =  gDataBuffer[0];
    temp =  (temp << 8) | gDataBuffer[1];
    *Identification =  (temp << 8) | gDataBuffer[2];

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_RES
 * Arguments:      ElectricIdentification, 8 bit buffer to store electric id
 * Description:    The RES instruction is to read the Device
 *                 electric identification of 1-byte.
 * Return Message: FlashOperationSuccess
 */
 ReturnMsg CMD_RES( uint8 *ElectricIdentification )
{
	uint8 rx_temp;

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Send flash command and insert dummy cycle
//    //SendByte( FLASH_CMD_RES, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_RES, &rx_temp);
//    InsertDummyCycle( 3 );

//    // Get electric identification
//    MX25_SPI_ReadWriteByte(SysSPI, 0, &rx_temp);

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

	*ElectricIdentification = rx_temp;

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_REMS
 * Arguments:      REMS_Identification, 16 bit buffer to store id
 *                 fsptr, pointer of flash status structure
 * Description:    The REMS instruction is to read the Device
 *                 manufacturer ID and electric ID of 1-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_REMS( uint16 *REMS_Identification, FlashStatus *fsptr )
{
//    uint8  gDataBuffer[2],rx_temp;

    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Send flash command and insert dummy cycle ( if need )
//    // ArrangeOpt = 0x00 will output the manufacturer's ID first
//    //            = 0x01 will output electric ID first
//    //SendByte( FLASH_CMD_REMS, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_REMS, &rx_temp);
//    InsertDummyCycle( 2);
//    //SendByte( fsptr->ArrangeOpt, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, fsptr->ArrangeOpt, &rx_temp);

//    // Get ID
//    //gDataBuffer[0] = GetByte( SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, fsptr->ArrangeOpt, gDataBuffer);
//    //gDataBuffer[1] = GetByte( SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, fsptr->ArrangeOpt, (gDataBuffer+1));

//    // Store identification informaion
//    *REMS_Identification = (gDataBuffer[0] << 8) | gDataBuffer[1];

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}


/*
 * Register  Command
 */

/*
 * Function:       CMD_RDSR
 * Arguments:      StatusReg, 8 bit buffer to store status register value
 * Description:    The RDSR instruction is for reading Status Register Bits.
 * Return Message: FlashOperationSuccess
 */
 ReturnMsg CMD_RDSR( uint8 *StatusReg )
{
    uint8 gDataBuffer;
	uint8 rx_temp;

    MX25_CS_Low();							// Chip select go low to start a flash command

    // Send command
	MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_RDSR, &rx_temp);
	MX25_SPI_ReadWriteByte(&hspi3, 0, &gDataBuffer);

    MX25_CS_High();							// Chip select go high to end a flash command

    *StatusReg = gDataBuffer;

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_WRSR
 * Arguments:      UpdateValue, 8/16 bit status register value to updata
 * Description:    The WRSR instruction is for changing the values of
 *                 Status Register Bits (and configuration register)
 * Return Message: FlashIsBusy, FlashTimeOut, FlashOperationSuccess
 */
#ifdef SUPPORT_WRSR_CR
ReturnMsg CMD_WRSR( uint16 UpdateValue )
#else
ReturnMsg CMD_WRSR( uint8 UpdateValue )
#endif
{
//	uint8 rx_temp;

    // Check flash is busy or not
    if( IsFlashBusy() )    return FlashIsBusy;

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Send command and update value
//    //SendByte( FLASH_CMD_WRSR, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_WRSR, &rx_temp);
//    //SendByte( UpdateValue, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, UpdateValue, &rx_temp);
//#ifdef SUPPORT_WRSR_CR
//    //SendByte( UpdateValue >> 8, SIO );    // write configuration register
//    MX25_SPI_ReadWriteByte(SysSPI, UpdateValue >> 8, &rx_temp);
//#endif

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

//    if( WaitFlashReady( WriteStatusRegCycleTime ) )
        return FlashOperationSuccess;
//    else
//        return FlashTimeOut;

}

/*
 * Function:       CMD_RDSCUR
 * Arguments:      SecurityReg, 8 bit buffer to store security register value
 * Description:    The RDSCUR instruction is for reading the value of
 *                 Security Register bits.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RDSCUR( uint8 *SecurityReg )
{
//    uint8  gDataBuffer,rx_temp;

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    //Send command
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_RDSCUR, &rx_temp);
//	MX25_SPI_ReadWriteByte(SysSPI, 0, &gDataBuffer);

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

//    *SecurityReg = gDataBuffer;

    return FlashOperationSuccess;

}

/*
 * Function:       CMD_WRSCUR
 * Arguments:      None.
 * Description:    The WRSCUR instruction is for changing the values of
 *                 Security Register Bits.
 * Return Message: FlashIsBusy, FlashOperationSuccess, FlashWriteRegFailed,
 *                 FlashTimeOut
 */
ReturnMsg CMD_WRSCUR( void )
{
    uint8  gDataBuffer;
//    uint8_t rx_temp;

    // Check flash is busy or not
    if( IsFlashBusy() )    return FlashIsBusy;

//    // Setting Write Enable Latch bit
//    CMD_WREN();

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Write WRSCUR command
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_WRSCUR, &rx_temp);

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    if( WaitFlashReady( WriteSecuRegCycleTime ) )
	{
        CMD_RDSCUR( &gDataBuffer );

        // Check security register LDSO bit
        if( (gDataBuffer & FLASH_LDSO_MASK) == FLASH_LDSO_MASK )
                return FlashOperationSuccess;
        else
                return FlashWriteRegFailed;
    }
    else
        return FlashTimeOut;

}


/*
 * Read Command
 */

/*
 * Function:       CMD_READ
 * Arguments:      flash_address, 32 bit flash memory address
 *                 target_address, buffer address to store returned data
 *                 byte_length, length of returned data in byte unit
 * Description:    The READ instruction is for reading data out.
 * Return Message: FlashAddressInvalid, FlashOperationSuccess
 */
ReturnMsg CMD_READ( uint32 flash_address, uint8 *target_address, uint32 byte_length )
{
    uint32 index;
//    uint8  addr_4byte_mode;
    uint8_t rx_tmp;

	#ifdef FLASH_DEBUG
		printf("CMD_READ,flash_address = 0x%08x,byte_length = 0x%08x (%d)\r\n",flash_address,byte_length,byte_length);
	#endif
	if(byte_length >= PAGE_SIZE)
	{
		byte_length = PAGE_SIZE;
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


/*
 * Function:       CMD_DREAD
 * Arguments:      flash_address, 32 bit flash memory address
 *                 target_address, buffer address to store returned data
 *                 byte_length, length of returned data in byte unit
 * Description:    The DREAD instruction enable double throughput of Serial
 *                 Flash in read mode
 * Return Message: FlashAddressInvalid, FlashOperationSuccess
 */
ReturnMsg CMD_DREAD( uint32 flash_address, uint8 *target_address, uint32 byte_length )
{
//    uint32 index;
//    uint8  addr_4byte_mode;
//    uint8  dc;
//	uint8  rx_tmp;

    // Check flash address
    if( flash_address > FlashSize ) return FlashAddressInvalid;

    // Check 3-byte or 4-byte mode
//    if( IsFlash4Byte() )
//        addr_4byte_mode = TRUE;    // 4-byte mode
//    else
//        addr_4byte_mode = FALSE;   // 3-byte mode

//    // get dummy cycle number
//    dc = GetDummyCycle( DUMMY_CONF_DREAD );

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Write 2-I/O Read command and address
//    //SendByte( FLASH_CMD_DREAD, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_DREAD, &rx_tmp);
//    SendFlashAddr( flash_address, SIO, addr_4byte_mode );
//    InsertDummyCycle( dc );                    // Wait 8 dummy cycle

//    // Set a loop to read data into data buffer
//    for( index=0; index < byte_length; index++ )
//    {
//        //*(target_address + index) = GetByte( DIO );
//		MX25_SPI_ReadWriteByte(SysSPI, *(target_address + index), &rx_tmp);
//    }

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}


/*
 * Function:       CMD_FASTREAD
 * Arguments:      flash_address, 32 bit flash memory address
 *                 target_address, buffer address to store returned data
 *                 byte_length, length of returned data in byte unit
 * Description:    The FASTREAD instruction is for quickly reading data out.
 * Return Message: FlashAddressInvalid, FlashOperationSuccess
 */
ReturnMsg CMD_FASTREAD( uint32 flash_address, uint8 *target_address, uint32 byte_length )
{
//    uint32 index;
//    uint8  addr_4byte_mode;
//    uint8  dc;
//	uint8  rx_tmp;

    // Check flash address
    if( flash_address > FlashSize ) return FlashAddressInvalid;

    // Check 3-byte or 4-byte mode
//    if( IsFlash4Byte() )
//        addr_4byte_mode = TRUE;  // 4-byte mode
//    else
//        addr_4byte_mode = FALSE; // 3-byte mode

//    dc = GetDummyCycle( DUMMY_CONF_FASTREAD );

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Write Fast Read command, address and dummy cycle
//    //SendByte( FLASH_CMD_FASTREAD, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_FASTREAD, &rx_tmp);
//    SendFlashAddr( flash_address, SIO, addr_4byte_mode );
//    InsertDummyCycle ( dc );          // Wait dummy cycle

//    // Set a loop to read data into data buffer
//    for( index=0; index < byte_length; index++ )
//    {
//		MX25_SPI_ReadWriteByte(SysSPI, 0, target_address + index);
//    }

    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}


/*
 * Function:       CMD_RDSFDP
 * Arguments:      flash_address, 32 bit flash memory address
 *                 target_address, buffer address to store returned data
 *                 byte_length, length of returned data in byte unit
 * Description:    RDSFDP can retrieve the operating characteristics, structure
 *                 and vendor-specified information such as identifying information,
 *                 memory size, operating voltages and timinginformation of device
 * Return Message: FlashAddressInvalid, FlashOperationSuccess
 */
ReturnMsg CMD_RDSFDP( uint32 flash_address, uint8 *target_address, uint32 byte_length )
{
//    uint32 index;
//    uint8  addr_4byte_mode,rx_tmp;

    // Check flash address
    if( flash_address > FlashSize ) return FlashAddressInvalid;

    // Check 3-byte or 4-byte mode
//    if( IsFlash4Byte() )
//        addr_4byte_mode = TRUE;  // 4-byte mode
//    else
//        addr_4byte_mode = FALSE; // 3-byte mode

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Write Read SFDP command
//    //SendByte( FLASH_CMD_RDSFDP, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_RDSFDP, &rx_tmp);
//    SendFlashAddr( flash_address, SIO, addr_4byte_mode );
//    InsertDummyCycle ( 1 );        // Insert dummy cycle

//    // Set a loop to read data into data buffer
//    for( index=0; index < byte_length; index++ )
//    {
//        //*(target_address + index) = GetByte( SIO );
//		MX25_SPI_ReadWriteByte(SysSPI, 0, target_address + index);
//    }

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}
/*
 * Program Command
 */

/*
 * Function:       CMD_WREN
 * Arguments:      None.
 * Description:    The WREN instruction is for setting
 *                 Write Enable Latch (WEL) bit.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_WREN( void )
{
    uint8 rx_temp;
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

/*
 * Function:       CMD_WRDI
 * Arguments:      None.
 * Description:    The WRDI instruction is to reset
 *                 Write Enable Latch (WEL) bit.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_WRDI( void )
{
	uint8 rx_temp;

	// Chip select go low to start a flash command
    MX25_CS_Low();

    // Write Disable command = 0x04, resets Write Enable Latch Bit
	MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_WRDI, &rx_temp);

    MX25_CS_High();

    return FlashOperationSuccess;
}


/*
 * Function:       CMD_PP
 * Arguments:      flash_address, 32 bit flash memory address
 *                 source_address, buffer address of source data to program
 *                 byte_length, byte length of data to programm
 * Description:    The PP instruction is for programming
 *                 the memory to be "0".
 *                 The device only accept the last 256 byte ( or 32 byte ) to program.
 *                 If the page address ( flash_address[7:0] ) reach 0xFF, it will
 *                 program next at 0x00 of the same page.
 *                 Some products have smaller page size ( 32 byte )
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg CMD_PageProgram( uint32 flash_address, void *source_address, uint32 byte_length )
{
    uint32 index;
//    uint8  addr_4byte_mode;
    uint8_t rd_tmp;
//	ReturnMsg msg;

    /* Check parameters */
	usr_para_check(flash_address <= FlashSize);
	usr_para_check(source_address != NULL);
	usr_para_check(byte_length <= PAGE_SIZE);

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


/*
 * Erase Command
 */

/*
 * Function:       CMD_SE
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The SE instruction is for erasing the data
 *                 of the chosen sector (4KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg CMD_SE( uint32 flash_address )
{
//    uint8  addr_4byte_mode;
	uint8  rd_tmp;
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


/*
 * Function:       CMD_BE
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The BE instruction is for erasing the data
 *                 of the chosen sector (64KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg CMD_BE( uint32 flash_address )
{
//    uint8  addr_4byte_mode;
//	uint8  rd_tmp;

//    // Check flash address
//    if( flash_address > FlashSize ) return FlashAddressInvalid;

//    // Check flash is busy or not
//    if( IsFlashBusy() )    return FlashIsBusy;

//    // Check 3-byte or 4-byte mode
//    if( IsFlash4Byte() )
//        addr_4byte_mode = TRUE;  // 4-byte mode
//    else
//        addr_4byte_mode = FALSE; // 3-byte mode

//    // Setting Write Enable Latch bit
//    CMD_WREN();

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    //Write Block Erase command = 0xD8;
//    //SendByte( FLASH_CMD_BE, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_BE, &rd_tmp);
//    SendFlashAddr( flash_address, SIO, addr_4byte_mode );

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    if( WaitFlashReady( BlockEraseCycleTime ) )
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}

/*
 * Function:       CMD_CE
 * Arguments:      None.
 * Description:    The CE instruction is for erasing the data
 *                 of the whole chip to be "1".
 * Return Message: FlashIsBusy, FlashOperationSuccess, FlashTimeOut
 */
ReturnMsg CMD_CE( void )
{
	uint8 rx_temp;

    // Check flash is busy or not
    if( IsFlashBusy() )    return FlashIsBusy;

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    MX25_CS_Low();

    //Write Chip Erase command = 0x60;
	MX25_SPI_ReadWriteByte(&hspi3, FLASH_CMD_CE, &rx_temp);

    // Chip select go high to end a flash command
    MX25_CS_High();

    if( WaitFlashReady( ChipEraseCycleTime ) )
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}


/*
 * Mode setting Command
 */

/*
 * Function:       CMD_DP
 * Arguments:      None.
 * Description:    The DP instruction is for setting the
 *                 device on the minimizing the power consumption.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_DP( void )
{
//	uint8 rx_temp;

    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Deep Power Down Mode command
//    //SendByte( FLASH_CMD_DP, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_DP, &rx_temp);

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_RDP
 * Arguments:      None.
 * Description:    The Release from RDP instruction is
 *                 putting the device in the Stand-by Power mode.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RDP( void )
{
//	uint8 rx_temp;

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Deep Power Down Mode command
//    //SendByte( FLASH_CMD_RDP, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_RDP, &rx_temp);

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_ENSO
 * Arguments:      None.
 * Description:    The ENSO instruction is for entering the secured OTP mode.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_ENSO( void )
{
//	uint8 rx_temp;

//    // Chip select go low to start a flash command
//    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Write ENSO command
//    //SendByte( FLASH_CMD_ENSO, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_ENSO, &rx_temp);

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_EXSO
 * Arguments:      None.
 * Description:    The EXSO instruction is for exiting the secured OTP mode.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_EXSO( void )
{
//	uint8 rx_temp;

//    // Chip select go low to start a flash command
////    SPI_Cmd(SysSPI, ENABLE);
//    MX25_CS_Low();

//    // Write EXSO command = 0xC1
//    //SendByte( FLASH_CMD_EXSO, SIO );
//	MX25_SPI_ReadWriteByte(SysSPI, FLASH_CMD_ENSO, &rx_temp);

//    // Chip select go high to end a flash command
//    MX25_CS_High();
//	SPI_Cmd(SysSPI, DISABLE);

    return FlashOperationSuccess;
}

/*
 * Reset setting Command
 */

/*
 * Security Command
 */

/*
 * Suspend/Resume Command
 */

