/**
  ******************************************************************************
  *            
  *          +--------------------------------------
  *          |                     Pin assignment                       
  *          +-----------------------------+-----------
  *          |  STM32 SPI Pins                            |     CC1101    
  *          +-----------------------------+-----------
  *          | cc1101_CS_PIN(SPI1_NSS)  PA4)  |    CSN    
  *          | cc1101_SPI_MISO_PIN / MISO      |    SO 
  *          | cc1101_SPI_MOSI_PIN / MOSI      |    SI  
  *          | cc1101_SPI_SCK_PIN / SCLK        |   CLK
  *          |            Interrupt PIN   /PB10         |   GDO0  
  *          |                                   PB11          |   GDO2 
  *          +-----------------------------+---------------+-------------+  

  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/

#include "cc1101_spi.h"
#include "stm32f10x_conf.h"
#include "hal.h"


static uint8_t spiRegAccess(uint8_t addrByte, uint8_t writeValue);
static BOOL spiBurstFifoAccess(uint8_t addrByte, uint8_t * pData, uint8_t len);

 
 void mrfiSpiInit(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  EXTI_InitTypeDef EXTI_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;

  /*!< CC1101_SPI_CS_GPIO, CC1101_SPI_MOSI_GPIO,CC1101_SPI_MISO_GPIO 
       and CC1101_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(CC1101_CS_GPIO_CLK | CC1101_SPI_MOSI_GPIO_CLK | CC1101_SPI_MISO_GPIO_CLK |
                         CC1101_SPI_SCK_GPIO_CLK | CC1101_SPI_GDO0_GPIO_CLK, ENABLE);

  /*!< CC1101_SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(CC1101_SPI_CLK | RCC_APB2Periph_AFIO, ENABLE);
  
  /*!< Configure CC1101_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = CC1101_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(CC1101_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure CC1101_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = CC1101_SPI_MOSI_PIN;
  GPIO_Init(CC1101_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure CC1101_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = CC1101_SPI_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_Init(CC1101_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  
  /*!< Configure CC1101_CS_PIN pin: CC1101 CS pin */
  GPIO_InitStructure.GPIO_Pin = CC1101_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(CC1101_CS_GPIO_PORT, &GPIO_InitStructure);


  // Config extern pin, GD0 from CC1101 connects to PB10 of STM32, using EXT10 as interrupt source.
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = CC1101_SPI_GDO0_PIN;
  GPIO_Init(CC1101_SPI_GDO0_GPIO_PORT, &GPIO_InitStructure);

  SPI_Cmd(CC1101_SPI, DISABLE);
  
  /*!< SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(CC1101_SPI, &SPI_InitStructure);


   // This interrupt is used to indicate receive.  The SYNC signal goes high when a receive OR a transmit begins.  
   // It goes high once the sync word is received or transmitted and then goes low again once the packet completes.
  GPIO_EXTILineConfig(CC1101_SPI_EXT_PORT_SOURCE, CC1101_SPI_EXT_PIN_SOURCE);
  EXTI_InitStructure.EXTI_Line = CC1101_SPI_EXT_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  #if 0   //暂不开启反向中断，待CC1101初始化函数中再开启
  NVIC_InitStructure.NVIC_IRQChannel = CC1101_SPI_EXT_CHANNEL;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  #endif

  SPI_Cmd(CC1101_SPI, ENABLE);

  CC1101_CS_HIGH();

  //something to do for active CC1101?
  
}
 

void mrfiEnableExtInterrupt(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  CC1101_CLEAR_GDO0_INT_FLAG();

  NVIC_InitStructure.NVIC_IRQChannel = CC1101_SPI_EXT_CHANNEL;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

}

void mrfiDisableExtInterrupt(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  CC1101_CLEAR_GDO0_INT_FLAG();

  NVIC_InitStructure.NVIC_IRQChannel = CC1101_SPI_EXT_CHANNEL;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure); 

}

/**
  * @brief  Reads a byte from the SPI Flash.
  * @note   This function must be used only if the Start_Read_Sequence function
  *         has been previously called.
  * @param  None
  * @retval Byte Read from the SPI Flash.
  */
uint8_t CC1101_ReadByte(void)
{
  return (CC1101_SendByte(CC1101_DUMMY_BYTE));
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t CC1101_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(CC1101_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(CC1101_SPI, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(CC1101_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(CC1101_SPI);
  
}




/**************************************************************************************************
 * @fn          mrfiSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio.  Returns status byte read during transfer
 *              of strobe command.
 *
 * @param       addr - address of register to strobe
 *
 * @return      status byte of radio
 **************************************************************************************************
 */
uint8_t mrfiSpiCmdStrobe(uint8_t addr)
{
  uint8_t statusByte;
  
  MRFI_SPI_ASSERT((addr >= 0x30) && (addr <= 0x3D));  /* invalid address */

  /* disable interrupts that use SPI */
  //MRFI_SPI_ENTER_CRITICAL_SECTION(s);

  /* turn chip select "off" and then "on" to clear any current SPI access */
  CC1101_CS_HIGH();
  CC1101_CS_LOW();

 while(CC1101_CHIP_SO_IS_HIGH());
 
  /* send the command strobe, wait for SPI access to complete */
  statusByte = CC1101_SendByte(addr);

  /* read the readio status byte returned by the command strobe */
  //statusByte = CC1101_ReadByte();

  /* turn off chip select; enable interrupts that call SPI functions */
  CC1101_CS_HIGH();
  //MRFI_SPI_EXIT_CRITICAL_SECTION(s);

  /* return the status byte */
  return(statusByte);
}


/**************************************************************************************************
 * @fn          mrfiSpiReadReg
 *
 * @brief       Read value from radio register.
 *
 * @param       addr - address of register
 *
 * @return      register value
 **************************************************************************************************
 */
uint8_t mrfiSpiReadReg(uint8_t addr)
{
  MRFI_SPI_ASSERT(addr <= 0x3B);    /* invalid address */
  
  /*
   *  The burst bit is set to allow access to read-only status registers.
   *  This does not affect normal register reads.
   */
  return( spiRegAccess(addr | MRFI_BURST_BIT | MRFI_READ_BIT, CC1101_DUMMY_BYTE) );
}


/**************************************************************************************************
 * @fn          mrfiSpiWriteReg
 *
 * @brief       Write value to radio register.
 *
 * @param       addr  - address of register
 * @param       value - register value to write
 *
 * @return      none
 **************************************************************************************************
 */
void mrfiSpiWriteReg(uint8_t addr, uint8_t value)
{
  MRFI_SPI_ASSERT((addr <= 0x2E) || (addr == 0x3E));    /* invalid address */
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
  spiRegAccess(addr, value);
}


/*=================================================================================================
 * @fn          spiRegAccess
 *
 * @brief       This function performs a read or write.  The
 *              calling code must configure the read/write bit of the register's address byte.
 *              This bit is set or cleared based on the type of access.
 *
 * @param       regAddrByte - address byte of register; the read/write bit already configured
 *
 * @return      register value

   // 务必注意: 直接从TI Simple Stack中移植的该函数错误，在读取reg的时候，原函数在发送完头部后，
   // 读取了2次，然后取第二次的值作为返回值，应该只读取1次(只写一次DUMMY BYTE)，然后取第一个
   // 返回值，否则读取的结果是错误的.
 *=================================================================================================
 */
static uint8_t spiRegAccess(uint8_t addrByte, uint8_t writeValue)
{
  uint8_t readValue;

  MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() );   /* SPI is not initialized */

  /* disable interrupts that use SPI */
  //MRFI_SPI_ENTER_CRITICAL_SECTION(s);

  /* turn chip select "off" and then "on" to clear any current SPI access */
  CC1101_CS_HIGH();
  CC1101_CS_LOW();

  while(CC1101_CHIP_SO_IS_HIGH());

  /* send register address byte, the read/write bit is already configured */
  CC1101_SendByte(addrByte);

 readValue = CC1101_SendByte(writeValue);

  /* turn off chip select; enable interrupts that call SPI functions */
  CC1101_CS_HIGH();
  //MRFI_SPI_EXIT_CRITICAL_SECTION(s);

  /* return the register value */
  return(readValue);
}


/**************************************************************************************************
 * @fn          mrfiSpiWriteTxFifo
 *
 * @brief       Write data to radio transmit FIFO.
 *
 * @param       pData - pointer for storing write data
 * @param       len   - length of data in bytes
 *
 * @return      TRUE if an interrupt was detected during the transfer, FALSE otherwise
 **************************************************************************************************
 */
BOOL mrfiSpiWriteTxFifo(uint8_t * pData, uint8_t len)
{
  return spiBurstFifoAccess(TXFIFO | MRFI_BURST_BIT, pData, len);
}


/**************************************************************************************************
 * @fn          macSpiReadRxFifo
 *
 * @brief       Read data from radio receive FIFO.
 *
 * @param       pData - pointer for storing read data
 * @param       len   - length of data in bytes
 *
 * @return      TRUE if an interrupt was detected during the transfer, FALSE otherwise
 **************************************************************************************************
 */
BOOL mrfiSpiReadRxFifo(uint8_t * pData, uint8_t len)
{
  return spiBurstFifoAccess(RXFIFO | MRFI_BURST_BIT | MRFI_READ_BIT, pData, len);
}


/*=================================================================================================
 * @fn          spiBurstFifoAccess
 *
 * @brief       Burst mode access used for reading or writing to radio FIFOs.
 *
 *              For more efficient interrupt latency, this function does not keep interrupts
 *              disabled for its entire execution.  It is designed to recover if an interrupt
 *              occurs that accesses SPI.  See comments in code for further details.
 *
 * @param       addrByte - first byte written to SPI, contains address and mode bits
 * @param       pData    - pointer to data to read or write
 * @param       len      - length of data in bytes
 *
 * @return      TRUE if an interrupt was detected during the transfer, FALSE otherwise
 *=================================================================================================
 */
static BOOL spiBurstFifoAccess(uint8_t addrByte, uint8_t * pData, uint8_t len)
{
  BOOL result = FALSE; // initialize to successful status

  MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() );   /* SPI is not initialized */
  MRFI_SPI_ASSERT(len != 0);                      /* zero length is not allowed */
  MRFI_SPI_ASSERT(addrByte & MRFI_BURST_BIT);          /* only burst mode supported */

  /* disable interrupts that use SPI */
  //MRFI_SPI_ENTER_CRITICAL_SECTION(s);

  /* turn chip select "off" and then "on" to clear any current SPI access */
  CC1101_CS_HIGH();
  CC1101_CS_LOW();

  /*-------------------------------------------------------------------------------
   *  Main loop.  If the SPI access is interrupted, execution comes back to
   *  the start of this loop.  Loop exits when nothing left to transfer.
   */
  do
  {
    /* send FIFO access command byte, wait for SPI access to complete */
    while(CC1101_CHIP_SO_IS_HIGH());
    CC1101_SendByte(addrByte);

    /*-------------------------------------------------------------------------------
     *  Inner loop.  This loop executes as long as the SPI access is not interrupted.
     *  Loop completes when nothing left to transfer.
     */
    do
    {   

      if (addrByte & MRFI_READ_BIT)
      {
        *pData = CC1101_SendByte(CC1101_DUMMY_BYTE);
      }
      else
      {
	  CC1101_SendByte(*pData);

      }

      /*-------------------------------------------------------------------------------
       *  Use idle time.  Perform increment/decrement operations before pending on
       *  completion of SPI access.
       *
       *  Decrement the length counter.  Wait for SPI access to complete.
       */
       
      len--;

      /*-------------------------------------------------------------------------------
       *  At least one byte of data has transferred.  Briefly enable (and then disable)
       *  interrupts that can call SPI functions.  This provides a window for any timing
       *  critical interrupts that might be pending.
       *
       *  To improve latency, take care of pointer increment within the interrupt
       *  enabled window.
       */
     // MRFI_SPI_EXIT_CRITICAL_SECTION(s);
      pData++;
     // MRFI_SPI_ENTER_CRITICAL_SECTION(s);

      /*-------------------------------------------------------------------------------
       *  If chip select is "off" the SPI access was interrupted (all SPI access
       *  functions leave chip select in the "off" state).  In this case, turn
       *  back on chip select and break to the main loop.  The main loop will
       *  pick up where the access was interrupted.
       */
      if (CC1101_CS_IS_OFF())
      {
        CC1101_CS_LOW();
        result = TRUE;   // indicate interruption detected
        break;
      }

    /*-------------------------------------------------------------------------------
     */
    } while (len); /* inner loop */
  } while (len);   /* main loop */

  /* turn off chip select; enable interrupts that call SPI functions */
  CC1101_CS_HIGH();
 // MRFI_SPI_EXIT_CRITICAL_SECTION(s);
  
  return result;
}

