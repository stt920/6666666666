/**
  ******************************************************************************
  * @file    stm32_eval_spi_flash.h
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file contains all the functions prototypes for the stm32_eval_spi_flash
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CC1101_SPI_H
#define __CC1101_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "compiler.h" 
#include "stm32_eval.h"


// CC1101 SPI Interface definition

#define CC1101_SPI                       SPI1
#define CC1101_SPI_CLK                   RCC_APB2Periph_SPI1
#define CC1101_SPI_SCK_PIN               GPIO_Pin_5                  /* PA.05 */
#define CC1101_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */
#define CC1101_SPI_SCK_GPIO_CLK          RCC_APB2Periph_GPIOA
#define CC1101_SPI_MISO_PIN              GPIO_Pin_6                  /* PA.06 */
#define CC1101_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */
#define CC1101_SPI_MISO_GPIO_CLK         RCC_APB2Periph_GPIOA
#define CC1101_SPI_MOSI_PIN              GPIO_Pin_7                  /* PA.07 */
#define CC1101_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */
#define CC1101_SPI_MOSI_GPIO_CLK         RCC_APB2Periph_GPIOA

#define GELANRUI_BOARD   //ÊÇ·ñ²ÉÓÃ¸ñÀ¼ÈðµÄ¿ª·¢°å

#ifdef GELANRUI_BOARD

#define CC1101_CS_PIN                     GPIO_Pin_1              
#define CC1101_CS_GPIO_PORT              GPIOB                 
#define CC1101_CS_GPIO_CLK               RCC_APB2Periph_GPIOB 

#define CC1101_SPI_GDO0_PIN              GPIO_Pin_6                
#define CC1101_SPI_GDO0_GPIO_PORT              GPIOC 
#define CC1101_SPI_GDO0_GPIO_CLK               RCC_APB2Periph_GPIOC
#define CC1101_SPI_EXT_PORT_SOURCE               GPIO_PortSourceGPIOC
#define CC1101_SPI_EXT_PIN_SOURCE             GPIO_PinSource6
#define CC1101_SPI_EXT_LINE               EXTI_Line6
#define CC1101_SPI_EXT_CHANNEL        EXTI9_5_IRQn

#else  // CQUPT BOARD
#define CC1101_CS_PIN                     GPIO_Pin_4                  
#define CC1101_CS_GPIO_PORT              GPIOA                   
#define CC1101_CS_GPIO_CLK               RCC_APB2Periph_GPIOA 

#define CC1101_SPI_GDO0_PIN              GPIO_Pin_10                  
#define CC1101_SPI_GDO0_GPIO_PORT              GPIOB  
#define CC1101_SPI_GDO0_GPIO_CLK               RCC_APB2Periph_GPIOB 
// extern interrupt config
// ×¢Òâ:  ¸ü¸ÄGDO0Òý³öµÄÍâ²¿ÖÐ¶ÏÏßÊ±£¬³ýÁË¸ü¸ÄÕâÀïµÄÅäÖÃÍâ£¬»¹ÐèÔÚstm32f10x_it.cÖÐÖØÐÂ¸ü»»ÖÐ¶Ïº¯ÊýÃû.
#define CC1101_SPI_EXT_PORT_SOURCE               GPIO_PortSourceGPIOB 
#define CC1101_SPI_EXT_PIN_SOURCE             GPIO_PinSource10 
#define CC1101_SPI_EXT_LINE               EXTI_Line10
#define CC1101_SPI_EXT_CHANNEL        EXTI15_10_IRQn

#endif

                


#define CC1101_DUMMY_BYTE         0xFF
#define MRFI_READ_BIT                    0x80
#define MRFI_BURST_BIT                   0x40


#define CC1101_CS_LOW()       GPIO_ResetBits(CC1101_CS_GPIO_PORT, CC1101_CS_PIN)
#define CC1101_CS_HIGH()      GPIO_SetBits(CC1101_CS_GPIO_PORT, CC1101_CS_PIN)   
#define CC1101_CS_IS_OFF()    (GPIO_ReadOutputDataBit(CC1101_CS_GPIO_PORT, CC1101_CS_PIN) == Bit_SET)

#define CC1101_CHIP_GDO0_PIN_IS_HIGH()  (GPIO_ReadInputDataBit(CC1101_SPI_GDO0_GPIO_PORT, CC1101_SPI_GDO0_PIN) == Bit_SET)
#define CC1101_CHIP_SO_IS_HIGH()    (GPIO_ReadInputDataBit(CC1101_SPI_MISO_GPIO_PORT, CC1101_SPI_MISO_PIN) == Bit_SET)
#define CC1101_CLEAR_GDO0_INT_FLAG()    (EXTI_ClearITPendingBit(CC1101_SPI_EXT_LINE))
#define CC1101_GDO0_INT_FLAG_IS_SET()     (EXTI_GetITStatus(CC1101_SPI_EXT_LINE) != RESET)


/* configuration registers */
#define IOCFG2      0x00      /*  IOCFG2   - GDO2 output pin configuration  */
#define IOCFG1      0x01      /*  IOCFG1   - GDO1 output pin configuration  */
#define IOCFG0      0x02      /*  IOCFG1   - GDO0 output pin configuration  */
#define FIFOTHR     0x03      /*  FIFOTHR  - RX FIFO and TX FIFO thresholds */
#define SYNC1       0x04      /*  SYNC1    - Sync word, high byte */
#define SYNC0       0x05      /*  SYNC0    - Sync word, low byte */
#define PKTLEN      0x06      /*  PKTLEN   - Packet length */
#define PKTCTRL1    0x07      /*  PKTCTRL1 - Packet automation control */
#define PKTCTRL0    0x08      /*  PKTCTRL0 - Packet automation control */
#define ADDR        0x09      /*  ADDR     - Device address */
#define CHANNR      0x0A      /*  CHANNR   - Channel number */
#define FSCTRL1     0x0B      /*  FSCTRL1  - Frequency synthesizer control */
#define FSCTRL0     0x0C      /*  FSCTRL0  - Frequency synthesizer control */
#define FREQ2       0x0D      /*  FREQ2    - Frequency control word, high byte */
#define FREQ1       0x0E      /*  FREQ1    - Frequency control word, middle byte */
#define FREQ0       0x0F      /*  FREQ0    - Frequency control word, low byte */
#define MDMCFG4     0x10      /*  MDMCFG4  - Modem configuration */
#define MDMCFG3     0x11      /*  MDMCFG3  - Modem configuration */
#define MDMCFG2     0x12      /*  MDMCFG2  - Modem configuration */
#define MDMCFG1     0x13      /*  MDMCFG1  - Modem configuration */
#define MDMCFG0     0x14      /*  MDMCFG0  - Modem configuration */
#define DEVIATN     0x15      /*  DEVIATN  - Modem deviation setting */
#define MCSM2       0x16      /*  MCSM2    - Main Radio Control State Machine configuration */
#define MCSM1       0x17      /*  MCSM1    - Main Radio Control State Machine configuration */
#define MCSM0       0x18      /*  MCSM0    - Main Radio Control State Machine configuration */
#define FOCCFG      0x19      /*  FOCCFG   - Frequency Offset Compensation configuration */
#define BSCFG       0x1A      /*  BSCFG    - Bit Synchronization configuration */
#define AGCCTRL2    0x1B      /*  AGCCTRL2 - AGC control */
#define AGCCTRL1    0x1C      /*  AGCCTRL1 - AGC control */
#define AGCCTRL0    0x1D      /*  AGCCTRL0 - AGC control */
#define WOREVT1     0x1E      /*  WOREVT1  - High byte Event0 timeout */
#define WOREVT0     0x1F      /*  WOREVT0  - Low byte Event0 timeout */
#define WORCTRL     0x20      /*  WORCTRL  - Wake On Radio control */
#define FREND1      0x21      /*  FREND1   - Front end RX configuration */
#define FREND0      0x22      /*  FREDN0   - Front end TX configuration */
#define FSCAL3      0x23      /*  FSCAL3   - Frequency synthesizer calibration */
#define FSCAL2      0x24      /*  FSCAL2   - Frequency synthesizer calibration */
#define FSCAL1      0x25      /*  FSCAL1   - Frequency synthesizer calibration */
#define FSCAL0      0x26      /*  FSCAL0   - Frequency synthesizer calibration */
#define RCCTRL1     0x27      /*  RCCTRL1  - RC oscillator configuration */
#define RCCTRL0     0x28      /*  RCCTRL0  - RC oscillator configuration */
#define FSTEST      0x29      /*  FSTEST   - Frequency synthesizer calibration control */
#define PTEST       0x2A      /*  PTEST    - Production test */
#define AGCTEST     0x2B      /*  AGCTEST  - AGC test */
#define TEST2       0x2C      /*  TEST2    - Various test settings */
#define TEST1       0x2D      /*  TEST1    - Various test settings */
#define TEST0       0x2E      /*  TEST0    - Various test settings */

/* status registers */
#define PARTNUM     0x30      /*  PARTNUM    - Chip ID */
#define VERSION     0x31      /*  VERSION    - Chip ID */
#define FREQEST     0x32      /*  FREQEST    – Frequency Offset Estimate from demodulator */
#define LQI         0x33      /*  LQI        – Demodulator estimate for Link Quality */
#define RSSI        0x34      /*  RSSI       – Received signal strength indication */
#define MARCSTATE   0x35      /*  MARCSTATE  – Main Radio Control State Machine state */
#define WORTIME1    0x36      /*  WORTIME1   – High byte of WOR time */
#define WORTIME0    0x37      /*  WORTIME0   – Low byte of WOR time */
#define PKTSTATUS   0x38      /*  PKTSTATUS  – Current GDOx status and packet status */
#define VCO_VC_DAC  0x39      /*  VCO_VC_DAC – Current setting from PLL calibration module */
#define TXBYTES     0x3A      /*  TXBYTES    – Underflow and number of bytes */
#define RXBYTES     0x3B      /*  RXBYTES    – Overflow and number of bytes */

/* burst write registers */
#define PA_TABLE0   0x3E      /*  PA_TABLE0 - PA control settings table */
#define TXFIFO      0x3F      /*  TXFIFO  - Transmit FIFO */
#define RXFIFO      0x3F      /*  RXFIFO  - Receive FIFO */

/* command strobe registers */
#define SRES        0x30      /*  SRES    - Reset chip. */
#define SFSTXON     0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define SXOFF       0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define SCAL        0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define SRX         0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define STX         0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define SIDLE       0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define SRSVD       0x37      /*  SRVSD   - Reserved.  Do not use. */
#define SWOR        0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define SPWD        0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define SFRX        0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define SFTX        0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define SWORRST     0x3C      /*  SWORRST - Reset real time clock. */
#define SNOP        0x3D      /*  SNOP    - No operation. Returns status byte. */



#define MRFI_SPI_ASSERT(x)      assert_param(x)

#define MRFI_SPI_IS_INITIALIZED()   1


/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
   
uint8_t CC1101_SendByte(uint8_t byte);
uint8_t CC1101_ReadByte(void);

void mrfiSpiInit(void);

uint8_t mrfiSpiCmdStrobe(uint8_t addr);

uint8_t mrfiSpiReadReg(uint8_t addr);
void mrfiSpiWriteReg(uint8_t addr, uint8_t value);

BOOL mrfiSpiWriteTxFifo(uint8_t * pWriteData, uint8_t len);
BOOL mrfiSpiReadRxFifo(uint8_t * pReadData, uint8_t len);

void mrfiEnableExtInterrupt(void);
void mrfiDisableExtInterrupt(void);

#define CC1101_ENABLE_SYNC_PIN_INT()    do { mrfiEnableExtInterrupt(); }  while (0)
#define CC1101_DISABLE_SYNC_PIN_INT()    do { mrfiDisableExtInterrupt(); }  while (0)


#ifdef __cplusplus
}
#endif

#endif /* __CC1101_SPI_H */


