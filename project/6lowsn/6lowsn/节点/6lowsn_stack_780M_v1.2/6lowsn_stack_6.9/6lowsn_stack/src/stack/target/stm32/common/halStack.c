
/******************************************************************************************************
*
* 文 件 名：halStack.c
*
* 文件描述：射频收发与定时器等函数及中断函数
*
* 创 建 者：Wang Heng
*
* 当前版本： 0.50
*
* 修 改 者：
*
* 修改历史：

  2013-05-25  22:38  by Wang Heng
   以WIA-PA CC2530为蓝本，移植到CC2530，上层功能运行成功，
   RF收发和MAC Timer功能进行了大量改进. 


********************************************************************************************************/

#include "compiler.h"
#include "hal.h"
#include "halStack.h"
#include "console.h"
#include "debug.h"
#include "ieee_lrwpan_defs.h"
#include "memalloc.h"
#include "phy.h"
#include "mac.h"
#include <string.h>
#include "cc1101_spi.h"

/***************************pu******************************/
#include "AT86RF212.h"
#include "_STM32F107_Console.h"
UINT16 g_rx_pkt_cnt;
/***************************pu******************************/

#ifndef BV
#define BV(n)      (1 << (n))
#endif

/***************** FOR CC1101 RADIO***********************************************/

#define MRFI_RSSI_OFFSET    74   /* no units */

 /* Worst case wait period in RX state before RSSI becomes valid.
  * These numbers are from Design Note DN505 with added safety margin.
  */
#define MRFI_RSSI_VALID_DELAY_US    1300


/* GDO functionality */
#define MRFI_GDO_SYNC           6
//#define MRFI_GDO_SYNC           1   // only accept RX FIFO interrupts
#define MRFI_GDO_CCA            9
#define MRFI_GDO_PA_PD          27  /* low when transmit is active, low during sleep */
#define MRFI_GDO_LNA_PD         28  /* low when receive is active, low during sleep */

/* ---------- Radio Abstraction ---------- */

#define MRFI_RADIO_PARTNUM          0x00
#define MRFI_RADIO_MIN_VERSION      4


/* GDO0 output pin configuration */
#define MRFI_SETTING_IOCFG0     MRFI_GDO_SYNC
// only accept FIFO RX interrupts.

/* Main Radio Control State Machine control configuration:
 * Auto Calibrate - when going from IDLE to RX/TX
 * PO_TIMEOUT is extracted from SmartRF setting.
 * XOSC is OFF in Sleep state.
 */
#define MRFI_SETTING_MCSM0      (0x10 | (SMARTRF_SETTING_MCSM0 & (BV(2)|BV(3))))

/* Main Radio Control State Machine control configuration:
 * - Remain RX state after RX
 * - Go to IDLE after TX
 * - RSSI below threshold and NOT receiving.
 */
#define MRFI_SETTING_MCSM1      0x3C

/*
 *  Packet Length - Setting for maximum allowed packet length.
 *  The PKTLEN setting does not include the length field but maximum frame size does.
 *  Subtract length field size from maximum frame size to get value for PKTLEN.
 */
///#define MRFI_SETTING_PKTLEN     (MRFI_MAX_FRAME_SIZE - MRFI_LENGTH_FIELD_SIZE)
#define MRFI_SETTING_PKTLEN     0xFF

#define MRFI_SETTING_PKTCTRL0    0x05
// Whitening off, Normal mode, use FIFOs for RX and TX, CRC calculation in TX and CRC check in RX enabled
// Variable packet length mode

//#define MRFI_SETTING_PKTCTRL1  0x0C
#define MRFI_SETTING_PKTCTRL1  0x04
// automatic flush of RX FIFO when CRC in not OK, two status bytes are appended to the payload of the packet
// No address check

/* FIFO threshold - this register has fields that need to be configured for the CC1101 */
#define MRFI_SETTING_FIFOTHR    0x0E
/*
 * Maximum tramsmit time is calculated from the data rate times the maximum
 * number of bits that can be transmitted.  The data rate is calculated from
 * the DRATE_M and DRATE_E fields of the MDMCFG3 and MDMCFG4 registers
 * respectively.
 */
#ifndef MRFI_XTAL_FREQ_Hz
#define MRFI_XTAL_FREQ_Hz 26000000L
#endif
#define MRFI_SYNC_WORD_SIZE ( ( (SMARTRF_SETTING_MDMCFG2 & 0x3) == 0 ) ? 0 : \
                            ( ( (SMARTRF_SETTING_MDMCFG2 & 0x3) != 3 ) ? 2 : 4 ) )
#define MRFI_PREAMBLE_SIZE ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x00 ) ?  2 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x10 ) ?  3 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x20 ) ?  4 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x30 ) ?  6 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x40 ) ?  8 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x50 ) ? 12 : \
                           ( ( ( SMARTRF_SETTING_MDMCFG1 & 0x70 ) == 0x60 ) ? 16 : \
                                                                              24 ) ) ) ) ) ) )
#define MRFI_MAX_TRANSMIT_BYTES ( MRFI_MAX_FRAME_SIZE + MRFI_SYNC_WORD_SIZE + MRFI_PREAMBLE_SIZE )
#define MRFI_TRANSMIT_BIT_PERIOD_us (1000000.0*(1L<<28)/((SMARTRF_SETTING_MDMCFG3+256.0)*MRFI_XTAL_FREQ_Hz*(1L<<(SMARTRF_SETTING_MDMCFG4 & 0xF))))
#define MRFI_MAX_TRANSMIT_TIME_us ((long)( MRFI_TRANSMIT_BIT_PERIOD_us * 8 * MRFI_MAX_TRANSMIT_BYTES + 500 ))

#define MRFI_PKTSTATUS_CCA BV(4)
#define MRFI_PKTSTATUS_CS  BV(6)

  /* The SW timer is calibrated by adjusting the call to the microsecond delay
   * routine. This allows maximum calibration control with repects to the longer
   * times requested by applicationsd and decouples internal from external calls
   * to the microsecond routine which can be calibrated independently.
   */

#define APP_USEC_VALUE    1000

#define MRFI_SPI_DRIVE_CSN_LOW()        CC1101_CS_LOW()
#define MRFI_SPI_DRIVE_CSN_HIGH()      CC1101_CS_HIGH()
#define MRFI_SPI_CSN_IS_HIGH()            CC1101_CS_IS_OFF()
#define MRFI_SPI_SO_IS_HIGH()               CC1101_CHIP_SO_IS_HIGH()

#define MRFI_SYNC_PIN_IS_HIGH()                     CC1101_CHIP_GDO0_PIN_IS_HIGH()
#define MRFI_CLEAR_SYNC_PIN_INT_FLAG()              CC1101_CLEAR_GDO0_INT_FLAG()
#define MRFI_ENABLE_SYNC_PIN_INT()                 CC1101_ENABLE_SYNC_PIN_INT()
#define MRFI_DISABLE_SYNC_PIN_INT()                 CC1101_DISABLE_SYNC_PIN_INT()
#define MRFI_SYNC_PIN_INT_FLAG_IS_SET()             CC1101_GDO0_INT_FLAG_IS_SET()

#define MRFI_PAPD_PIN_IS_HIGH()                     MRFI_SYNC_PIN_IS_HIGH()
#define MRFI_CLEAR_PAPD_PIN_INT_FLAG()              MRFI_CLEAR_SYNC_PIN_INT_FLAG()
#define MRFI_PAPD_INT_FLAG_IS_SET()                 MRFI_SYNC_PIN_INT_FLAG_IS_SET()

#define MRFI_CONFIG_GDO0_AS_PAPD_SIGNAL()           mrfiSpiWriteReg(IOCFG0, MRFI_GDO_PA_PD)
#define MRFI_CONFIG_GDO0_AS_SYNC_SIGNAL()           mrfiSpiWriteReg(IOCFG0, MRFI_GDO_SYNC)

#define MRFI_ASSERT(x)        assert_param(x)

#ifdef MRFI_ASSERTS_ARE_ON
#define RX_FILTER_ADDR_INITIAL_VALUE  0xFF
#endif

/* Packet automation control - base value is power up value whick has APPEND_STATUS enabled; no CRC autoflush */
#define PKTCTRL1_BASE_VALUE         BV(2)
#define PKTCTRL1_ADDR_FILTER_OFF    PKTCTRL1_BASE_VALUE
#define PKTCTRL1_ADDR_FILTER_ON     (PKTCTRL1_BASE_VALUE | (BV(0)|BV(1)))

#define MRFI_WRITE_REGISTER(x,y)      mrfiSpiWriteReg( x, y )

/* There is no bit in h/w to tell if RSSI in the register is valid or not.
 * The hardware needs to be in RX state for a certain amount of time before
 * a valid RSSI value is calculated and placed in the register. This min
 * wait time is defined by MRFI_BOARD_RSSI_VALID_DELAY_US. We don't need to
 * add such delay every time RSSI value is needed. If the Carier Sense signal
 * is high or CCA signal is high, we know that the RSSI value must be valid.
 * We use that knowledge to reduce our wait time. We break down the delay loop
 * in multiple chunks and during each iteration, check for the CS and CCA
 * signal. If either of these signals is high, we return immediately. Else,
 * we wait for the max delay specified.
 */
#define MRFI_RSSI_VALID_WAIT()                                                \
{                                                                             \
  int16_t delay = MRFI_RSSI_VALID_DELAY_US;                                   \
  do                                                                          \
  {                                                                           \
    if(mrfiSpiReadReg(PKTSTATUS) & (MRFI_PKTSTATUS_CCA | MRFI_PKTSTATUS_CS))  \
    {                                                                         \
      break;                                                                  \
    }                                                                         \
    Mrfi_DelayUsec(64); /* sleep */                                           \
    delay -= 64;                                                              \
  }while(delay > 0);                                                          \
}                                                                             \


#define MRFI_STROBE_IDLE_AND_WAIT()                   \
{                                                     \
  mrfiSpiCmdStrobe( SIDLE );                          \
  while (mrfiSpiCmdStrobe( SNOP ) & 0xF0) ;           \
}



RADIO_FLAGS local_radio_flags;

uint8_t mac_timer_upper_byte;  // 模拟MAC Timer的高字节，只有低4位有效，加上TIM2的16位, 构成20位的模拟MAC Timer.

BYTE volatile long_address[8];

#ifdef  LOWSN_ASYNC_INTIO
BYTE serio_rxBuff[LOWSN_ASYNC_RX_BUFSIZE];
BYTE serio_rxHead, serio_rxTail;
#endif

// 射频状态变量:  该变量应该归入PHY PIB或者local_radio_flags封装体中，但考虑对各芯片的兼容性，
// 目前暂时使用全局变量来操作.
static uint8_t mrfiRadioState  = MRFI_RADIO_STATE_UNKNOWN;

/* reply delay support */
static          uint16_t sReplyDelayScalar = 0;
static          uint16_t sBackoffHelper = 0;

// 信道基准为433MHz，信道逻辑号等于433MHz+逻辑号* 200KHz，逻辑好从0到8共9个信道号。
static const uint8_t mrfiLogicalChanTable[] =
{
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8
};

static const uint8_t mrfiRFPowerTable[] =
{
  0x0F,
  0x27,
  0x50
};

static const uint8_t mrfiRadioCfg[][2] =
{
  /* internal radio configuration */
  {  IOCFG0,    MRFI_SETTING_IOCFG0       },
  
  {  MCSM1,     MRFI_SETTING_MCSM1        }, /* CCA mode, RX_OFF_MODE and TX_OFF_MODE */
  {  MCSM0,     MRFI_SETTING_MCSM0        }, /* AUTO_CAL and XOSC state in sleep */
  
  {  PKTLEN,    MRFI_SETTING_PKTLEN       },
  {  PKTCTRL0,  MRFI_SETTING_PKTCTRL0     },
  {  PKTCTRL1,  MRFI_SETTING_PKTCTRL1     },
  {  FIFOTHR,   MRFI_SETTING_FIFOTHR      },

  /* imported SmartRF radio configuration */
  {  FSCTRL1,   SMARTRF_SETTING_FSCTRL1   },
  {  FSCTRL0,   SMARTRF_SETTING_FSCTRL0   },
  {  FREQ2,     SMARTRF_SETTING_FREQ2     },
  {  FREQ1,     SMARTRF_SETTING_FREQ1     },
  {  FREQ0,     SMARTRF_SETTING_FREQ0     },
  
  {  MDMCFG4,   SMARTRF_SETTING_MDMCFG4   },
  {  MDMCFG3,   SMARTRF_SETTING_MDMCFG3   },
  {  MDMCFG2,   SMARTRF_SETTING_MDMCFG2   },
  {  MDMCFG1,   SMARTRF_SETTING_MDMCFG1   },
  {  MDMCFG0,   SMARTRF_SETTING_MDMCFG0   },
  {  DEVIATN,   SMARTRF_SETTING_DEVIATN   },
  
  {  FOCCFG,    SMARTRF_SETTING_FOCCFG    },
  {  BSCFG,     SMARTRF_SETTING_BSCFG     },
  {  AGCCTRL2,  SMARTRF_SETTING_AGCCTRL2  },
  {  AGCCTRL1,  SMARTRF_SETTING_AGCCTRL1  },
  {  AGCCTRL0,  SMARTRF_SETTING_AGCCTRL0  },
  {  FREND1,    SMARTRF_SETTING_FREND1    },
  {  FREND0,    SMARTRF_SETTING_FREND0    },
  {  FSCAL3,    SMARTRF_SETTING_FSCAL3    },
  {  FSCAL2,    SMARTRF_SETTING_FSCAL2    },
  {  FSCAL1,    SMARTRF_SETTING_FSCAL1    },
  {  FSCAL0,    SMARTRF_SETTING_FSCAL0    },
  {  TEST2,     SMARTRF_SETTING_TEST2     },
  {  TEST1,     SMARTRF_SETTING_TEST1     },
  {  TEST0,     SMARTRF_SETTING_TEST0     },
};


static int8_t Mrfi_CalculateRssi(uint8_t rawValue);
static BOOL mrfi_TxCCADone( void );
static void Mrfi_RxModeOn(void);
static void Mrfi_RxModeOff(void);
static void Mrfi_RandomBackoffDelay(void);
static void Mrfi_DelayUsec(uint16_t);


void halInit(void)
{
  //Set clock source
  local_radio_flags.val = 0;
  //SET_MAIN_CLOCK_SOURCE(CRYSTAL);
    /* Configure the NVIC Preemption Priority Bits */  
  // 设置抢占优先级从0-1，响应优先级从0到7，将系统定时器的优先级设定为最高
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  halInitUart();
  halInitMACTimer();
  halInitLed();
  halInitRFChip();      //pu 2014.2.17
}

#if 0
//initialize UART to be used by
//this is usart2
void halInitUart(void)
{
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE );
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef   NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
   
     /*配置Tx-------------------*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
     /*配置Rx-------------------*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    /* Enable the USART2 Pins Software Remapping */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    serio_rxHead = 0;
    serio_rxTail = 0;
    
    /* USART_InitStruct members default value */
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No ;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    
    USART_Init(USART2, &USART_InitStruct);
    /* Disable USARTy Receive and Transmit interrupts */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_Cmd(USART2,ENABLE);
    
    USART_ClearFlag(USART2, USART_FLAG_TC);     // 清标志
}
#else

void halInitUart(void) 
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
 /* System Clocks Configuration */
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
  /* Enable USART1 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

  #ifdef LOWSN_ASYNC_INTIO       
  /* NVIC configuration */
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  serio_rxHead = 0;
  serio_rxTail = 0;
  #endif

  /* Configure the GPIO ports */
  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART1 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);

  #ifdef LOWSN_ASYNC_INTIO   
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   #endif

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);

}
#endif

//write a character to serial port
// Uses UART initialized by halInitUart

void halPutch(char c)
{

    USART_SendData(USART1, (int8_t)c);
	
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
	
}

void halRawPut(char c) 
{

    USART_SendData(USART1, (int8_t)c);
	
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }

    
	
}


#ifdef  LOWSN_ASYNC_INTIO

//get a character from serial port, uses interrupt driven IO
char halGetch(void)
{
    char x;
    do {
      x = serio_rxHead;  //use tmp because of volt decl
    }  while(serio_rxTail == x);
    serio_rxTail++;
   if (serio_rxTail == LOWSN_ASYNC_RX_BUFSIZE) serio_rxTail = 0;
   return (serio_rxBuff[serio_rxTail]);
}

 BOOL halGetchRdy(void)
 {
   char x;
   x = serio_rxHead;
   return(serio_rxTail != x);
 }

#else
//get a character from serial port
char halGetch(void)
{
   char c;

    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {
    }

    c = (USART_ReceiveData(USART1) & 0xFF);  

    return c;
	
}

BOOL halGetchRdy(void)
{

  if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) return (1);
  else return(0);

}

#endif


void halUtilMemCopy(BYTE *dst, BYTE *src, BYTE len) 
{
  while (len) {
    *dst = *src;
    dst++;src++;
    len--;
  }
}


/********************************************************
use TIM2 as a MAC timer,  the base clock period of TIM2 is set to 16us
Update_event = TIM_CLK/((PSC + 1)*(ARR + 1)*(RCR + 1))
TIM_CLK = timer clock input; PSC = 16-bit prescaler register; 
ARR = 16/32-bit Autoreload register; RCR = 16-bit repetition counter
base clocl period = (Prescaler + 1) / TIM_CLK

TIM2的特殊性质:
for all cases other than APB prescaler of 1 the TIMCLK is APB *2.
APB = CLK / 1; TIM = CLK;
APB = CLK / 2; TIM = CLK;
APB = CLK / 4; TIM = CLK / 2;

Thus even with an APB1 clock of 36 MHz, the timers attached run at 72 MHz
当APB1对系统时钟进行1分频时，TIM2时钟不倍频，但APB1最高时钟是36MHz，所以至少2分频。
当APB1对系统时钟进行2、4、8分频时，TIM2对APB1分频后的时钟再次进行2倍倍频。
初始化时APB1是系统时钟的4分频，之后TIM2 自动倍频，故TIM2输入时钟为36MHz。
TIM2最小时钟粒度= TIM_CLK/(Prescaler + 1), TIM_CLK初始化为32MHz，即SystemCoreClock/2
PrescalerValue = (uint16_t) (SystemCoreClock * 8 / 1000000) - 1。
ARR = 65535; RCR=0;
every overflow period: 16us*65536=1.048576s
******************************************************/

void halInitMACTimer(void) 
{

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  uint16_t PrescalerValue;

  /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div4);
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* GPIOC clock enable */
 // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);


  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock * 8 / 1000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

  #ifdef LOWSN_ENABLE_SLOW_TIMER

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
   // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_VAL;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

  #endif

  /* TIM IT enable */
  #ifdef LOWSN_ENABLE_SLOW_TIMER
  TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, ENABLE);
  #else
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  #endif

  mac_timer_upper_byte = 0;

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);

}

// MAC Timer为模拟的20位Timer，由TIM2的16位作为低16位，mac_timer_upper_byte变量的
// 低4位作为模拟Timer的最高位，当TIM2溢出时，中断中对mac_timer_upper_byte加1，
// 当mac_timer_upper_byte超过0x0F时，重置为0.

UINT32 halGetMACTimer(void) 
{
  UINT32 t;
  //BOOL gie_status;

  //SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
  t = TIM_GetCounter(TIM2);
  t += ((((UINT32) mac_timer_upper_byte) & 0x000000FF) << 16);
  //RESTORE_GLOBAL_INTERRUPT(gie_status);
  return (t & 0x00FFFFFF);
}

#ifdef LOWSN_COMPILER_NO_RECURSION
UINT32 halISRGetMACTimer(void)
{
  UINT32 t;
  //BOOL gie_status;

  //SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
  t = TIM_GetCounter(TIM2);
  t += (((UINT32) mac_timer_upper_byte & 0xFF)<<16);
  //RESTORE_GLOBAL_INTERRUPT(gie_status);
  return (t & 0x00FFFFFF);
}

#endif


//only works as long as SYMBOLS_PER_MAC_TICK is not less than 1
UINT32 halMacTicksToUs(UINT32 ticks){

   UINT32 rval;

   rval =  (ticks/SYMBOLS_PER_MAC_TICK())* (1000000/LOWSN_SYMBOLS_PER_SECOND);
   return(rval);
}

//STM32 hava a random number generator, here we use TIM2 lower bytes instead temporarily.
UINT8 halGetRandomByte(void) {
  return((UINT8)(TIM_GetCounter(TIM2) & 0x00FF));
}




//set the radio frequency, channel should be 0,1,2,3 in 433MHz, the actural channel are 20, 50,80, 110, respectively.
LOWSN_STATUS_ENUM halSetRadioIEEEFrequency(PHY_FREQ_ENUM frequency, BYTE channel)
{
   if (frequency != PHY_FREQ_780M) return(LOWSN_STATUS_PHY_FAILED);
   if (channel >= MRFI_NUM_LOGICAL_CHANS) return(LOWSN_STATUS_PHY_FAILED);

    /* make sure radio is off before changing channels */
    //Mrfi_RxModeOff();

    //MRFI_WRITE_REGISTER( CHANNR, mrfiLogicalChanTable[channel] );

    /* turn radio back on if it was on before channel change */
    //if(mrfiRadioState == MRFI_RADIO_STATE_RX)
   // {
  //   Mrfi_RxModeOn();
  //  }
        trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);	// Set to TRX_OFF
        while((trx_reg_read(RG_TRX_STATUS)&0x0f) != TRX_OFF)	// Check if it is TRX_OFF  
        {
          trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
        }
        trx_reg_write(RG_CC_CTRL_0,GET_CHINA_FREQ(channel));            //信道选择（CC_NUMBER），中心频率
        trx_reg_write(RG_CC_CTRL_1,0x04);            //信道选择（CC_NUMBER），中心频率

	RF212_Rx_Mode();
  
   return(LOWSN_STATUS_SUCCESS);
}


//this assumes 2.4GHz frequency
LOWSN_STATUS_ENUM halSetChannel(BYTE channel){
 return(halSetRadioIEEEFrequency(PHY_FREQ_780M, channel));
}


void halSetPowerLevel(uint8_t idx)
{
  /* is power level specified valid? */
  MRFI_ASSERT( idx < MRFI_NUM_POWER_SETTINGS );

  /* make sure radio is off before changing power levels */
  Mrfi_RxModeOff();

  MRFI_WRITE_REGISTER( PA_TABLE0, mrfiRFPowerTable[idx] );

  /* turn radio back on if it was on before power level change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }
}


// 为了调试方便，使用自己设定的EUI64地址

void halGetProcessorIEEEAddress(BYTE *buf) 
{
	buf[0] = aExtendedAddress_B0;
	buf[1] = aExtendedAddress_B1;
	buf[2] = aExtendedAddress_B2;
	buf[3] = aExtendedAddress_B3;
	buf[4] = aExtendedAddress_B4;
	buf[5] = aExtendedAddress_B5;
	buf[6] = aExtendedAddress_B6;
	buf[7] = aExtendedAddress_B7;

}


// 暂时为了MAC层长地址按反阅读顺序发送而进行设定，以后需要统一处理长地址
void halGetProcessorIEEEAddress_MAC(BYTE *buf) 
{

	buf[0] = aExtendedAddress_B7;
	buf[1] = aExtendedAddress_B6;
	buf[2] = aExtendedAddress_B5;
	buf[3] = aExtendedAddress_B4;
	buf[4] = aExtendedAddress_B3;
	buf[5] = aExtendedAddress_B2;
	buf[6] = aExtendedAddress_B1;
	buf[7] = aExtendedAddress_B0;
}



// 在编译器参数中配置的aExtendedAddress_B0-B7是按大端格式存储的(自己设定的)
// 这里存储到芯片中时需要反转
// 芯片中存储的是小端格式

void halSetRadioIEEEAddress(void)
{

}


void halSetRadioPANID(UINT16 panid)
{
   // Do not use address filter in CC1101, leave it for comparity.
 }

void halSetRadioShortAddr(SADDR saddr)
{
   // Do not use address filter in CC1101, leave it for comparity.
}


// 初始化射频函数芯片本身，包括SPI接口、复位、默认配置等。
// 该函数执行后，射频芯片才能被SPI口读写，但射频芯片并不处于接收态，
// 而是IDLE态，对射频性能的初始化在halInitRadio()函数中完成。
// 此外，射频中断也未启动，在halInitRadio()函数的结束部分才启用。
#if 0
void halInitRFChip(void)
{

  mrfiSpiInit();

  /* ------------------------------------------------------------------
   *    Radio power-up reset
   *   ----------------------
   */
  MRFI_ASSERT(MRFI_SPI_CSN_IS_HIGH());

  /* pulse CSn low then high */
  CC1101_CS_LOW();
  Mrfi_DelayUsec(16);
  CC1101_CS_HIGH();

  /* hold CSn high for at least 40 microseconds */
  Mrfi_DelayUsec(48);

  /* pull CSn low and wait for SO to go low */
  CC1101_CS_LOW();
  while (MRFI_SPI_SO_IS_HIGH());

  /* directly send strobe command - cannot use function as it affects CSn pin */
  CC1101_SendByte(SRES);

  /* wait for SO to go low again, reset is complete at that point */
  while (MRFI_SPI_SO_IS_HIGH());

  /* return CSn pin to its default high level */
  CC1101_CS_HIGH();

  // Run-time integrity checks
  /* verify that SPI is working, PKTLEN is an arbitrary read/write register used for testing */
  //#ifdef MRFI_ASSERTS_ARE_ON 
  #define TEST_VALUE 0x12
  mrfiSpiWriteReg(PKTLEN, TEST_VALUE);
  //MRFI_ASSERT( mrfiSpiReadReg( PKTLEN ) == TEST_VALUE ); /* SPI is not responding */
  if (mrfiSpiReadReg(PKTLEN) == TEST_VALUE)
  {
	DEBUG_STRING(DBG_INFO, "CC1101: SPI TEST OK! \n");
  }
  else
  {
	DEBUG_STRING(DBG_INFO, "CC1101: SPI TEST FAILED!Write 0x12, read: ");
      DEBUG_UINT8(DBG_INFO, mrfiSpiReadReg(PKTLEN));
      DEBUG_STRING(DBG_INFO, "\n");
  }
  
  //#endif
  /* verify the correct radio is installed */
  MRFI_ASSERT( mrfiSpiReadReg( PARTNUM ) == MRFI_RADIO_PARTNUM );      /* incorrect radio specified */
  MRFI_ASSERT( mrfiSpiReadReg( VERSION ) >= MRFI_RADIO_MIN_VERSION );  /* obsolete radio specified  */

  /* initialize radio registers */
  {
    uint8_t i;

    for (i=0; i<(sizeof(mrfiRadioCfg)/sizeof(mrfiRadioCfg[0])); i++)
    {
      mrfiSpiWriteReg(mrfiRadioCfg[i][0], mrfiRadioCfg[i][1]);
    }
  }

  /* Initial radio state is IDLE state */
  // 确保IDLE态，重置一次
  MRFI_STROBE_IDLE_AND_WAIT();
  mrfiRadioState = MRFI_RADIO_STATE_IDLE;

}
#endif
void halInitRFChip(void)
{
//  STM32F107_RF212_PortInit();
  RF212_PortInit();
  SPI1_GPIO_Configuration();
  SPI1_Configuration_Configuration();
  
  AT86RF212_Init();
  RF212_EXTI_Init();   //上升沿触发
  //RF_Inter_Confg();    //定时器
  //STM_TIM_BaseInit();  
}

LOWSN_STATUS_ENUM halInitRadio(PHY_FREQ_ENUM frequency, BYTE channel, RADIO_FLAGS radio_flags)
{

  LOWSN_STATUS_ENUM status;
  
  AT86RF212_Init();
  
  // Setting the frequency
  status = halSetRadioIEEEFrequency(frequency, channel);  //pu
 // status = LOWSN_STATUS_SUCCESS;
  if (status != LOWSN_STATUS_SUCCESS) 
  {
       DEBUG_STRING(DBG_INFO, "AT86RF212: Setting channel failed. \n");
  	return(status);
  }
    
    RF212_Rx_Mode();  
  
  mrfiRadioState = MRFI_RADIO_STATE_RX;

   local_radio_flags = radio_flags;
   halSetRadioIEEEAddress();
 
   //2017.4.24
  DEBUG_STRING(DBG_INFO, "AT86RF212: Radio initialization is done. \n");
  
   return(LOWSN_STATUS_SUCCESS);
}

LOWSN_STATUS_ENUM halInitRadio1(PHY_FREQ_ENUM frequency, BYTE channel, RADIO_FLAGS radio_flags)
{

  LOWSN_STATUS_ENUM status;

  // Setting the frequency
  status = halSetRadioIEEEFrequency(frequency, channel);
  if (status != LOWSN_STATUS_SUCCESS) 
  {
       DEBUG_STRING(DBG_INFO, "CC1101: Setting channel failed. \n");
  	return(status);
  }

  /* set default power */
  halSetPowerLevel(MRFI_NUM_POWER_SETTINGS - 1);

   /* flush the receive FIFO of any residual data */
  mrfiSpiCmdStrobe( SFRX );
  /* Put the radio in RX state */
  mrfiSpiCmdStrobe( SRX );
  
  mrfiRadioState = MRFI_RADIO_STATE_RX;
 
  /* delay for the rssi to be valid */
  MRFI_RSSI_VALID_WAIT();
  
   
  /* Turn off RF. */
  //Mrfi_RxModeOff();


  /*****************************************************************************************
   *                            Compute reply delay scalar
   *
   * Formula from data sheet for all the narrow band radios is:
   *
   *                (256 + DATAR_Mantissa) * 2^(DATAR_Exponent)
   * DATA_RATE =    ------------------------------------------ * f(xosc)
   *                                    2^28
   *
   * To try and keep some accuracy we change the exponent of the denominator
   * to (28 - (exponent from the configuration register)) so we do a division
   * by a smaller number. We find the power of 2 by shifting.
   *
   * The maximum delay needed depends on the MAX_APP_PAYLOAD parameter. Figure
   * out how many bits that will be when overhead is included. Bits/bits-per-second
   * is seconds to transmit (or receive) the maximum frame. We multiply this number
   * by 1000 to find the time in milliseconds. We then additionally multiply by
   * 10 so we can add 5 and divide by 10 later, thus rounding up to the number of
   * milliseconds. This last won't matter for slow transmissions but for faster ones
   * we want to err on the side of being conservative and making sure the radio is on
   * to receive the reply. The semaphore monitor will shut it down. The delay adds in
   * a platform fudge factor that includes processing time on peer plus lags in Rx and
   * processing time on receiver's side. Also includes round trip delays from CCA
   * retries. This portion is included in PLATFORM_FACTOR_CONSTANT defined in mrfi.h.
   *
   * Note that we assume a 26 MHz clock for the radio...
   * ***************************************************************************************
   */
#define   MRFI_RADIO_OSC_FREQ        26000000
#define   PHY_PREAMBLE_SYNC_BYTES    8

  {
    uint32_t dataRate, bits;
    uint16_t exponent, mantissa;

    /* mantissa is in MDMCFG3 */
    mantissa = 256 + SMARTRF_SETTING_MDMCFG3;

    /* exponent is lower nibble of MDMCFG4. */
    exponent = 28 - (SMARTRF_SETTING_MDMCFG4 & 0x0F);

    /* we can now get data rate */
    dataRate = mantissa * (MRFI_RADIO_OSC_FREQ>>exponent);

    bits = ((uint32_t)((PHY_PREAMBLE_SYNC_BYTES + MRFI_MAX_FRAME_SIZE)*8))*10000;

    /* processing on the peer + the Tx/Rx time plus more */
    sReplyDelayScalar = PLATFORM_FACTOR_CONSTANT + (((bits/dataRate)+5)/10);

    /* This helper value is used to scale the backoffs during CCA. At very
     * low data rates we need to backoff longer to prevent continual sampling
     * of valid frames which take longer to send at lower rates. Use the scalar
     * we just calculated divided by 32. With the backoff algorithm backing
     * off up to 16 periods this will result in waiting up to about 1/2 the total
     * scalar value. For high data rates this does not contribute at all. Value
     * is in microseconds.
     */
     sBackoffHelper = MRFI_BACKOFF_PERIOD_USECS + (sReplyDelayScalar>>5)*1000;
  }

   local_radio_flags = radio_flags;
   halSetRadioIEEEAddress();
 
   /*  Configure and enable the SYNC signal interrupt.
   *
   *  This interrupt is used to indicate receive.  The SYNC signal goes
   *  high when a receive OR a transmit begins.  It goes high once the
   *  sync word is receved or transmitted and then goes low again once
   *  the packet completes.
   */
   
  MRFI_CONFIG_GDO0_AS_SYNC_SIGNAL();
  CC1101_ENABLE_SYNC_PIN_INT(); 
  MRFI_CLEAR_SYNC_PIN_INT_FLAG();
  /* enable global interrupts */
  //BSP_ENABLE_INTERRUPTS();

  DEBUG_STRING(DBG_INFO, "CC1101: Radio initialization is done. \n");
  
   return(LOWSN_STATUS_SUCCESS);
}


BOOL mrfi_TxImmediateDone( void )
{
  return MRFI_SYNC_PIN_INT_FLAG_IS_SET(); // || marc_state == RF_SM_RX || marc_state == RF_SM_IDLE;
}


/**************************************************************************************************
 * @fn          mrfi_TxCCADone
 *
 * @brief       Indicates status of transmission completion
 *
 * @param       None
 *
 * @return      TRUE if transmission is complete, FALSE otherwise
 **************************************************************************************************
 */
BOOL mrfi_TxCCADone( void )
{
  return MRFI_PAPD_PIN_IS_HIGH(); // || marc_state == RF_SM_RX || marc_state == RF_SM_IDLE;
}

/*********************************************************************************************
@function:

regardless of what happens here, we will try TXONCCA after this returns.
**********************************************************************************************/
BOOL  doIEEE_backoff(void)
{
  BYTE be, nb, tmp, rannum;
  UINT32  delay, start_tick;

  be = aMinBE;
  nb = 0;
  do {
    if (be) {
      //do random delay
      tmp = be;
      //compute new delay
      delay = 1;
      while (tmp) {
        delay = delay << 1;  //delay = 2**be;
        tmp--;
      }
      rannum =  halGetRandomByte() & (delay-1); //rannum will be between 0 and delay-1
      delay = 0;
      while (rannum) {
        delay  += SYMBOLS_TO_MACTICKS(aUnitBackoffPeriod);
        rannum--;
      }//delay = aUnitBackoff * rannum
      //now do backoff
      start_tick = halGetMACTimer();
      while (halMACTimerNowDelta(start_tick) < delay);
    }
    //check CCA
    //if (RF212_CCA_Perform())  break;
    if (RF212_CCA_Perform())  return 1;
    nb++;
    be++;
    if (be > aMaxBE) be =aMaxBE;
  }while (nb <= macMaxCSMABackoffs);
  return 0;
}

LOWSN_STATUS_ENUM halSendPacket(BYTE flen, BYTE *frm)
{
  LOWSN_STATUS_ENUM res = LOWSN_STATUS_SUCCESS;
  uint8_t ccaRetries;
  uint8_t len;

  //if you print out the packet, this can cause timeouts on waits
  //dbgPrintPacket(frm,flen+2);

  //上层传来的flen，不包括第一个长度字节，也不包括最后两个字节的CRC域
  // 在添加位于首部的长度字节时，若按IEEE 802.15.4标准，该字节的值等于其后负载字节数加上两个CRC字节数
  // 但按照CC1101的数据手册，该值应等于其后字节数，不包括CRC的两个字节。采用CC1101定义，否则接收时
  // 对字节的处理会出现混乱。

  len = flen;

  #if 1
  DEBUG_STRING(DBG_INFO, "AT86RF212: TX pkt Size: ");
  DEBUG_UINT8(DBG_INFO, flen);
  DEBUG_STRING(DBG_INFO,"\n");
  #endif
  
  if (flen > MRFI_MAX_FRAME_SIZE) {
    //packet size is too large!
    return(LOWSN_STATUS_PHY_TX_PKT_TOO_BIG);
  }

  RF212_Tx_Mode();
#if 1
  
   while(((trx_reg_read(RG_TRX_STATUS))&0x0f) != 0x09)	    // wait until PLL_ON
   {
       trx_bit_write(SR_TRX_CMD, CMD_PLL_ON);	
   }
      
  //  trx_frame_write (len+2,frm);                 //帧缓冲区写入
   if( doIEEE_backoff() )
   {
     //
	RF212_Tx_Mode();
	trx_frame_write (len+2,frm);                 //帧缓冲区写入 
	trx_bit_write(SR_TRX_CMD, CMD_TX_START);		// Start TX

	while((trx_reg_read(RG_TRX_STATUS)&0x0f) != 0x09);//PLL_ON);	// Wait for Tx end
        //  Rf231_SendPacket();    
        //STM32F107_Led_Toggle(2);      
	DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_TXFIN );
	phyTxEndCallBack();
	macTxCallback();
	res = LOWSN_STATUS_SUCCESS;
   }
   else
   {
        res = LOWSN_STATUS_PHY_CHANNEL_BUSY;
   }
   goto test_point;

#endif

  test_point: 
  halLedToggle(0);  
  RF212_Rx_Mode();
  return(res); 
}


LOWSN_STATUS_ENUM halSendPacket1(BYTE flen, BYTE *frm)
{
  LOWSN_STATUS_ENUM res = LOWSN_STATUS_SUCCESS;
  uint8_t ccaRetries;
  uint8_t len;

  //if you print out the packet, this can cause timeouts on waits
  //dbgPrintPacket(frm,flen+2);

  //上层传来的flen，不包括第一个长度字节，也不包括最后两个字节的CRC域
  // 在添加位于首部的长度字节时，若按IEEE 802.15.4标准，该字节的值等于其后负载字节数加上两个CRC字节数
  // 但按照CC1101的数据手册，该值应等于其后字节数，不包括CRC的两个字节。采用CC1101定义，否则接收时
  // 对字节的处理会出现混乱。

  len = flen;

  #if 0
  DEBUG_STRING(DBG_INFO, "CC1101: TX pkt Size: ");
  DEBUG_UINT8(DBG_INFO, flen);
  DEBUG_STRING(DBG_INFO,"\n");
  #endif
  
  if (flen > MRFI_MAX_FRAME_SIZE) {
    //packet size is too large!
    return(LOWSN_STATUS_PHY_TX_PKT_TOO_BIG);
  }

 /* radio must be awake to transmit */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* Turn off reciever. We can ignore/drop incoming packets during transmit. */
  Mrfi_RxModeOff();
  
  mrfiSpiCmdStrobe( SFTX ); // flush the tx fifo

  mrfiSpiWriteTxFifo(&len, 1);
  mrfiSpiWriteTxFifo(frm, flen);
  
  #if 0
  // 直接发送，不进行CCA检测和CSMA回退
  // Issue the TX strobe. 
    mrfiSpiCmdStrobe( STX );

    //Wait for transmit to complete 
    Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                        MRFI_MAX_TRANSMIT_TIME_us % 1000,
                        mrfi_TxImmediateDone );

    // Clear the interrupt flag 
    MRFI_CLEAR_SYNC_PIN_INT_FLAG();

	 DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_TXFIN );
        phyTxEndCallBack();
        macTxCallback();
   goto test_point;

#endif


    /* set number of CCA retries */
    ccaRetries = MRFI_CCA_RETRIES;

    /* For CCA algorithm, we need to know the transition from the RX state to
     * the TX state. There is no need for SYNC signal in this logic. So we
     * can re-configure the GDO_0 output from the radio to be PA_PD signal
     * instead of the SYNC signal.
     * Since both SYNC and PA_PD are used as falling edge interrupts, we
     * don't need to reconfigure the MCU input.
     */
    MRFI_CONFIG_GDO0_AS_PAPD_SIGNAL();

    /* ===============================================================================
     *    Main Loop
     *  =============
     */
    for (;;)
    {
      /* Radio must be in RX mode for CCA to happen.
       * Otherwise it will transmit without CCA happening.
       */

      /* Can not use the Mrfi_RxModeOn() function here since it turns on the
       * Rx interrupt, which we don't want in this case.
       */
      mrfiSpiCmdStrobe( SRX );

      /* wait for the rssi to be valid. */
      MRFI_RSSI_VALID_WAIT();
	 
      /*
       *  Clear the PA_PD pin interrupt flag.  This flag, not the interrupt itself,
       *  is used to capture the transition that indicates a transmit was started.
       *  The pin level cannot be used to indicate transmit success as timing may
       *  prevent the transition from being detected.  The interrupt latch captures
       *  the event regardless of timing.
       */
      MRFI_CLEAR_PAPD_PIN_INT_FLAG();

      /* Issue the TX strobe. */
      mrfiSpiCmdStrobe( STX );

      /* Delay long enough for the PA_PD signal to indicate a
       * successful transmit. This is the 250 XOSC periods
       * (9.6 us for a 26 MHz crystal) See section 19.6 of 2500 datasheet.
       * Found out that we need a delay of atleast 20 us on CC2500 and
       * 25 us on CC1100 to see the PA_PD signal change.
       */
      Mrfi_DelayUsec(48);

      /* PA_PD signal goes from HIGH to LOW when going from RX state.
       * This transition is trapped as a falling edge interrupt flag
       * to indicate that CCA passed and the transmit has started.
       */
      if (MRFI_PAPD_INT_FLAG_IS_SET())
      {
        /* ------------------------------------------------------------------
        *    Clear Channel Assessment passed.
        *   ----------------------------------
        */

        /* Clear the PA_PD int flag */
        MRFI_CLEAR_PAPD_PIN_INT_FLAG();

        phyTxStartCallBack(); // 开始发送成功后记录发送时刻
        //DEBUG_CHAR( DBG_TX,DBG_CHAR_TXSTART);

        Mrfi_DelayUsecLong( MRFI_MAX_TRANSMIT_TIME_us / 1000,
                            MRFI_MAX_TRANSMIT_TIME_us % 1000,
                            mrfi_TxCCADone );

        /* transmit done */

	 // 完全发送成功后调用回调函数, 由于GDO0用于了PA_PD，此时无法用外部中断
	 // 指示发送结束，所以用上面的delay函数，该函数结束表示发送结束
        //Finished TX, do call back
        //DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_TXFIN );
        phyTxEndCallBack();
        macTxCallback();
	
        break;
      }
      else
      {
        /* ------------------------------------------------------------------
         *    Clear Channel Assessment failed.
         *   ----------------------------------
         */
	  //DEBUG_STRING(DBG_INFO, "CC1101: CCA Failed. \n");

        /* Turn off radio and save some power during backoff */

        /* NOTE: Can't use Mrfi_RxModeOff() - since it tries to update the
         * sync signal status which we are not using during the TX operation.
         */
        MRFI_STROBE_IDLE_AND_WAIT();

        /* flush the receive FIFO of any residual data */
        mrfiSpiCmdStrobe( SFRX );

        /* Retry ? */
        if (ccaRetries != 0)
        {
          /* delay for a random number of backoffs */
          Mrfi_RandomBackoffDelay();

          mrfiSpiCmdStrobe( SFTX ); // flush the tx fifo
          mrfiSpiWriteTxFifo(frm, flen);

          /* decrement CCA retries before loop continues */
          ccaRetries--;
        }
        else /* No CCA retries are left, abort */
        {
          /* set return value for failed transmit and break */
          res = LOWSN_STATUS_PHY_CHANNEL_BUSY;
          //DEBUG_CHAR( DBG_TX,DBG_CHAR_TXBUSY);
          break;
        }
      } /* CCA Failed */
    } /* CCA loop */

  /* Done with TX. Clean up time... */

  /* Radio is already in IDLE state */

  /*
   * Flush the transmit FIFO.  It must be flushed so that
   * the next transmit can start with a clean slate.
   */

    test_point: 

  
  mrfiSpiCmdStrobe( SFTX );

  /* Restore GDO_0 to be SYNC signal */
  MRFI_CONFIG_GDO0_AS_SYNC_SIGNAL();

  /* If the radio was in RX state when transmit was attempted,
   * put it back to Rx On state.
   */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }

  return(res);
  
}
void spp_rf_IRQ1(uint_8 irq_res_status )
{

  BYTE flen = 0x00;
  BYTE *ptr=NULL;
  BYTE *rx_frame=NULL;
  BYTE ack_bytes[5];
  BYTE crc;
  BYTE i;
  BYTE rxBytes;
#ifdef LOWSN_MANUAL_MAC_ACK
  UINT8 seq;
#endif
  //define alternate names for readability in this function
#define  fcflsb  ack_bytes[0]
#define  fcfmsb  ack_bytes[1]
#define  dstmode ack_bytes[2]
#define  srcmode ack_bytes[3]
  rxBytes = trx_frame_length_read();
  if(rxBytes >127)
    return;
  if(irq_res_status == 0xff||irq_res_status == 0xC9||irq_res_status == 0x8f||irq_res_status == 0x19) 	
  //if(irq_res_status != 0x08)
  {
        for(i=0;i<rxBytes+2;i++)
       {
         SPI2_SendByte(0xff);;;	
       }
       //STM32F107_Led_Toggle(2);
       return;
  }
   g_rx_pkt_cnt++;
  halLedToggle(1); 
  
   if (!macRxBuffFull()) 
   {       
        rx_frame = MemAlloc(rxBytes + 3);
   }
   else 
   {
        return;
   }
  //rx_frame = MemAlloc(rxBytes + 3);
  if(rx_frame == NULL)
    return;
  ptr = rx_frame;
 // *ptr = rxBytes+2;  //2017.4.24
  *ptr = rxBytes;
  ptr++;
  for(i=0;i<rxBytes+2;i++)
   {
     *ptr = SPI2_SendByte(0xff);
      ptr++;			
    }
 //   conPrintROMString("\n");  //pu
    GPIO_SetBits(GPIOA,GPIO_Pin_4);            //屏蔽CSN
  
  if (rxBytes == 0)
  {
    /* receive FIFO is empty - do nothing, skip to end */
	//conPrintROMString("*");
  }
  else
  {
    /* receive FIFO is not empty, continue processing */
	
    /* ------------------------------------------------------------------
     *    Process frame length
     *   ----------------------
     */

    /* read the first byte from FIFO - the packet length */
    // 注意: 采用IEEE 15.4格式后，数据包的第一个字节所表示的长度包含了CRC长度
   // mrfiSpiReadRxFifo(&flen, MRFI_LENGTH_FIELD_SIZE);

    #if 0
    DEBUG_STRING(DBG_INFO, "AT86RF212: RX Pkt len :");
    DEBUG_UINT8(DBG_INFO, rxBytes);
    DEBUG_STRING(DBG_INFO, "\n");
    #endif

    {
      
	  
     // 由于CC1101第一个长度字节不包含CRC域长度，而协议栈上层处理时均按照IEEE 802.15.4模式，长度字节
     // 包含负载长度和CRC长度。为了保持处理的统一性，将flen改为包含CRC长度的值。
     flen = rxBytes;
#ifdef LOWSN_MANUAL_MAC_ACK
        //get the sequence number of the frame
    seq = rx_frame[3];
#endif  
    if (flen == LOWSN_ACKFRAME_LENGTH) {
      //this should be an ACK.
      //read the packet, do not allocate space for it
      //DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_ACKPKT );
      ack_bytes[0]= flen;
      //mrfiSpiReadRxFifo(&(ack_bytes[1]), LOWSN_ACKFRAME_LENGTH-1);
      //mrfiSpiReadRxFifo(&crc, 1);
      for(i=1; i<flen+1 ; i++)
      	{
      		ack_bytes[i]=rx_frame[i];
      	}
	crc = ack_bytes[LOWSN_ACKFRAME_LENGTH-1];
      //check CRC
      if ( LOWSN_IS_ACK(ack_bytes[1]))
      {
        // CRC ok, perform callback if this is an ACK
        //Convert the raw RSSI value and do offset compensation for this radio 
        ack_bytes[4] = rx_frame[rxBytes+1];
        macRxCallback(ack_bytes, ack_bytes[4]);

       // MemFree(rx_frame);
      }
    }
	
    else 
    {
      //not an ack packet, lets do some more early rejection
      //read the fcflsb, fcfmsb

     // mrfiSpiReadRxFifo(&fcflsb, 1);
     // mrfiSpiReadRxFifo(&fcfmsb, 1);
	fcflsb = rx_frame[1];
	fcfmsb = rx_frame[2]; 
   
        srcmode = LOWSN_GET_SRC_ADDR(fcfmsb);
        dstmode = LOWSN_GET_DST_ADDR(fcfmsb);
        if ((srcmode == LOWSN_ADDRMODE_NOADDR) && (dstmode == LOWSN_ADDRMODE_NOADDR)) {
          //reject this packet, no addressing info
          goto do_rxflush;
        }
 //     }

      if (!macRxBuffFull()) {
        //MAC TX buffer has room
        //allocate new memory space
        //read the length
        //rx_frame = MemAlloc(flen+1);
	 //rx_frame = MemAlloc(flen + 1);
        //ptr = rx_frame;
      } else {
        //MAC RX buffer is full
        //DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_MACFULL );
      }

      // at this point, if ptr is null, then either
      // the MAC RX buffer is full or there is  no
      // free memory for the new frame, or the packet is
      // going to be rejected because of addressing info.
      // In these cases, we need to
      // throw the RX packet away
//      if (ptr == NULL) {
        //just flush the bytes
//        goto do_rxflush;
//      }else 
    {
 
        if (1) {
          //CRC good
          //change the RSSI byte from 2's complement to unsigned number
          //*(ptr-2) = Mrfi_CalculateRssi(*(ptr-2) );
          
#ifdef LOWSN_MANUAL_MAC_ACK
//	if (LOWSN_GET_ACK_REQUEST(fcflsb)) 
    {
	  // only data frame and MAC command frame may need ACK.
	  if ((LOWSN_IS_DATA(fcflsb)) || (LOWSN_IS_MAC(fcflsb)))  {
		phy_pib.flags.bits.needACK = 1; 					
		phy_pib.rcvSeqforACK = seq; 			
		phy_pib.flags.bits.ackIsSending = 1; 
	   }
	}
#endif          
          phyRxCallback();
         //  conPrintROMString("ED ...  \n");
      //conPrintUINT8(*(ptr-1));
     // conPrintROMString("ED ...  \n");
          macRxCallback(rx_frame, *(ptr-1));
        }else {
          // CRC bad. Free the packet
          //DEBUG_STRING(DBG_INFO, "CC1101: RX CRC Error!\n ");
          MemFree(rx_frame);
        }
      }
    }

  }
  
 }

      //flush any remaining bytes
do_rxflush:

 // MRFI_STROBE_IDLE_AND_WAIT();
 // mrfiSpiCmdStrobe( SFRX );
 // mrfiSpiCmdStrobe( SRX );
 // RF212_Rx_Mode();

  usrIntCallback();
  MRFI_ENABLE_SYNC_PIN_INT( );  

#undef  fcflsb
#undef  fcfmsb
#undef  dstmode
#undef  srcmode
}

void spp_rf_IRQ(void)
{

  BYTE flen = 0x00;
  BYTE *ptr=NULL;
  BYTE *rx_frame;
  BYTE ack_bytes[5];
  BYTE crc;
  BYTE rxBytes;
  //define alternate names for readability in this function
#define  fcflsb ack_bytes[0]
#define  fcfmsb  ack_bytes[1]
#define  dstmode ack_bytes[2]
#define  srcmode ack_bytes[3]

  MRFI_DISABLE_SYNC_PIN_INT( );

  //DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_RXRCV );

 /* We should receive this interrupt only in RX state
   * Should never receive it if RX was turned On only for
   * some internal mrfi processing like - during CCA.
   * Otherwise something is terribly wrong.
   */
  MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );



  /* ------------------------------------------------------------------
   *    Get RXBYTES
   *   -------------
   */

  /*
   *  Read the RXBYTES register from the radio.
   *  Bit description of RXBYTES register:
   *    bit 7     - RXFIFO_OVERFLOW, set if receive overflow occurred
   *    bits 6:0  - NUM_BYTES, number of bytes in receive FIFO
   *
   *  Due a chip bug, the RXBYTES register must read the same value twice
   *  in a row to guarantee an accurate value.
   */
  {
    uint8_t rxBytesVerify;

    rxBytesVerify = mrfiSpiReadReg( RXBYTES );

    do
    {
      rxBytes = rxBytesVerify;
      rxBytesVerify = mrfiSpiReadReg( RXBYTES );
    }
    while (rxBytes != rxBytesVerify);
  }


  /* ------------------------------------------------------------------
   *    FIFO empty?
   *   -------------
   */

  /*
   *  See if the receive FIFIO is empty before attempting to read from it.
   *  It is possible nothing the FIFO is empty even though the interrupt fired.
   *  This can happen if address check is enabled and a non-matching packet is
   *  received.  In that case, the radio automatically removes the packet from
   *  the FIFO.
   */
  if (rxBytes == 0)
  {
    /* receive FIFO is empty - do nothing, skip to end */
	//conPrintROMString("*");
  }
  else
  {
    /* receive FIFO is not empty, continue processing */
	
    /* ------------------------------------------------------------------
     *    Process frame length
     *   ----------------------
     */

    /* read the first byte from FIFO - the packet length */
    // 注意: 采用IEEE 15.4格式后，数据包的第一个字节所表示的长度包含了CRC长度
    mrfiSpiReadRxFifo(&flen, MRFI_LENGTH_FIELD_SIZE);

    #if 0
    DEBUG_STRING(DBG_INFO, "CC1101: RX Pkt len :");
    DEBUG_UINT8(DBG_INFO, flen);
    DEBUG_STRING(DBG_INFO, "\n");
    #endif

    /*
     *  Make sure that the frame length just read corresponds to number of bytes in the buffer.
     *  If these do not match up something is wrong.
     *
     *  This can happen for several reasons:
     *   1) Incoming packet has an incorrect format or is corrupted.
     *   2) The receive FIFO overflowed.  Overflow is indicated by the high
     *      bit of rxBytes.  This guarantees the value of rxBytes value will not
     *      match the number of bytes in the FIFO for overflow condition.
     *   3) Interrupts were blocked for an abnormally long time which
     *      allowed a following packet to at least start filling the
     *      receive FIFO.  In this case, all received and partially received
     *      packets will be lost - the packet in the FIFO and the packet coming in.
     *      This is the price the user pays if they implement a giant
     *      critical section.
     *   4) A failed transmit forced radio to IDLE state to flush the transmit FIFO.
     *      This could cause an active receive to be cut short.
     *
     *  Also check the sanity of the length to guard against rogue frames.
     */

	
    if ((rxBytes != (flen + MRFI_LENGTH_FIELD_SIZE + MRFI_RX_METRICS_SIZE)) ||
		(flen > MRFI_MAX_FRAME_SIZE) )
    {
	  //DEBUG_STRING(DBG_INFO, "CC1101: RX pkt len error!\n ");
    }
    else
    {
      /* bytes-in-FIFO and frame length match up - continue processing */

      /* ------------------------------------------------------------------
       *    Get packet
       *   ------------
       */
	  
     // 由于CC1101第一个长度字节不包含CRC域长度，而协议栈上层处理时均按照IEEE 802.15.4模式，长度字节
     // 包含负载长度和CRC长度。为了保持处理的统一性，将flen改为包含CRC长度的值。
     flen = flen + MRFI_RX_METRICS_SIZE;
       
    if (flen == LOWSN_ACKFRAME_LENGTH) {
      //this should be an ACK.
      //read the packet, do not allocate space for it
      //DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_ACKPKT );
      ack_bytes[0]= flen;
      mrfiSpiReadRxFifo(&(ack_bytes[1]), LOWSN_ACKFRAME_LENGTH-1);
      mrfiSpiReadRxFifo(&crc, 1);

      //check CRC
      if ((crc & 0x80) && LOWSN_IS_ACK(ack_bytes[1]))
      {
        // CRC ok, perform callback if this is an ACK
        //Convert the raw RSSI value and do offset compensation for this radio 
        ack_bytes[4] = Mrfi_CalculateRssi(ack_bytes[4]);
        macRxCallback(ack_bytes, ack_bytes[4]);
      }
    }
	
    else 
    {
      //not an ack packet, lets do some more early rejection
      //read the fcflsb, fcfmsb

      mrfiSpiReadRxFifo(&fcflsb, 1);
      mrfiSpiReadRxFifo(&fcfmsb, 1);

      if (!local_radio_flags.bits.listen_mode) {
        //only reject if not in listen mode
        //get the src, dst addressing modes
        srcmode = LOWSN_GET_SRC_ADDR(fcfmsb);
        dstmode = LOWSN_GET_DST_ADDR(fcfmsb);
        if ((srcmode == LOWSN_ADDRMODE_NOADDR) && (dstmode == LOWSN_ADDRMODE_NOADDR)) {
          //reject this packet, no addressing info
          goto do_rxflush;
        }
      }

      if (!macRxBuffFull()) {
        //MAC TX buffer has room
        //allocate new memory space
        //read the length
        //rx_frame = MemAlloc(flen+1);
	 rx_frame = MemAlloc(flen + 1);
        ptr = rx_frame;
      } else {
        //MAC RX buffer is full
        //DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_MACFULL );
      }

      // at this point, if ptr is null, then either
      // the MAC RX buffer is full or there is  no
      // free memory for the new frame, or the packet is
      // going to be rejected because of addressing info.
      // In these cases, we need to
      // throw the RX packet away
      if (ptr == NULL) {
        //just flush the bytes
        goto do_rxflush;
      }else {
        //save packet, including the length
        *ptr = flen; ptr++;
        //save the fcflsb, fcfmsb bytes
        *ptr = fcflsb; ptr++; flen--;
        *ptr = fcfmsb; ptr++; flen--;
		
        //get the rest of the bytes
	 mrfiSpiReadRxFifo(ptr, flen);
	 ptr = ptr + flen;
        //do RX callback
        //check the CRC
	
        if (*(ptr-1) & 0x80) {
          //CRC good
          //change the RSSI byte from 2's complement to unsigned number
          *(ptr-2) = Mrfi_CalculateRssi(*(ptr-2) );
          phyRxCallback();
          macRxCallback(rx_frame, *(ptr-2));
        }else {
          // CRC bad. Free the packet
          //DEBUG_STRING(DBG_INFO, "CC1101: RX CRC Error!\n ");
          MemFree(rx_frame);
        }
      }
    }

  }
  
 }

      //flush any remaining bytes
do_rxflush:

  MRFI_STROBE_IDLE_AND_WAIT();
  mrfiSpiCmdStrobe( SFRX );
  mrfiSpiCmdStrobe( SRX );

  usrIntCallback();
  MRFI_ENABLE_SYNC_PIN_INT( );  

#undef  fcflsb
#undef  fcfmsb
#undef  dstmode
#undef  srcmode
}



/**************************************************************************************************
 * @fn          Mrfi_RxModeOn
 *
 * @brief       Put radio into receive mode.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOn(void)
{
  /* clear any residual receive interrupt */
  MRFI_CLEAR_SYNC_PIN_INT_FLAG();

  /* send strobe to enter receive mode */
  mrfiSpiCmdStrobe( SRX );

  /* enable receive interrupts */
  MRFI_ENABLE_SYNC_PIN_INT();

}

/**************************************************************************************************
 * @fn          MRFI_RxOn
 *
 * @brief       Turn on the receiver.  No harm is done if this function is called when
 *              receiver is already on.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxOn(void)
{
  /* radio must be awake before we can move it to RX state */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* if radio is off, turn it on */
  if(mrfiRadioState != MRFI_RADIO_STATE_RX)
  {
    mrfiRadioState = MRFI_RADIO_STATE_RX;
    Mrfi_RxModeOn();
  }
}

/**************************************************************************************************
 * @fn          Mrfi_RxModeOff
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOff(void)
{
  /*disable receive interrupts */
  MRFI_DISABLE_SYNC_PIN_INT();

  /* turn off radio */
  MRFI_STROBE_IDLE_AND_WAIT();

  /* flush the receive FIFO of any residual data */
  mrfiSpiCmdStrobe( SFRX );

  /* clear receive interrupt */
  MRFI_CLEAR_SYNC_PIN_INT_FLAG();

}


/**************************************************************************************************
 * @fn          MRFI_RxIdle
 *
 * @brief       Put radio in idle mode (receiver if off).  No harm is done this function is
 *              called when radio is already idle.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxIdle(void)
{
  /* radio must be awake to move it to idle mode */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* if radio is on, turn it off */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOff();
    mrfiRadioState = MRFI_RADIO_STATE_IDLE;
  }
}


/**************************************************************************************************
 * @fn          MRFI_Sleep
 *
 * @brief       Request radio go to sleep.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_Sleep(void)
{
  /* Critical section necessary for watertight testing and
   * setting of state variables.
   */
  //BSP_ENTER_CRITICAL_SECTION(s);

  /* If radio is not asleep, put it to sleep */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    /* go to idle so radio is in a known state before sleeping */
    MRFI_RxIdle();

    mrfiSpiCmdStrobe( SPWD );

    /* Our new state is OFF */
    mrfiRadioState = MRFI_RADIO_STATE_OFF;
  }

  //BSP_EXIT_CRITICAL_SECTION(s);
}


/**************************************************************************************************
 * @fn          MRFI_WakeUp
 *
 * @brief       Wake up radio from sleep state.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_WakeUp(void)
{
  /* if radio is already awake, just ignore wakeup request */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    return;
  }

  /* drive CSn low to initiate wakeup */
  CC1101_CS_LOW();

  /* wait for MISO to go high indicating the oscillator is stable */
  while (MRFI_SPI_SO_IS_HIGH());

  /* wakeup is complete, drive CSn high and continue */
  CC1101_CS_HIGH();

  /* enter idle mode */
  mrfiRadioState = MRFI_RADIO_STATE_IDLE;
  MRFI_STROBE_IDLE_AND_WAIT();

}


/**************************************************************************************************
 * @fn          Mrfi_CalculateRssi
 *
 * @brief       Does binary to decimal conversiont and offset compensation.
 *
 * @param       none
 *
 * @return      RSSI value in units of dBm.
 **************************************************************************************************
 */
int8_t Mrfi_CalculateRssi(uint8_t rawValue)
{
  int16_t rssi;

  /* The raw value is in 2's complement and in half db steps. Convert it to
   * decimal taking into account the offset value.
   */
  if(rawValue >= 128)
  {
    rssi = (int16_t)(rawValue - 256)/2 - MRFI_RSSI_OFFSET;
  }
  else
  {
    rssi = (rawValue/2) - MRFI_RSSI_OFFSET;
  }

  /* Restrict this value to least value can be held in an 8 bit signed int */
  if(rssi < -128)
  {
    rssi = -128;
  }

  return rssi;
}


/**************************************************************************************************
 * @fn          Mrfi_RandomBackoffDelay
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RandomBackoffDelay(void)
{
  uint8_t backoffs;
  uint8_t i;

  /* calculate random value for backoffs - 1 to 16 */
  backoffs = (halGetRandomByte() & 0x0F) + 1;

  /* delay for randomly computed number of backoff periods */
  for (i=0; i<backoffs; i++)
  {
    Mrfi_DelayUsec( sBackoffHelper );
  }
}

// 这个定时函数不准，只有加上打印的叹号，射频才能正常收发，
// 需要在研究一下定时器
static void Mrfi_DelayUsec(uint16_t howLong)
{

  volatile uint32_t tmp_timer;
  uint16_t count = howLong/MRFI_MAX_DELAY_US;
  
  if (howLong)
  {
    tmp_timer= halGetMACTimer();
    while (halMACTimerNowDelta(tmp_timer) < count);
    //conPrintROMString("!");
	
  }

  return;
}


/****************************************************************************************************
 * @fn          Mrfi_DelayUsecLong -- Frequency Hopping Disabled
 *
 * @brief       Delay the number of microseconds specified by the parameters passed as
 *                 ms * 1000 + us
 *              If the parameter <term> is not NULL, then it must point to a function
 *              which is called during the delay period and if that function returns
 *              TRUE the delay period is truncated at that point.  If the parameter
 *              <term> is NULL or never returns TRUE, the function returns after the
 *              entire delay period has transpired.
 *
 * input parameters
 * @param   ms   - number of milliseconds to delay
 * @param   us   - number of microseconds to delay
 * @param   term - function pointer to semaphore test function to truncate delay period
 *
 * @return      status; TRUE if timeout truncated due to semaphore test, FALSE otherwise
 ****************************************************************************************************
 */
BOOL Mrfi_DelayUsecLong(uint32_t ms, uint16_t us, TimeoutTerminator_t term)
{
  BOOL timeout = FALSE;
  uint16_t count;
  volatile uint32_t tmp_timer;

  while (!timeout && ms)
  {
    count = APP_USEC_VALUE / MRFI_MAX_DELAY_US;
    do
    {
      Mrfi_DelayUsec(MRFI_MAX_DELAY_US);
      if (term != NULL)
        timeout = term( );
    } while (!timeout && count--);
    ms--;
  }
  count = us/MRFI_MAX_DELAY_US;
  if (!timeout && us)
  {
    do
    {
      Mrfi_DelayUsec(MRFI_MAX_DELAY_US);
      if (term != NULL)
        timeout = term( );
    } while (!timeout && count--);
  }
  return timeout;
}




/*********************************************************************************************
@function:

software delay, waits is in milliseconds
**********************************************************************************************/
void halWait(BYTE wait){

}


/*********************************************************************************************
@function:

**********************************************************************************************/
void halWaitMs(const UINT32 msecs){
  UINT32 towait;

  towait = msecs;
  while (towait > 100)
  {
    halWait(100);
    towait -= 100;
  }
  halWait(towait);
}

/*********************************************************************************************
@function:

**********************************************************************************************/









/*********************************************************************************************
@function:
输入参数:  SleepMode
#define POWER_MODE_0  0x00  // Clock oscillators on, voltage regulator on
#define POWER_MODE_1  0x01  // 32.768 KHz oscillator on, voltage regulator on
#define POWER_MODE_2  0x02  // 32.768 KHz oscillator on, voltage regulator off
#define POWER_MODE_3  0x03  // All clock oscillators off, voltage regulator off


**********************************************************************************************/
void halSleep(UINT8 SleepMode, UINT32 msecs )
{
  

}



/*********************************************************************************************
@function:

**********************************************************************************************/
INT16 halGetAdcValue()
{
  return 0;
}


// 获取电池电压

UINT16 getVoltageValue(void)
{
  return 0;

}


void halShutdown(void) {

// 等待添加

}

void halWarmstart(void) {

// 等待添加
  
 }

void halInitLed(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //开启GPIOC端口 
        
	//PC4, PC5配置为输出;LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;			   //选择第端口2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //50M时钟速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);				   //GPIO配置函数 
}
void halLedToggle( uint8_t led )
{
  switch (led){
	  case 0:
		GPIOC->ODR ^= GPIO_Pin_4;
		break;
	  case 1:
		GPIOC->ODR ^= GPIO_Pin_5;
		break;
  default:break;
  }
}

