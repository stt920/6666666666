/*

CC2530底层驱动文件

*/


#ifndef HAL_H
#define HAL_H
#include <stdio.h>
#include <string.h>
#include "compiler.h"
#include "ioCC2530.h"
#include "6lowsn_config.h"         //user configurations
#include "6lowsn_common_types.h"   //types common acrosss most files


//-----------------------------------------------------------------------------
// Common values
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef HIGH
#define HIGH 1
#endif

#ifndef LOW
#define LOW 0
#endif

#define halIdle()              //do nothing in idle state

// Macros for configuring IO peripheral location:
// Example usage:
//   IO_PER_LOC_TIMER1_AT_PORT0_PIN234();
//   IO_PER_LOC_TIMER4_AT_PORT2_PIN03();
//   IO_PER_LOC_USART1_AT_PORT0_PIN2345();

#define IO_PER_LOC_TIMER1_AT_PORT0_PIN234() do { PERCFG = (PERCFG&~0x40)|0x00; } while (0)
#define IO_PER_LOC_TIMER1_AT_PORT1_PIN012() do { PERCFG = (PERCFG&~0x40)|0x40; } while (0)

#define IO_PER_LOC_TIMER3_AT_PORT1_PIN34()  do { PERCFG = (PERCFG&~0x20)|0x00; } while (0)
#define IO_PER_LOC_TIMER3_AT_PORT1_PIN67()  do { PERCFG = (PERCFG&~0x20)|0x20; } while (0)

#define IO_PER_LOC_TIMER4_AT_PORT1_PIN01()  do { PERCFG = (PERCFG&~0x10)|0x00; } while (0)
#define IO_PER_LOC_TIMER4_AT_PORT2_PIN03()  do { PERCFG = (PERCFG&~0x10)|0x10; } while (0)

#define IO_PER_LOC_SPI1_AT_PORT0_PIN2345()  do { PERCFG = (PERCFG&~0x08)|0x00; } while (0)
#define IO_PER_LOC_SPI1_AT_PORT1_PIN4567()  do { PERCFG = (PERCFG&~0x08)|0x08; } while (0)

#define IO_PER_LOC_SPI0_AT_PORT0_PIN2345()  do { PERCFG = (PERCFG&~0x04)|0x00; } while (0)
#define IO_PER_LOC_SPI0_AT_PORT1_PIN2345()  do { PERCFG = (PERCFG&~0x04)|0x04; } while (0)

#define IO_PER_LOC_UART1_AT_PORT0_PIN2345() do { PERCFG = (PERCFG&~0x02)|0x00; } while (0)
#define IO_PER_LOC_UART1_AT_PORT1_PIN4567() do { PERCFG = (PERCFG&~0x02)|0x02; } while (0)

#define IO_PER_LOC_UART0_AT_PORT0_PIN2345() do { PERCFG = (PERCFG&~0x01)|0x00; } while (0)
#define IO_PER_LOC_UART0_AT_PORT1_PIN2345() do { PERCFG = (PERCFG&~0x01)|0x01; } while (0)


// Actual MCU pin configuration:
//
// Peripheral I/O signal     Alt1       Alt2
// -------------------------------------------
// Timer1 channel0           P0.2       P1.2
// Timer1 channel1           P0.3       P1.1
// Timer1 channel2           P0.4       P1.0
// Timer3 channel0           P1.3       P1.6
// Timer3 channel1           P1.4       P1.7
// Timer4 channel0           P1.0       P2.0
// Timer4 channel1           P1.1       P2.3
// USART0 TXD/MOSI           P0.3       P1.5
// USART0 RXD/MISO           P0.2       P1.4
// USART0 RTS/SCK            P0.5       P1.3
// USART0 CTS/SS_N           P0.4       P1.2
// USART1 TXD/MOSI           P0.4       P1.6
// USART1 RXD/MISO           P0.5       P1.7
// USART1 RTS/SCK            P0.3       P1.5
// USART1 CTS/SS_N           P0.2       P1.4

// Macros for configuring IO direction:
// Example usage:
//   IO_DIR_PORT_PIN(0, 3, IO_IN);    // Set P0_3 to input
//   IO_DIR_PORT_PIN(2, 1, IO_OUT);   // Set P2_1 to output

#define IO_DIR_PORT_PIN(port, pin, dir)  \
  do {                                  \
    if (dir == IO_OUT)                 \
      P##port##DIR |= (0x01<<(pin));  \
      else                               \
        P##port##DIR &= ~(0x01<<(pin)); \
   }while(0)

// Where port={0,1,2}, pin={0,..,7} and dir is one of:
#define IO_IN   0
#define IO_OUT  1

// Macros for configuring IO input mode:
// Example usage:
//   IO_IMODE_PORT_PIN(0,0,IO_IMODE_PUD);
//   IO_IMODE_PORT_PIN(2,0,IO_IMODE_TRI);
//   IO_IMODE_PORT_PIN(1,3,IO_IMODE_PUD);

#define IO_IMODE_PORT_PIN(port, pin, imode) \
  do {                                     \
    if (imode == IO_IMODE_TRI)            \
      P##port##INP |= (0x01<<(pin));     \
      else                                  \
        P##port##INP &= ~(0x01<<(pin));    \
   } while (0)

// where imode is one of:
#define IO_IMODE_PUD  0 // Pull-up/pull-down
#define IO_IMODE_TRI  1 // Tristate

// Macro for configuring IO drive mode:
// Example usage:
//   IIO_PUD_PORT(0, IO_PULLUP);
//   IIO_PUD_PORT(1, IO_PULLDOWN);
//   IIO_PUD_PORT(2, IO_PULLUP);

#define IO_PUD_PORT(port, pud)        \
  do {                               \
    if (pud == IO_PULLDOWN)         \
      P2INP |= (0x01 << (port+5)); \
      else                            \
        P2INP &= ~(0x01 << (port+5));\
   } while (0)

#define IO_PULLUP          0
#define IO_PULLDOWN        1

// Macros for function select (General purpose I/O / Peripheral function):
// Example usage:
//   IO_FUNC_PORT0_PIN0(0, 0, IO_FUNC_PERIPH);
//   IO_FUNC_PORT0_PIN1(0, 1, IO_FUNC_GIO);
//   IO_FUNC_PORT2_PIN3(2, 3, IO_FUNC_PERIPH);

#define IO_FUNC_PORT_PIN(port, pin, func)  \
  do {                                    \
    if((port == 2) && (pin == 3)){       \
      if (func) {                       \
        P2SEL |= 0x02;                 \
         } else {                          \
           P2SEL &= ~0x02;                \
         }                                 \
      }                                    \
      else if((port == 2) && (pin == 4)){  \
        if (func) {                       \
          P2SEL |= 0x04;                 \
         } else {                          \
           P2SEL &= ~0x04;                \
         }                                 \
      }                                    \
      else{                                \
        if (func) {                       \
          P##port##SEL |= (0x01<<(pin)); \
         } else {                          \
           P##port##SEL &= ~(0x01<<(pin));\
        }                                  \
      }                                    \
   } while (0)

// where func is one of:
#define IO_FUNC_GIO     0 // General purpose I/O
#define IO_FUNC_PERIPH  1 // Peripheral function

// Macros for configuring the ADC input:
// Example usage:
//   IO_ADC_PORT0_PIN(0, IO_ADC_EN);
//   IO_ADC_PORT0_PIN(4, IO_ADC_DIS);
//   IO_ADC_PORT0_PIN(6, IO_ADC_EN);

#define IO_ADC_PORT0_PIN(pin, adcEn) \
  do {                              \
    if (adcEn)                     \
      ADCCFG |= (0x01<<pin);      \
      else                           \
        ADCCFG &= ~(0x01<<pin); }   \
          while (0)

// where adcEn is one of:
#define IO_ADC_EN           1 // ADC input enabled
#define IO_ADC_DIS          0 // ADC input disab

//from ChipCon
//#define CC2530_FLASH_SIZE 32
//#define CC2530_FLASH_SIZE 64
//#define CC2530_FLASH_SIZE 128
#define CC2530_FLASH_SIZE 256

#if (CC2530_FLASH_SIZE == 32)
#define IEEE_ADDRESS_ARRAY 0x7FE8
#elif (CC2530_FLASH_SIZE == 64) || (CC2530_FLASH_SIZE == 128) || (CC2530_FLASH_SIZE == 256)
#define IEEE_ADDRESS_ARRAY 0xFFE8
#endif


#define HAL_SUSPEND(x)         //dummy in uC, only needed for Win32

//Timer Support
//For the CC2530, we set the timer to exactly 1 tick per symbol
//assuming a clock frequency of 32MHZ, and 2.4GHz

#define SYMBOLS_PER_MAC_TICK()     1
#define SYMBOLS_TO_MACTICKS(x) (x/SYMBOLS_PER_MAC_TICK())   //every
#define MSECS_TO_MACTICKS(x)   (x*(LOWSN_SYMBOLS_PER_SECOND/1000))
#define MACTIMER_MAX_VALUE 0x00FFFFFF   //20 bit counter
#define halMACTimerNowDelta(x) ((halGetMACTimer()-(x))& MACTIMER_MAX_VALUE)
#define halMACTimerDelta(x,y) ((x-(y))& MACTIMER_MAX_VALUE)

/******************************************************************************
*******************       Interrupt functions/macros        *******************
******************************************************************************/

// Macros which simplify access to interrupt enables, interrupt flags and
// interrupt priorities. Increases code legibility.

//******************************************************************************

#define INT_ON   1
#define INT_OFF  0
#define INT_SET  1
#define INT_CLR  0

// Global interrupt enables
#define INT_GLOBAL_ENABLE(on) EA=(!!on)
#define SAVE_AND_DISABLE_GLOBAL_INTERRUPT(x) {x=EA;EA=0;}
#define RESTORE_GLOBAL_INTERRUPT(x) EA=x
#define ENABLE_GLOBAL_INTERRUPT() INT_GLOBAL_ENABLE(INT_ON)
#define DISABLE_GLOBAL_INTERRUPT() INT_GLOBAL_ENABLE(INT_OFF)

#define DISABLE_ALL_INTERRUPTS() (IEN0 = IEN1 = IEN2 = 0x00)

#define INUM_RFERR 0
#define INUM_ADC   1
#define INUM_URX0  2
#define INUM_URX1  3
#define INUM_ENC   4
#define INUM_ST    5
#define INUM_P2INT 6
#define INUM_UTX0  7
#define INUM_DMA   8
#define INUM_T1    9
#define INUM_T2    10
#define INUM_T3    11
#define INUM_T4    12
#define INUM_P0INT 13
#define INUM_UTX1  14
#define INUM_P1INT 15
#define INUM_RF    16
#define INUM_WDT   17

#define NBR_OF_INTERRUPTS 18


#define INT_ENABLE_RF(on)     { (on) ? (IEN2 |= 0x01) : (IEN2 &= ~0x01); }
#define INT_ENABLE_RFERR(on)  { RFERRIE = on; }
#define INT_ENABLE_T2(on)     { T2IE    = on; }
#define INT_ENABLE_T3(on)     { T3IE    = on; }
#define INT_ENABLE_URX0(on)   { URX0IE  = on; }


#define INT_GETFLAG_RFERR() RFERRIF
#define INT_GETFLAG_RF()    S1CON &= ~0x03


#define INT_SETFLAG_RFERR(f) RFERRIF= f
#define INT_SETFLAG_RF(f)  { (f) ? (S1CON |= 0x03) : (S1CON &= ~0x03); }
#define INT_SETFLAG_T2(f)  { T2IF  = f;  }
#define INT_SETFLAG_T3(f)  { T3IF  = f;  }




/******************************************************************************
*******************         Common USART functions/macros   *******************
******************************************************************************/

// The macros in this section are available for both SPI and UART operation.

//*****************************************************************************

// Example usage:
//   USART0_FLUSH();
#define USART_FLUSH(num)              (U##num##UCR |= 0x80)
#define USART0_FLUSH()                USART_FLUSH(0)
#define USART1_FLUSH()                USART_FLUSH(1)

// Example usage:
//   if (USART0_BUSY())
//     ...
#define USART_BUSY(num)               (U##num##CSR & 0x01 == 0x01)
#define USART0_BUSY()                 USART_BUSY(0)
#define USART1_BUSY()                 USART_BUSY(1)

// Example usage:
//   while(!USART1_BYTE_RECEIVED())
//     ...
#define USART_BYTE_RECEIVED(num)      ((U##num##CSR & 0x04) == 0x04)
#define USART0_BYTE_RECEIVED()        USART_BYTE_RECEIVED(0)
#define USART1_BYTE_RECEIVED()        USART_BYTE_RECEIVED(1)

// Example usage:
//   if(USART1_BYTE_TRANSMITTED())
//     ...
#define USART_BYTE_TRANSMITTED(num)   ((U##num##CSR & 0x02) == 0x02)
#define USART0_BYTE_TRANSMITTED()     USART_BYTE_TRANSMITTED(0)
#define USART1_BYTE_TRANSMITTED()     USART_BYTE_TRANSMITTED(1)


/******************************************************************************
*******************  USART-UART specific functions/macros   *******************
******************************************************************************/
// The macros in this section simplify UART operation.
#define BAUD_E(baud, clkDivPow) (     \
  (baud==1200)   ?  5 +clkDivPow : \
    (baud==2400)   ?  6  +clkDivPow : \
      (baud==4800)   ?  7  +clkDivPow : \
        (baud==9600)   ?  8  +clkDivPow : \
          (baud==14400)  ?  8  +clkDivPow : \
            (baud==19200)  ?  9  +clkDivPow : \
              (baud==28800)  ?  9  +clkDivPow : \
                (baud==38400)  ?  10 +clkDivPow : \
                  (baud==57600)  ?  10 +clkDivPow : \
                    (baud==76800)  ?  11 +clkDivPow : \
                      (baud==115200) ?  11 +clkDivPow : \
                        (baud==153600) ?  12 +clkDivPow : \
                          (baud==230400) ?  12 +clkDivPow : \
                            (baud==307200) ?  13 +clkDivPow : \
                              0  )


#define BAUD_M(baud) (      \
  (baud==1200)   ?  59  : \
    (baud==2400)   ?  59  : \
      (baud==4800)   ?  59  : \
        (baud==9600)   ?  59  : \
          (baud==14400)  ?  216 : \
            (baud==19200)  ?  59  : \
              (baud==28800)  ?  216 : \
                (baud==38400)  ?  59  : \
                  (baud==57600)  ?  216 : \
                    (baud==76800)  ?  59  : \
                      (baud==115200) ?  216 : \
                        (baud==153600) ?  59  : \
                          (baud==230400) ?  216 : \
                            (baud==307200) ?  59  : \
                              0)




//*****************************************************************************

// Macro for setting up a UART transfer channel. The macro sets the appropriate
// pins for peripheral operation, sets the baudrate, and the desired options of
// the selected uart. _uart_ indicates which uart to configure and must be
// either 0 or 1. _baudRate_ must be one of 2400, 4800, 9600, 14400, 19200,
// 28800, 38400, 57600, 76800, 115200, 153600, 230400 or 307200. Possible
// options are defined below.
//
// Example usage:
//
//      UART_SETUP(0,115200,HIGH_STOP);
//
// This configures uart 0 for contact with "hyperTerminal", setting:
//      Baudrate:           115200
//      Data bits:          8
//      Parity:             None
//      Stop bits:          1
//      Flow control:       None
//

#define UART_SETUP(uart, baudRate, options)      \
  do {                                          \
    if((uart) == 0){                           \
      if(PERCFG & 0x01){                      \
        P1SEL |= 0x30;                       \
         } else {                                \
           P0SEL |= 0x0C;                       \
         }                                       \
      }                                          \
      else {                                     \
        if(PERCFG & 0x02){                      \
          P1SEL |= 0xC0;                       \
         } else {                                \
           P0SEL |= 0x30;                       \
         }                                       \
      }                                          \
        \
          U##uart##GCR = BAUD_E((baudRate),CLKSPD);  \
            U##uart##BAUD = BAUD_M(baudRate);          \
              \
                U##uart##CSR |= 0x80;                      \
                  \
                    \
                      U##uart##UCR |= ((options) | 0x80);        \
                        \
                          if((options) & TRANSFER_MSB_FIRST){        \
                            U##uart##GCR |= 0x20;                   \
      }                                          \
        U##uart##CSR |= 0x40;                      \
   } while(0)



//can do this via a macro
#define halSetBaud(baud) UART_SETUP(0, baud, HIGH_STOP);


// Options for UART_SETUP macro
#define FLOW_CONTROL_ENABLE         0x40
#define FLOW_CONTROL_DISABLE        0x00
#define EVEN_PARITY                 0x20
#define ODD_PARITY                  0x00
#define NINE_BIT_TRANSFER           0x10
#define EIGHT_BIT_TRANSFER          0x00
#define PARITY_ENABLE               0x08
#define PARITY_DISABLE              0x00
#define TWO_STOP_BITS               0x04
#define ONE_STOP_BITS               0x00
#define HIGH_STOP                   0x02
#define LOW_STOP                    0x00
#define HIGH_START                  0x01
#define TRANSFER_MSB_FIRST          0x80
#define TRANSFER_MSB_LAST           0x00
#define UART_ENABLE_RECEIVE         0x40


// Example usage:
//   if(UART0_PARERR())
//     ...
#define UART_PARERR(num)      ((U##num##CSR & 0x08) == 0x08)
#define UART0_PARERR()        UART_PARERR(0)
#define UART1_PARERR()        UART_PARERR(1)

// Example usage:
//   if(UART1_FRAMEERR())
//     ...
#define UART_FRAMEERR(num)    ((U ##num## CSR & 0x10) == 0x10)
#define UART0_FRAMEERR()      UART_FRAMEERR(0)
#define UART1_FRAMEERR()      UART_FRAMEERR(1)


// Example usage:
//   char ch = 'A';
//   UART1_SEND(ch);
//   ...
//   UART1_RECEIVE(ch);
#define UART_SEND(num, x)   U##num##DBUF = x
#define UART0_SEND(x)       UART_SEND(0, x)
#define UART1_SEND(x)       UART_SEND(1, x)

#define UART_RECEIVE(num, x)  x = U##num##DBUF
#define UART0_RECEIVE(x)      UART_RECEIVE(0, x)
#define UART1_RECEIVE(x)      UART_RECEIVE(1, x)

/******************************************************************************
*******************      Power and clock management        ********************
*******************************************************************************

These macros are used to set power-mode, clock source and clock speed.

******************************************************************************/
/*
#### RADIO Support
*/

/* FSMSTAT1 */
#define TX_ACTIVE                BV(1)
#define CCA                           BV(4)
#define SFD                           BV(5)
#define FIFOP                        BV(6)
#define FIFO                          BV(7)

#define MAC_RADIO_RX_IS_AT_THRESHOLD()                (FSMSTAT1 & FIFOP)
#define MAC_RADIO_RX_IS_AT_BUSY()                           (FSMSTAT1 & CCA)
#define MAC_RADIO_CCA_INDICATION()                           (FSMSTAT1 & CCA)




// Various radio settings
//#define PAN_COORDINATOR     0x10
#define ADR_DECODE          0x08
#define AUTO_CRC            0x20
#define AUTO_ACK            0x10
#define AUTO_TX2RX_OFF      0x08
#define RX2RX_TIME_OFF      0x04
#define ACCEPT_ACKPKT       0x01


//-----------------------------------------------------------------------------
// Command Strobe Processor (CSP) instructions
//-----------------------------------------------------------------------------
#define ISTXCAL       0xEC
#define ISRXON        0xE3
#define ISTXON        0xE9
#define ISTXONCCA     0xEA
#define ISRFOFF       0xEF
#define ISFLUSHRX     0xED
#define ISFLUSHTX     0xEE
#define ISACK         0xE6
#define ISACKPEND     0xE7
#define ISNACK        0xE8
#define ISSTART     0xE1
#define ISSTOP      0xE2
#define ISCLEAR     0xFF

#define SACK         0xD6
#define SACKPEND     0xD7


#define MAC_RADIO_RX_ON()                             st( RFST = ISRXON;    )
#define MAC_RADIO_RXTX_OFF()                          st( RFST = ISRFOFF; )
#define MAC_RADIO_FLUSH_RX_FIFO()                     st( RFST = ISFLUSHRX; RFST = ISFLUSHRX; )
#define MAC_RADIO_FLUSH_TX_FIFO()                     st( RFST = ISFLUSHTX; )
#define MAC_RADIO_TX_ON_CCA() 				st( RFST = ISTXONCCA; )
#define MAC_RADIO_TX_ON() 				st( RFST = ISTXON; )

#define MAC_RADIO_SACK() 				st( RFST = SACK; )
#define MAC_RADIO_SACKPEND() 				st( RFST = SACKPEND; )

#define PACKET_FOOTER_SIZE 2    //负载之后的位数


/******************************************************************************
*******************              Utility functions          *******************
******************************************************************************/

/******************************************************************************
* @fn  halWait
*
* @brief
*      This function waits approximately a given number of m-seconds
*      regardless of main clock speed.
*
* Parameters:
*
* @param  BYTE	 wait
*         The number of m-seconds to wait.
*
* @return void
*
******************************************************************************/
void halWait(BYTE wait);



/******************************************************************************
*******************             ADC macros/functions        *******************
*******************************************************************************

These functions/macros simplifies usage of the ADC.

******************************************************************************/
// Macro for setting up a single conversion. If ADCCON1.STSEL = 11, using this
// macro will also start the conversion.
#define ADC_SINGLE_CONVERSION(settings) \
  do{ ADCCON3 = settings; }while(0)

// Macro for setting up a single conversion
#define ADC_SEQUENCE_SETUP(settings) \
  do{ ADCCON2 = settings; }while(0)

// Where _settings_ are the following:
// Reference voltage:
#define ADC_REF_1_25_V      0x00     // Internal 1.25V reference
#define ADC_REF_P0_7        0x40     // External reference on AIN7 pin
#define ADC_REF_AVDD        0x80     // AVDD_SOC pin
#define ADC_REF_P0_6_P0_7   0xC0     // External reference on AIN6-AIN7 differential input

// Resolution (decimation rate):
#define ADC_8_BIT           0x00     //  64 decimation rate
#define ADC_10_BIT          0x10     // 128 decimation rate
#define ADC_12_BIT          0x20     // 256 decimation rate
#define ADC_14_BIT          0x30     // 512 decimation rate
// Input channel:
#define ADC_AIN0            0x00     // single ended P0_0
#define ADC_AIN1            0x01     // single ended P0_1
#define ADC_AIN2            0x02     // single ended P0_2
#define ADC_AIN3            0x03     // single ended P0_3
#define ADC_AIN4            0x04     // single ended P0_4
#define ADC_AIN5            0x05     // single ended P0_5
#define ADC_AIN6            0x06     // single ended P0_6
#define ADC_AIN7            0x07     // single ended P0_7
#define ADC_GND             0x0C     // Ground
#define ADC_TEMP_SENS       0x0E     // on-chip temperature sensor
#define ADC_VDD_3           0x0F     // (vdd/3)


#define HAL_ADC_REF_125V    0x00    /* Internal 1.25V Reference */
#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_CHN_VDD3    0x0f    /* Input channel: VDD/3 */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */


//-----------------------------------------------------------------------------
// Macro for starting the ADC in continuous conversion mode
#define ADC_SAMPLE_CONTINUOUS() \
  do { ADCCON1 &= ~0x30; ADCCON1 |= 0x10; } while (0)

// Macro for stopping the ADC in continuous mode (and setting the ADC to be
// started manually by ADC_SAMPLE_SINGLE() )
#define ADC_STOP() \
  do { ADCCON1 |= 0x30; } while (0)

// Macro for initiating a single sample in single-conversion mode (ADCCON1.STSEL = 11).
#define ADC_SAMPLE_SINGLE() \
  do { ADC_STOP(); ADCCON1 |= 0x40;  } while (0)

// Macro for configuring the ADC to be started from T1 channel 0. (T1 ch 0 must be in compare mode!!)
#define ADC_TRIGGER_FROM_TIMER1()  do { ADC_STOP(); ADCCON1 &= ~0x10;  } while (0)

// Expression indicating whether a conversion is finished or not.
#define ADC_SAMPLE_READY()  (ADCCON1 & 0x80)

// Macro for setting/clearing a channel as input of the ADC
#define ADC_ENABLE_CHANNEL(ch)   ADCCFG |=  (0x01<<ch)
#define ADC_DISABLE_CHANNEL(ch)  ADCCFG &= ~(0x01<<ch)



/******************************************************************************
* @fn  halAdcSampleSingle
*
* @brief
*      This function makes the adc sample the given channel at the given
*      resolution with the given reference.
*
* Parameters:
*
* @param BYTE reference
*          The reference to compare the channel to be sampled.
*        BYTE resolution
*          The resolution to use during the sample (8, 10, 12 or 14 bit)
*        BYTE input
*          The channel to be sampled.
*
* @return INT16
*          The conversion result
*
******************************************************************************/
INT16 halAdcSampleSingle(BYTE reference, BYTE resolution, UINT8 input);



/******************************************************************************
* @fn  halGetAdcValue
*
* @brief
*      Returns the result of the last ADC conversion.
*
* Parameters:
*
* @param  void
*
* @return INT16
*         The ADC value
*
******************************************************************************/
INT16 halGetAdcValue(void);


/******************************************************************************
*******************      Power and clock management        ********************
*******************************************************************************

These macros are used to set power-mode, clock source and clock speed.

******************************************************************************/

// Macro for getting the clock division factor
#define CLKSPD  (CLKCONCMD & 0x07)

// Macro for getting the timer tick division factor.
#define TICKSPD ((CLKCONCMD & 0x38) >> 3)

// Macro for checking status of the crystal oscillator
#define XOSC_STABLE (SLEEP & 0x40)

// Macro for checking status of the high frequency RC oscillator.
#define HIGH_FREQUENCY_RC_OSC_STABLE    (SLEEP & 0x20)


// Macro for setting power mode
#define SET_POWER_MODE(mode)                   \
  do {                                        \
    if(mode == 0)        { SLEEP &= ~0x03; } \
      else if (mode == 3)  { SLEEP |= 0x03;  } \
      else { SLEEP &= ~0x03; SLEEP |= mode;  } \
        PCON |= 0x01;                            \
          asm("NOP");                              \
   }while (0)


// Where _mode_ is one of
#define POWER_MODE_0  0x00  // Clock oscillators on, voltage regulator on
#define POWER_MODE_1  0x01  // 32.768 KHz oscillator on, voltage regulator on
#define POWER_MODE_2  0x02  // 32.768 KHz oscillator on, voltage regulator off
#define POWER_MODE_3  0x03  // All clock oscillators off, voltage regulator off


#define HAL_SLEEP_OFF           POWER_MODE_0
#define HAL_SLEEP_TIMER       POWER_MODE_2
#define HAL_SLEEP_DEEP         POWER_MODE_3


/* MAX_SLEEP_TIME calculation:
*   Sleep timer maximum duration = 0xFFFFFF / 32768 Hz = 511.996 seconds
*   Round it to 510 seconds or 510000 ms
*/
#define MAX_SLEEP_TIME                   510000             /* maximum time to sleep allowed by ST */



// Macro for setting the 32 KHz clock source
#define SET_32KHZ_CLOCK_SOURCE(source) \
  do {                                \
    if( source ) {                   \
      CLKCONCMD |= 0x80;               \
      } else {                         \
        CLKCONCMD &= ~0x80;              \
      }                                \
   } while (0)

// Where _source_ is one of
#define CRYSTAL 0x00
#define RC      0x01

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#define st(x)      do { x } while (__LINE__ == -1)

#define XOSC_STB   BV(6)  /* XOSC: powered, stable=1 */
#define CLKCONCMD_32MHZ  (0)
 #define OSC_32KHZ  0x80 /* internal 32 KHz rcosc */


// Macro for setting the main clock oscillator source,
//turns off the clock source not used
//changing to XOSC will take approx 150 us
#define SET_MAIN_CLOCK_SOURCE()                                         \
{                                                                \
  UINT16 i;                                                      \
                                                                 \
  SLEEPCMD &= ~0x04;                       /* turn on 16MHz RC and 32MHz XOSC */                \
  while (!(SLEEPSTA & XOSC_STB));            /* wait for 32MHz XOSC stable */                     \
  asm("NOP");                                /* chip bug workaround */                            \
  for (i=0; i<504; i++) asm("NOP");          /* Require 63us delay for all revs */                \
  CLKCONCMD = (CLKCONCMD_32MHZ | OSC_32KHZ); /* Select 32MHz XOSC and the source for 32K clock */ \
  while (CLKCONSTA != (CLKCONCMD_32MHZ | OSC_32KHZ)); /* Wait for the change to be effective */   \
  SLEEPCMD |= 0x04;                        /* turn off 16MHz RC */                              \
                                                                    \
}

/* This value is used to adjust the sleep timer compare value such that the sleep timer
* compare takes into account the amount of processing time spent in function halSleep().
* The first value is determined by measuring the number of sleep timer ticks it from
* the beginning of the function to entering sleep mode.  The second value is determined
* by measuring the number of sleep timer ticks from exit of sleep mode to the call to
* osal_adjust_timers().
*/
//#define HAL_SLEEP_ADJ_TICKS   (9 + 25)
#define HAL_SLEEP_ADJ_TICKS   30     //test result  0x1D--0x20   mostly 0x1E

#define UINT32_NDX0   0
#define UINT32_NDX1   1
#define UINT32_NDX2   2
#define UINT32_NDX3   3

#define T2M_OVF_BITS    (BV(6) | BV(5) | BV(4))
#define T2M_BITS        (BV(2) | BV(1) | BV(0))

#define T2M_OVFSEL(x)   ((x) << 4)
#define T2M_SEL(x)      (x)

#define T2M_T2OVF       T2M_OVFSEL(0)
#define T2M_T2OVF_CAP   T2M_OVFSEL(1)
#define T2M_T2OVF_PER   T2M_OVFSEL(2)
#define T2M_T2OVF_CMP1  T2M_OVFSEL(3)
#define T2M_T2OVF_CMP2  T2M_OVFSEL(4)

#define T2M_T2TIM       T2M_SEL(0)
#define T2M_T2_CAP      T2M_SEL(1)
#define T2M_T2_PER      T2M_SEL(2)
#define T2M_T2_CMP1     T2M_SEL(3)
#define T2M_T2_CMP2     T2M_SEL(4)

#define MAC_RADIO_SET_CHANNEL(x)                      st( FREQCTRL = FREQ_2405MHZ + 5 * ((x) - 11); )

#define MAC_RADIO_TIMER_SLEEP()                       st(T2CTRL &= ~TIMER2_RUN; while(  T2CTRL & TIMER2_STATE);)
#define MAC_RADIO_TIMER_WAKE_UP()                     st(T2CTRL |=  TIMER2_RUN; while(!(T2CTRL & TIMER2_STATE));)

#define MAC_MCU_T2_ACCESS_OVF_COUNT_VALUE()   st( T2MSEL = T2M_T2OVF; )
#define MAC_MCU_T2_ACCESS_OVF_CAPTURE_VALUE() st( T2MSEL = T2M_T2OVF_CAP; )
#define MAC_MCU_T2_ACCESS_OVF_PERIOD_VALUE()  st( T2MSEL = T2M_T2OVF_PER; )
#define MAC_MCU_T2_ACCESS_OVF_CMP1_VALUE()    st( T2MSEL = T2M_T2OVF_CMP1; )
#define MAC_MCU_T2_ACCESS_OVF_CMP2_VALUE()    st( T2MSEL = T2M_T2OVF_CMP2; )

#define MAC_MCU_T2_ACCESS_COUNT_VALUE()       st( T2MSEL = T2M_T2TIM; )
#define MAC_MCU_T2_ACCESS_CAPTURE_VALUE()     st( T2MSEL = T2M_T2_CAP; )
#define MAC_MCU_T2_ACCESS_PERIOD_VALUE()      st( T2MSEL = T2M_T2_PER; )
#define MAC_MCU_T2_ACCESS_CMP1_VALUE()        st( T2MSEL = T2M_T2_CMP1; )
#define MAC_MCU_T2_ACCESS_CMP2_VALUE()        st( T2MSEL = T2M_T2_CMP2; )

#define MAC_MCU_CONFIG_CSP_EVENT1()           st( T2CSPCFG = 1; )

#define MAC_RADIO_SET_PAN_ID(x)                       st( PAN_ID0 = (x) & 0xFF; PAN_ID1 = (x) >> 8; )
#define MAC_RADIO_SET_SHORT_ADDR(x)                   st( SHORT_ADDR0 = (x) & 0xFF; SHORT_ADDR1 = (x) >> 8; )

#define MAC_RADIO_TURN_ON_RX_FRAME_FILTERING()        st( FRMFILT0 |=  FRAME_FILTER_EN; )
#define MAC_RADIO_TURN_OFF_RX_FRAME_FILTERING()       st( FRMFILT0 &= ~FRAME_FILTER_EN; )

#define MAC_RADIO_SRC_MATCH_INIT_EXTPENDEN()          st( SRCEXTPENDEN0 = 0; \
                                                          SRCEXTPENDEN1 = 0; \
                                                          SRCEXTPENDEN2 = 0; )
#define MAC_RADIO_SRC_MATCH_INIT_SHORTPENDEN()        st( SRCSHORTPENDEN0 = 0; \
                                                          SRCSHORTPENDEN1 = 0; \
                                                          SRCSHORTPENDEN2 = 0; )
typedef unsigned char halIntState_t;
#define HAL_ENTER_CRITICAL_SECTION(x)   st( x = EA;  HAL_DISABLE_INTERRUPTS(); )
#define HAL_EXIT_CRITICAL_SECTION(x)    st( EA = x; )
#define HAL_CRITICAL_STATEMENT(x)       st( halIntState_t _s; HAL_ENTER_CRITICAL_SECTION(_s); x; HAL_EXIT_CRITICAL_SECTION(_s); )
#define HAL_DISABLE_INTERRUPTS()        st( EA = 0; )






#define MAC_RADIO_SET_RX_THRESHOLD(x)                 st( FIFOPCTRL = ((x)-1); )

#define MAC_RADIO_ENABLE_RX_THRESHOLD_INTERRUPT()     MAC_MCU_FIFOP_ENABLE_INTERRUPT()
#define MAC_RADIO_DISABLE_RX_THRESHOLD_INTERRUPT()    MAC_MCU_FIFOP_DISABLE_INTERRUPT()
#define MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG() MAC_MCU_FIFOP_CLEAR_INTERRUPT()

#define CSP_STOP_AND_CLEAR_PROGRAM()          st( RFST = ISSTOP; RFST = ISCLEAR; )
#define CSP_START_PROGRAM()                   st( RFST = ISSTART; )


#define FRMCTRL0_RESET_VALUEL   0x40

/* FRMCTRL1 */
#define PENDING_OR                    BV(2)

/* FRMFILT0 */
#define PAN_COORDINATOR               BV(1)
#define FRAME_FILTER_EN               BV(0)

/* FSCTRLL */
#define FREQ_2405MHZ                  0x0B
#define CORR_THR                      0x14

/* CCACTRL0 */
#define CCA_THR                       0xF8

/* TXFILTCFG */
#define TXFILTCFG                     XREG( 0x61FA )
#define TXFILTCFG_RESET_VALUE         0x09

/* FRMCTRL0 */
#define FRMCTRL0_RESET_VALUE          0x40
#define AUTOACK                       BV(5)
#define RX_MODE(x)                    ((x) << 2)
#define RX_MODE_INFINITE_RECEPTION    RX_MODE(2)
#define RX_MODE_NORMAL_OPERATION      RX_MODE(0)

/* FRMCTRL1 */
#define PENDING_OR                    BV(2)
#define SET_RXENMASK_ON_TX    BV(0)

/* IEN2 */
#define RFIE                          BV(0)

/* IEN0 */
#define RFERRIE                       BV(0)

/* T2CTRL */
#define LATCH_MODE            BV(3)
#define TIMER2_STATE          BV(2)
#define TIMER2_SYNC           BV(1)
#define TIMER2_RUN            BV(0)

/* T2IRQF */
#define TIMER2_OVF_COMPARE2F  BV(5)
#define TIMER2_OVF_COMPARE1F  BV(4)
#define TIMER2_OVF_PERF       BV(3)
#define TIMER2_COMPARE2F      BV(2)
#define TIMER2_COMPARE1F      BV(1)
#define TIMER2_PERF           BV(0)

/* T2IRQM */
#define TIMER2_OVF_COMPARE2M  BV(5)
#define TIMER2_OVF_COMPARE1M  BV(4)
#define TIMER2_OVF_PERM       BV(3)
#define TIMER2_COMPARE2M      BV(2)
#define TIMER2_COMPARE1M      BV(1)
#define TIMER2_PERM           BV(0)

/* T2PEROF2 */
#define CMPIM           BV(7)
#define PERIM           BV(6)
#define OFCMPIM         BV(5)
#define PEROF2_BITS     (BV(3) | BV(2) | BV(1) | BV(0))

/* RFIRQF0 */
#define IRQ_SFD         BV(1)
#define IRQ_FIFOP       BV(2)
#define IRQ_RXPKTDONE     BV(6)

/* RFIRQF1 */
#define IRQ_TXACKDONE   BV(0)
#define IRQ_TXDONE      BV(1)
#define IRQ_CSP_MANINT  BV(3)
#define IRQ_CSP_STOP    BV(4)

/* RFIRQM0 */
#define IM_SFD          BV(1)
#define IM_FIFOP        BV(2)

/* RFIRQM1 */
#define IM_TXACKDONE    BV(0)
#define IM_TXDONE       BV(1)
#define IM_CSP_MANINT   BV(3)
#define IM_CSP_STOP     BV(4)

/* IRQSRC */
#define TXACK           BV(0)

/* RFERRM and RFERRF */
#define RFERR_RXOVERF   BV(2)

/*T3CTL*/
#define T3CTL_DIV      BV(7) | BV(6) | BV(5)
#define T3CTL_START BV(4)
#define T3CTL_OVFIM BV(3)
#define T3CTL_CLR     BV(2)
#define T3CTL_MODE  BV(1) | BV(0)
/*TIMIF*/
#define TIMIF_T3CH1IF BV(2)
#define TIMIF_T3CH0IF BV(1)
#define TIMIF_T3OVFIF BV(0)

/* ------------------------------------------------------------------------------------------------
 *                                       Interrupt Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MAC_MCU_WRITE_RFIRQF0(x)      HAL_CRITICAL_STATEMENT( S1CON = 0x00; RFIRQF0 = x; )
#define MAC_MCU_WRITE_RFIRQF1(x)      HAL_CRITICAL_STATEMENT( S1CON = 0x00; RFIRQF1 = x; )
#define MAC_MCU_OR_RFIRQM0(x)         st( RFIRQM0 |= x; )  /* compiler must use atomic ORL instruction */
#define MAC_MCU_AND_RFIRQM0(x)        st( RFIRQM0 &= x; )  /* compiler must use atomic ANL instruction */
#define MAC_MCU_OR_RFIRQM1(x)         st( RFIRQM1 |= x; )  /* compiler must use atomic ORL instruction */
#define MAC_MCU_AND_RFIRQM1(x)        st( RFIRQM1 &= x; )  /* compiler must use atomic ANL instruction */

#define MAC_MCU_SFD_ENABLE_INTERRUPT()              MAC_MCU_OR_RFIRQM0(IM_SFD)

#define MAC_MCU_FIFOP_ENABLE_INTERRUPT()              MAC_MCU_OR_RFIRQM0(IM_FIFOP)
#define MAC_MCU_FIFOP_DISABLE_INTERRUPT()             MAC_MCU_AND_RFIRQM0(~IM_FIFOP)
#define MAC_MCU_FIFOP_CLEAR_INTERRUPT()               MAC_MCU_WRITE_RFIRQF0(~IRQ_FIFOP)
#define MAC_MCU_SFD_CLEAR_INTERRUPT()                   MAC_MCU_WRITE_RFIRQF0(~IRQ_SFD)

#define MAC_MCU_TXACKDONE_ENABLE_INTERRUPT()          MAC_MCU_OR_RFIRQM1(IM_TXACKDONE)
#define MAC_MCU_TXACKDONE_DISABLE_INTERRUPT()         MAC_MCU_AND_RFIRQM1(~IM_TXACKDONE)
#define MAC_MCU_TXACKDONE_CLEAR_INTERRUPT()           MAC_MCU_WRITE_RFIRQF1(~IRQ_TXACKDONE)

#define MAC_MCU_TXDONE_ENABLE_INTERRUPT()          MAC_MCU_OR_RFIRQM1(IM_TXDONE)
#define MAC_MCU_TXDONE_DISABLE_INTERRUPT()         MAC_MCU_AND_RFIRQM1(~IM_TXDONE)
#define MAC_MCU_TXDONE_CLEAR_INTERRUPT()           MAC_MCU_WRITE_RFIRQF1(~IRQ_TXDONE)

#define MAC_MCU_CSP_STOP_ENABLE_INTERRUPT()           MAC_MCU_OR_RFIRQM1(IM_CSP_STOP)
#define MAC_MCU_CSP_STOP_DISABLE_INTERRUPT()          MAC_MCU_AND_RFIRQM1(~IM_CSP_STOP)
#define MAC_MCU_CSP_STOP_CLEAR_INTERRUPT()            MAC_MCU_WRITE_RFIRQF1(~IRQ_CSP_STOP)
#define MAC_MCU_CSP_STOP_INTERRUPT_IS_ENABLED()       (RFIRQM1 & IM_CSP_STOP)

#define MAC_MCU_CSP_INT_ENABLE_INTERRUPT()            MAC_MCU_OR_RFIRQM1(IM_CSP_MANINT)
#define MAC_MCU_CSP_INT_DISABLE_INTERRUPT()           MAC_MCU_AND_RFIRQM1(~IM_CSP_MANINT)
#define MAC_MCU_CSP_INT_CLEAR_INTERRUPT()             MAC_MCU_WRITE_RFIRQF1(~IRQ_CSP_MANINT)
#define MAC_MCU_CSP_INT_INTERRUPT_IS_ENABLED()        (RFIRQM1 & IM_CSP_MANINT)

#define MAC_MCU_RFERR_ENABLE_INTERRUPT()              st( RFERRM |=  RFERR_RXOVERF; )
#define MAC_MCU_RFERR_DISABLE_INTERRUPT()             st( RFERRM &= ~RFERR_RXOVERF; )


#define DRV_SLEEP_TIMER_CLEAR_INT( ) do{IRCON &= ~0x80;} while(0)
#define DRV_CLEAR_SLEEP_MODE() do{SLEEPCMD &= ~0x03;} while(0)
#endif //HAL_H


