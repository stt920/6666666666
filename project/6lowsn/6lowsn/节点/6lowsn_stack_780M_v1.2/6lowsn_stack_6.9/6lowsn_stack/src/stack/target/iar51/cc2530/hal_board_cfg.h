

#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H



#include "hal.h"
#include "compiler.h"



#define HAL_BOARD_CC2430DB

#define HAL_CPU_CLOCK_MHZ     32

#define HAL_NUM_LEDS            2
#define HAL_LED_BLINK_DELAY()   st( { volatile uint32 i; for (i=0; i<0x5800; i++) { }; } )

/* D1 - Green */
#define LED1_BV           BV(0)
#define LED1_SBIT         P1_0
#define LED1_DDR          P1DIR
#define LED1_POLARITY     ACTIVE_LOW

/* D2 - Red */
#define LED2_BV           BV(1)
#define LED2_SBIT         P1_1
#define LED2_DDR          P1DIR
#define LED2_POLARITY     ACTIVE_LOW


#define ACTIVE_LOW        !
#define ACTIVE_HIGH       !!    /* double negation forces result to be '1' */

/* S1 */
#define PUSH1_BV          BV(1)
#define PUSH1_SBIT        P0_1
#define PUSH1_POLARITY    ACTIVE_LOW

/* Joystick Center Press */
#define PUSH2_BV          BV(0)
#define PUSH2_SBIT        P2_0
#define PUSH2_POLARITY    ACTIVE_HIGH


/* ------------------------------------------------------------------------------------------------
*                                            Macros
* ------------------------------------------------------------------------------------------------
*/

/* ----------- Board Initialization ---------- */
#define VDD_SW_BV         BV(2)
#define VDD_SW_SBIT       P1_2
#define VDD_SW_DDR        P1DIR

#define HAL_BOARD_INIT()                                         \
  {                                                                \
    /* set direction for GPIO outputs  */                          \
      LED1_DDR |= LED1_BV;                                           \
        LED2_DDR |= LED2_BV;                                           \
          \
            /* configure tristates */                                      \
              P2INP |= PUSH2_BV;                                             \
                \
                  /* configure software controlled peripheral VDD */             \
                    VDD_SW_DDR |= VDD_SW_BV;                                       \
                      VDD_SW_SBIT = 0;                                               \
}

/* ----------- Debounce ---------- */
#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }

/* ----------- Push Buttons ---------- */
#define HAL_PUSH_BUTTON1()        (PUSH1_POLARITY (PUSH1_SBIT))
#define HAL_PUSH_BUTTON2()        (PUSH2_POLARITY (PUSH2_SBIT))
#define HAL_PUSH_BUTTON3()        (0)
#define HAL_PUSH_BUTTON4()        (0)
#define HAL_PUSH_BUTTON5()        (0)
#define HAL_PUSH_BUTTON6()        (0)

/* ----------- LED's ---------- */
#define HAL_TURN_OFF_LED1()       st( LED1_SBIT = LED1_POLARITY (0); )
#define HAL_TURN_OFF_LED2()       st( LED2_SBIT = LED2_POLARITY (0); )
#define HAL_TURN_OFF_LED3()       HAL_TURN_OFF_LED2()
#define HAL_TURN_OFF_LED4()       HAL_TURN_OFF_LED1()

#define HAL_TURN_ON_LED1()        st( LED1_SBIT = LED1_POLARITY (1); )
#define HAL_TURN_ON_LED2()        st( LED2_SBIT = LED2_POLARITY (1); )
#define HAL_TURN_ON_LED3()        HAL_TURN_ON_LED2()
#define HAL_TURN_ON_LED4()        HAL_TURN_ON_LED1()

#define HAL_TOGGLE_LED1()         st( if (LED1_SBIT) { LED1_SBIT = 0; } else { LED1_SBIT = 1;} )
#define HAL_TOGGLE_LED2()         st( if (LED2_SBIT) { LED2_SBIT = 0; } else { LED2_SBIT = 1;} )
#define HAL_TOGGLE_LED3()         HAL_TOGGLE_LED2()
#define HAL_TOGGLE_LED4()         HAL_TOGGLE_LED1()

#define HAL_STATE_LED1()          (LED1_POLARITY (LED1_SBIT))
#define HAL_STATE_LED2()          (LED2_POLARITY (LED2_SBIT))
#define HAL_STATE_LED3()          HAL_STATE_LED2()
#define HAL_STATE_LED4()          HAL_STATE_LED1()

/* ------------------------------------------------------------------------------------------------
*                                     Driver Configuration
* ------------------------------------------------------------------------------------------------
*/

/* Set to TRUE enable ADC usage, FALSE disable it */
#ifndef HAL_ADC
#define HAL_ADC TRUE
#endif

/* Set to TRUE enable DMA usage, FALSE disable it */
#ifndef HAL_DMA
#define HAL_DMA TRUE
#endif

/* Set to TRUE enable AES usage, FALSE disable it */
#ifndef HAL_AES
#define HAL_AES TRUE
#endif

/* Set to TRUE enable LCD usage, FALSE disable it */
#ifndef HAL_LCD
#define HAL_LCD TRUE
#endif

/* Set to TRUE enable LED usage, FALSE disable it */
#ifndef HAL_LED
#define HAL_LED TRUE
#endif
#if (!defined BLINK_LEDS) && (HAL_LED == TRUE)
#define BLINK_LEDS
#endif

/*
Set to TRUE enable KEY usage, FALSE disable it
Notes: On 2430EB/DB analog joystick is used to simulate
keys. Keys won't work unless HAL_ADC is also set
to TRUE
*/
#ifndef HAL_KEY
#define HAL_KEY TRUE
#endif

/* Set to TRUE enable UART usage, FALSE disable it */
#ifndef HAL_UART
#if (defined ZAPP_P1) || (defined ZAPP_P2) || (defined ZTOOL_P1) || (defined ZTOOL_P2)
#define HAL_UART TRUE
#else
#define HAL_UART FALSE
#endif /* ZAPP, ZTOOL */
#endif /* HAL_UART */

#if HAL_UART
#define HAL_UART_0_ENABLE  FALSE
#define HAL_UART_1_ENABLE  TRUE

#if HAL_DMA
// The DB is forced to still use ISR since DMA cannot be tested on less than Rev-D chips and all available DB's are older.
//  #if !defined( HAL_UART_DMA )
//    /* Can only run DMA on one USART or the other, not both at the same time.
//     * So define to 1 for USART0, 2 for USART1, or 0 for neither.
//     */
// TBD   #define HAL_UART_DMA  2
//  #endif
//#else
//  #undef  HAL_UART_DMA
#define HAL_UART_DMA 0
#endif

/* The preferred method of implementation is by DMA for faster buad rate
* support. Customer may prefer to use the DMA channels for something else,
* in which case USART can be driven by ISR. Also, if the 2nd USART is to be
* used, this module does not currently support using 2 more DMA channels for
* it, so it must use ISR.
*/
#if !defined( HAL_UART_ISR )
#define HAL_UART_ISR     TRUE  // TBD FALSE
#endif

/* It is not common for embedded applications to open/close/re-open a USART,
* so make the close function optional.
*/
#if !defined( HAL_UART_CLOSE )
#define HAL_UART_CLOSE  FALSE
#endif
#else
#define HAL_UART_0_ENABLE  FALSE
#define HAL_UART_1_ENABLE  FALSE
#define HAL_UART_DMA       FALSE
#define HAL_UART_ISR       FALSE
#define HAL_UART_CLOSE     FALSE
#endif

/*******************************************************************************************************
*/
#endif

