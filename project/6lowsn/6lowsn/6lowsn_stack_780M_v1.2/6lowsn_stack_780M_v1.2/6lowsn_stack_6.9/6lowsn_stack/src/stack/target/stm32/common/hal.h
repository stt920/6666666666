/*

CC2530底层驱动文件

*/


#ifndef HAL_H
#define HAL_H
#include <stdio.h>
#include <string.h>
#include "compiler.h"
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

#define HAL_SUSPEND(x)         //dummy in uC, only needed for Win32

//Timer Support
//We set the timer to exactly 1 tick per symbol
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
#define INT_GLOBAL_ENABLE(on) 
#define SAVE_AND_DISABLE_GLOBAL_INTERRUPT(x) 
#define RESTORE_GLOBAL_INTERRUPT(x) 
#define ENABLE_GLOBAL_INTERRUPT() 
#define DISABLE_GLOBAL_INTERRUPT() 

#define DISABLE_ALL_INTERRUPTS() 

#define PACKET_FOOTER_SIZE 2//2    //负载之后的位数   //2017.4


#endif //HAL_H


