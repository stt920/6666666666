

#ifndef COMPILER_H
#define COMPILER_H




//compatible with both IAR8051 compiler

#ifdef __IAR_SYSTEMS_ICC__
#define IAR8051
#include "ioCC2530.h"

#define _PRAGMA(x) _Pragma(#x)
#define HAL_ISR_FUNC_DECLARATION(f,v)   _PRAGMA(vector=v) __near_func __interrupt void f(void)
#define HAL_ISR_FUNC_PROTOTYPE(f,v)     _PRAGMA(vector=v) __near_func __interrupt void f(void)
#define HAL_ISR_FUNCTION(f,v)           HAL_ISR_FUNC_PROTOTYPE(f,v); HAL_ISR_FUNC_DECLARATION(f,v)
#endif


/******************************************************************************
*******************              Commonly used types        *******************
******************************************************************************/
typedef unsigned char       BOOL;

// Data
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

// Unsigned numbers
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned long       UINT32;

// Signed numbers
typedef signed char         INT8;
typedef signed short        INT16;
typedef signed long         INT32;

// For IPv6
typedef char                int8_t;
typedef unsigned char       uint8_t;
typedef unsigned short      uint16_t;
typedef signed long         uint32_t;

#ifndef timer_t
//typedef unsigned long timer_t;	/*in msec*/

typedef unsigned long mstimer_t;
typedef unsigned long stimer_t;
#endif

#ifndef systime_t
//typedef unsigned long systime_t;	/*in msec*/

typedef unsigned long systime_s_t;	/*in second*/
typedef unsigned long systime_ms_t;	/*in millisecond*/
#endif

//Define these compiler labels

//uncomment this if compiler is BIG ENDIAN
//define LWRPAN_COMPILER_BIG_ENDIAN

#if defined(IAR8051)

#define INTERRUPT_FUNC __interrupt static void
//char type in ROM memory
typedef  unsigned char const __code    ROMCHAR ;

#endif

#if defined(HI_TECH_C)
#include <intrpt.h>
#define LWRPAN_COMPILER_NO_RECURSION
#define INTERRUPT_FUNC interrupt static void
typedef  unsigned char const ROMCHAR ;
#include "cdefs.h"    //this has all #defines since demo C51 disables -D flag

#endif




#endif
