

#ifndef CONSOLE_H
#define CONSOLE_H

#include "compiler.h"
#include "6lowsn_config.h" 
#include "console.h"
#include "ds.h"

void conPrintROMString_func(ROMCHAR *s);
void conPrintUINT8 (UINT8 x);
void conPrintUINT16 (UINT16 x);
void conPrintUINT32 (UINT32 x);
void conPrintLADDR(LADDR *laddr); //print long address
void conPrintLADDR_bytes(BYTE *ptr); //print long address
void conPrintConfig(void);
void conPCRLF(void);
void conPrintString (char *s);
void conPrintIP6ADDR_bytes(BYTE *ptr);
void conPrintIP6ADDR_dbytes(BYTE *ptr);
void conPrintIP6ADDR(IPADDR *ipaddr, UINT8 type);

//strings passed into this macro must be ROM constants

#ifdef LOWSN_SLIP_COM_QUIET

#define conPrintROMString(s)
/***************************2014-11-18 zdp*****************************/
#define conPrintROMString_new(s) \
  {\
    static  ROMCHAR xxxxromstr[] = s;\
    conPrintROMString_func_new(xxxxromstr);\
  }
/**********************************************************************/
#else
#define conPrintROMString(s) \
  {\
    static  ROMCHAR xxxxromstr[] = s;\
    conPrintROMString_func(xxxxromstr);\
  }
#endif

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#endif


#endif

