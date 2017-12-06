

#ifndef DEBUG_H
#define DEBUG_H

#include "compiler.h"
#include "6lowsn_config.h"  
#include "console.h"



//verbosity increases as the debug level increases
#define DBG_MAX_LEVEL 10
#define DBG_FSM    5
#define DBG_ITRACE 4    //any interrupts
#define DBG_INFO   3    //general information
#define DBG_TX     2    //send a character
#define DBG_ERR    1    //serious error


//single char debug
#define DBG_CHAR_TXSTART     '!'
#define DBG_CHAR_TXBUSY      '@'
#define DBG_CHAR_RXOFLOW     '+'
#define DBG_CHAR_RXRCV       '$'
#define DBG_CHAR_ACKPKT      '%'
#define DBG_CHAR_MEMFULL     '^'
#define DBG_CHAR_TXFIN       '&'
#define DBG_CHAR_OURACK      '*'
#define DBG_CHAR_MACFULL     '~'   // MAC RX is full
#define DBG_CHAR_FIFO_OFLOW      '{'


#ifdef LOWSN_DEBUG

#define DEBUG_STRING(level,s) if (debug_level >= level) conPrintROMString(s)

#ifdef LOWSN_SLIP_COM_QUIET
#define DEBUG_CHAR(level,c)
#else
#define DEBUG_CHAR(level,c) if (debug_level >= level) halPutch(c)
#endif

#define DEBUG_UINT8(level,x) if (debug_level >= level) conPrintUINT8(x)
#define DEBUG_UINT16(level,x) if (debug_level >= level) conPrintUINT16(x)
#define DEBUG_UINT32(level,x) if (debug_level >= level) conPrintUINT32(x)
#define DEBUG_PRINTNEIGHBORS(level) if (debug_level >= level) dbgPrintNeighborTable()
#define DEBUG_PRINTPACKET(level, ptr, plen) if (debug_level >= level)dbgPrintPacket(ptr, plen)

#define DEBUG_LADDR(level, lptr) if (debug_level >= level)   conPrintLADDR(lptr)
#define DEBUG_IP6ADDR(level, ipptr, type) if (debug_level >= level)  conPrintIP6ADDR(ipptr, type)



BYTE *dbgPrintMacPacket (BYTE *ptr, BYTE len);
void dbgPrintPacket(BYTE *ptr, BYTE plen);
void dbgPrintNeighborTable(void);

#define DEBUG_SET_LEVEL(x) debug_level=x

#else

#define DEBUG_STRING(level,s)
#define DEBUG_CHAR(level,c)
#define DEBUG_UINT8(level,x)
#define DEBUG_UINT16(level,x)
#define DEBUG_UINT32(level,x)
#define DEBUG_PRINTNEIGHBORS(level, x)
#define DEBUG_PRINTPACKET(level, ptr, plen)

#define DEBUG_LADDR(level, lptr) 
#define DEBUG_IP6ADDR(level, ipptr, type) 
#endif

extern BYTE debug_level;
#endif


