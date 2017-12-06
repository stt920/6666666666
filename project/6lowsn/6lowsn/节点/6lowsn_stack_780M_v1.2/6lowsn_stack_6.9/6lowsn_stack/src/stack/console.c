



#include "compiler.h"               //compiler specific
#include "hal.h"
#include "6lowsn_config.h"
#include "halStack.h"
#include "console.h"
#include "6lowsn.h"

//utility print functions that do not use printf and expect ROM strings
//these assume that constants and strings are stored in code memory





#ifdef LOWSN_SLIP_COM_QUIET

void conPrintROMString_func (ROMCHAR *s)  {  }
/**************************2014-11-18******************************/
void conPrintROMString_func_new (ROMCHAR *s) {
  while(*s) {
    if (*s == '\n') halPutch('\r');
    halPutch(*s);
    s++;
  }
}
/******************************************************************/
void conPrintString (char *s)  {  }
void conPrintUINT8_noleader (UINT8 x) {  }

#else

void conPrintROMString_func (ROMCHAR *s) {
  while(*s) {
    if (*s == '\n') halPutch('\r');
    halPutch(*s);
    s++;
  }
}

void conPrintString (char *s) {
  while(*s) {
    if (*s == '\n') halPutch('\r');
    halPutch(*s);
    s++;
  }
}

void conPrintUINT8_noleader (UINT8 x) {
  BYTE c;
  c = (x>>4)& 0xf;
  if (c > 9) halPutch('A'+c-10);
   else halPutch('0'+c);
  //LSDigit
  c = x & 0xf;
  if (c > 9) halPutch('A'+c-10);
   else halPutch('0'+c);
}

#endif

void conPCRLF(void){
	conPrintROMString("\n");
}


void conPrintUINT8 (UINT8 x) {
  conPrintROMString("0x");
  conPrintUINT8_noleader(x);
}

void conPrintUINT16 (UINT16 x) {
 BYTE c;

 conPrintROMString("0x");
 c = (x >> 8);
 conPrintUINT8_noleader(c);
 c = (BYTE) x;
 conPrintUINT8_noleader(c);
}


void conPrintUINT32 (UINT32 x) {
 BYTE c;
 conPrintROMString("0x");
 c = (x >> 24);
 conPrintUINT8_noleader(c);
 c = (x >> 16);
 conPrintUINT8_noleader(c);
 c = (x >> 8);
 conPrintUINT8_noleader(c);
 c = x;
 conPrintUINT8_noleader(c);
}


#if 0
//assumed little endian
void conPrintLADDR_bytes(BYTE *ptr) {
char i;
 conPrintROMString("0x");
 for (i=8;i!=0;i--){
   conPrintUINT8_noleader(*(ptr+i-1));

  }
}
#endif


void conPrintLADDR_bytes(BYTE *ptr)
{
char i;
 for (i=0; i < 8; i++){
   conPrintUINT8_noleader(*(ptr+i));
   conPrintROMString(" : ");

  }
}


/*---------------------------------------------------------------------*

打印EUI64长地址
存储顺序: 把EUI64当做类似字符数组按照阅读顺序从低向高存储

*---------------------------------------------------------------------*/

void conPrintLADDR(LADDR *laddr)
{
 BYTE *ptr;

 ptr = &laddr->bytes[0];
 conPrintLADDR_bytes(ptr);
}


void conPrintIP6ADDR_bytes(BYTE *ptr)
{
	char i;
	for (i=0; i < 16; i++){
   		conPrintUINT8_noleader(*(ptr+i));
   		conPrintROMString(" : ");
  	}

}


void conPrintIP6ADDR_dbytes(BYTE *ptr)
{
	char i;
	for (i=0; i < 16; i=i+2){
   		conPrintUINT8_noleader(*(ptr+i));
   		conPrintUINT8_noleader(*(ptr+i+1));
   		conPrintROMString(" : ");
  	}
}


/*---------------------------------------------------------------------*

打印IPv6地址
type: 0:按字节逐一打印; 1或其他值: 按两个字节逐一打印


*---------------------------------------------------------------------*/

void conPrintIP6ADDR(IPADDR *ipaddr, UINT8 type)
{
	BYTE *ptr;

	ptr = &ipaddr->u8[0];

	if (type == 0)  {
 		conPrintIP6ADDR_bytes(ptr);
 	}	
	else
	{
		conPrintIP6ADDR_dbytes(ptr);
	}	
}



void conPrintConfig(void){
  BYTE b[8];

  conPrintROMString("\n*************************\n");
  conPrintROMString("     6LoWSN Stack \n");
  conPrintROMString("        CQUPT       \n");
  conPrintROMString("     Version  ");
  conPrintROMString(LOWSN_VERSION)
  conPCRLF();
    conPrintROMString("*************************\n");
#ifdef LOWSN_COORDINATOR
  conPrintROMString("Coordinator, ");
#endif
#ifdef LOWSN_ROUTER
  conPrintROMString("Router, ");
#endif
#ifdef LOWSN_RFD
  conPrintROMString("RFD, ");
#endif
  conPrintROMString("Address: ");
  halGetProcessorIEEEAddress(b);
  conPrintLADDR_bytes(b);

  conPCRLF();
  conPrintROMString("Default PAN: ");
  conPrintUINT32(LOWSN_DEFAULT_PANID);

  conPrintROMString(",Default Channel: ");
  conPrintUINT8(LOWSN_DEFAULT_START_CHANNEL);

  conPCRLF();
  conPCRLF();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  //printf("Wrong parameters value: file %s on line %d\r\n", file, line)
  conPrintROMString("ASSERT FAILED ! FILE: ");
  conPrintString(file);
    conPrintROMString("    LINE: ");
  conPrintUINT32(line);
   conPrintROMString(" \n");
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

