


/*
This is a router test, using direct messages from
RFD to coordinator through router.

Expects coordinator, one router, at least one RFD.
The topology to test should be:

Coordinator ->  router -> RFD1
-> RFD2 (optional)


Start the coordinator first, then the router, then
RFD1. If a RFD1 fails to join the network, try
again. The usrJoinVerifyCallback() is written such
that the Coordinator will reject the RFDs, so that
the RFDs should join the router. Can add more than
one RFD.

YOU MUST EDIT the coord[] long address and put
in the long address of your coordinator so the
test that uses long addressing will work correctly.

The usrJoinVerify function at the coordinator level
rejects any node that is not a router, so this forces
routers to join the coordinator, and RFDs to join the
router.

This does not require any Virtual Boards to be running,
everything done through the console.

For WIN32, the Router project also has LOWSN_RSSI defined
as a stronger value than the default (see util_get_rssi()
in halStack.c), so that packets
from the router appear to be closer in physical distance
than from other nodes, since RFDs use the RSSI value to
choose a node to respond to when they get multiple responses
from a beacon request.


In WIN32, start the coord first, then the router so that it
joins the coord, then RFDs so that they join the router.

*/

#include "6lowsn.h"

BYTE payload[32];  //buffer for payload
BYTE test_number;
LADDR_UNION dstADDR;
UINT32 my_timer;
IPADDR dstIPADDR;
IPADDR network_prefix;
UINT8 prefix_len;

//  EUI64地址使用示例(本例子中未使用长地址做目标地址)
// 按阅读顺序排列，注意第一个字节的LSB倒数第2位必须为0，即U/L原始位为0.
// 倒数第1位也为0，表示是单播。建议第一个字节为0x00.
// 可以用buf设定目标节点的EUI64地址，若要设定节点自己的EUI64，首选在IAR的工程文件编译选项
// 中的预配置中进行定义，若没有在编译参数中定义，则还可在6lowsn_config.h中
// 用aExtendedAddress_B0-aExtendedAddress_B7宏设定。若编译参数中已经定义，在在文件中定义，
// 就会出现重定义的警告。

UINT8 coordEUI[8] = {0x00,0x10,0x20,0x30,0x40,0x50,0x51,0x00};
UINT8 rfdEUI[8] = {0x00,0x10,0x20,0x30,0x40,0x50,0x53,0x00};


void getpayload(void);

void print_test(void){
    conPrintROMString("Test ");
    conPrintUINT8((UINT8) test_number);
     conPrintROMString(", Sending msg: ");
     conPrintString((char *)&payload[0]);
	conPCRLF();
}


//this is assumed to be the long address of our coordinator, in little endian order
//used to test LONG ADDRESSING back to coordinator
UINT8 coord[8] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x88};


#define NUM_MESSAGES 8
//convoluted mess because of different ways compilers treat const char strings
void getpayload(void) {
   BYTE *dst;
   ROMCHAR *src;
   BYTE msgnum;

   static ROMCHAR _x0_[] = "Beijing ";
   static ROMCHAR _x1_[] = "Shanghai ";
   static ROMCHAR _x2_[] = "Tianjin ";
   static ROMCHAR _x3_[] = "Chongqing ";
   static ROMCHAR _x4_[] = "Shijiazhuang ";
   static ROMCHAR _x5_[] = "Wuhan ";
   static ROMCHAR _x6_[] = "Guangzhou ";
   static ROMCHAR _x7_[] = "Shenzhen ";

   msgnum = halGetRandomByte();
   msgnum = msgnum % NUM_MESSAGES;
	
   switch (msgnum) {
     case 0 :  src = &_x0_[0]; break;
     case 1 :  src = &_x1_[0]; break;
     case 2 :  src = &_x2_[0]; break;
     case 3 :  src = &_x3_[0]; break;
     case 4 :  src = &_x4_[0]; break;
     case 5 :  src = &_x5_[0]; break;
     case 6 :  src = &_x6_[0]; break;
     default :  src = &_x7_[0]; break;
   }
    dst = &payload[0];
    while (*src) {
        *dst = *src;
         dst++;src++;
    }
   *dst = *src;
}


void packet_test(void) {

	//switch(test_number) {
	  //case 0:
                getpayload();
		  print_test();
				
		  conPrintROMString("Tsetmsg: Send packet to :\n");
		  conPrintIP6ADDR(&dstIPADDR, 1);
		  conPrintROMString("\n");

		  aplSendPublishData(dstIPADDR,
			0xF0B2,
			0, 
			0xF0B3,
			0, 
			apsGenServiceID(), 
			1, 
			 payload,
			 strlen((char *)payload)+1); 
		  //test_number++;
		//  break;

	//block, and see if message sent successfully
	while(apsBusy()) {apsFSM();}
	if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
		conPrintROMString("MSG send succeeded!\n");
	}else {
		conPrintROMString("MSG send FAILED!\n");
	}


}

void main (void){


	//this initialization set our SADDR to 0xFFFF,
	//PANID to the default PANID

	halInit();
	evbInit();

	aplInit();  //init the stack
	conPrintConfig();
	ENABLE_GLOBAL_INTERRUPT();  //enable interrupts


	EVB_LED1_OFF();
	EVB_LED2_OFF();
	
	test_number = 0;

	debug_level = DBG_MAX_LEVEL;


#ifdef LOWSN_COORDINATOR

	aplFormNetwork();
	while(apsBusy()) {apsFSM();} //wait for finish

	conPrintROMString("Nwk formed, waiting for join and reception\n");

	lowsn_ip6addr(&network_prefix,0x0066,0x0077,0x1234,0x5678,0,0,0,0);
	prefix_len = LOWSN_DEFAULT_PREFIX_LEN;
	aplSetPrefix(&network_prefix, prefix_len);
	conPrintROMString("Set global prefix:  ");
	conPrintIP6ADDR(&network_prefix, 1);
	conPrintROMString("  Prefix Len: ");
	conPrintUINT8(prefix_len);
	conPrintROMString("\n");
	
	while(1) {apsFSM();}

#else
	do {
		conPrintROMString("Begin to join a network. \n");
		aplJoinNetwork();
		while(apsBusy()) {apsFSM();} //wait for finish
		if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
			conPrintROMString("Network Join succeeded!\n");
			conPrintROMString("My ShortAddress is: ");
			conPrintUINT16(aplGetMyShortAddress());
			conPCRLF();
			conPrintROMString("Parent LADDR: ")
			conPrintLADDR(aplGetParentLongAddress());
			conPrintROMString(", Parent SADDR: ");
			conPrintUINT16(aplGetParentShortAddress());
			conPCRLF();
			break;
		}else {
			conPrintROMString("Network Join FAILED! Waiting, then trying again\n");
                        my_timer= halGetMACTimer();
                       //wait for 2 seconds
                      while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(2*1000));
		       }
	} while(1);


	do {
		conPrintROMString("Begin to get a gloabal prefix. \n");
		aplGetPrefix();
		while(apsBusy()) {apsFSM();} //wait for finish
		if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
			conPrintROMString("Get a global prefix!\n");
			break;
		}else {
			conPrintROMString("Prefix obtaining FAILED! Waiting, then trying again\n");
                        my_timer= halGetMACTimer();
                       //wait for 2 seconds
                      while ((halMACTimerNowDelta(my_timer))< MSECS_TO_MACTICKS(2*1000));
		       }
	} while(1);



#ifdef LOWSN_RFD
	//now send packets
	while (1) {
		packet_test();
		while(apsBusy()) {apsFSM();} //wait for finish
	}
#endif
#ifdef LOWSN_ROUTER
	//router does nothing, just routes
	DEBUG_PRINTNEIGHBORS(DBG_INFO);
	conPrintROMString("Router, doing its thing.!\n");
	while(1) {apsFSM();}
#endif

#endif


}


//callback from APS when packet is received
//user must do something with data as it is freed
//within the stack upon return.

LOWSN_STATUS_ENUM usrRxPacketCallback(void) {

	BYTE len, *ptr;
        IPADDR srcIP;

	//just print out this data

	conPrintROMString("User Data Packet Received: \n");
	conPrintROMString("SrcSADDR: ");
	conPrintUINT16(aplGetRxSrcSADDR());

	conPrintROMString(", DstPort: ");
	conPrintUINT16(aplGetRxDstPort());

	conPrintROMString(", SrcPort: ");
	conPrintUINT16(aplGetRxSrcPort());

	srcIP = aplGetRxSrcIPAddr();

	conPrintROMString("\n Source IP is : ");
	conPrintIP6ADDR(&srcIP, 1);
	conPrintROMString("\n");
	
	conPrintROMString(", Msg Length: ");
	len = aplGetRxMsgLen();
	conPrintUINT8(len);
	conPCRLF();
	conPrintROMString("Msg: ");
	ptr = aplGetRxMsgData();
	while(len){
		halPutch(*ptr);
		ptr++; len--;
	}

	conPCRLF();
        return LOWSN_STATUS_SUCCESS;
}

#ifdef LOWSN_FFD
//Callback to user level to see if OK for this node
//to join - implement Access Control Lists here based
//upon IEEE address if desired
BOOL usrJoinVerifyCallback(LADDR *ptr, BYTE capinfo){


	return TRUE;


}

BOOL usrJoinNotifyCallback(LADDR *ptr){

	//allow anybody to join

	conPrintROMString("Node joined: ");
	conPrintLADDR(ptr);
	conPCRLF();
	DEBUG_PRINTNEIGHBORS(DBG_INFO);
	return TRUE;
}
#endif

// callback when a prefix is obtained by receving RS frame.
BOOL usrGetPrefixCallback(IPADDR *prefix_ptr, UINT8 prefix_len)
{

	conPrintROMString("Get Prefix Successfully, Prefix: ");
	conPrintIP6ADDR(prefix_ptr, 1);
	conPCRLF();
	conPrintROMString("Prefix Length: ");
	conPrintUINT8(prefix_len);
	conPCRLF();
	
	return TRUE;


}

void usrIntCallback(void){}

//called when the slow timer interrupt occurs
#ifdef LOWSN_ENABLE_SLOW_TIMER
void usrSlowTimerInt(void ) {}
#endif
