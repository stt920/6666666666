

/*
This is a two node test, requires a Coordinator
and an RFD. The coordinator and node simply
ping-pong a packet back and forth, and print
out the RSSI byte.  The RFD waits before
bouncing it back, while the coordinator responds
immediately.

Expects coordinator, and one RFD.
The topology to test should be:

Coordinator ->  RFD1


Start the coordinator first, then
RFD1. If a RFD1 fails to join the network, try
again. The RFD1 will prompt the user to hit
a key to start the ping-pong.

You can connect multiple RFDs if desired.

You can also ping-pong through a router; see
the note in usrJoinVerifyCallback(). The topology
for a router would be:

coord -> router -> RFD1
-> RFD2
-> ..RFDn


This  requires Virtual Boards to be running,
since a switch press is needed to start the pinging.


*/

#include "6lowsn.h"

#ifndef LOWSN_COORDINATOR
#define PING_DELAY   2  //wait before bouncing back
#else
#define PING_DELAY   0 //coordinator does not wait
#endif

#define RX_PING_TIMEOUT     5    //seconds
//this is assumed to be the long address of our coordinator, in little endian order
//used to test LONG ADDRESSING back to coordinator


UINT16 ping_cnt;
UINT32 my_timer;
UINT32  last_tx_start;
UINT8 LcdPage = 1;

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


typedef enum _PP_STATE_ENUM {
	PP_STATE_START_RX,
	PP_STATE_WAIT_FOR_RX,
	PP_STATE_SEND,
	PP_STATE_WAIT_FOR_TX
}PP_STATE_ENUM;

PP_STATE_ENUM ppState;
BYTE rxFlag;              //set from within usrRxPacketCallback
BYTE payload[2];
UINT16 numTimeouts;
BOOL first_packet;

void  PingPong(void);

void PingPong (void ) {

	apsFSM();

	switch (ppState) {

			case  PP_STATE_START_RX:
				if (!first_packet) {
					my_timer= halGetMACTimer();
					ppState = PP_STATE_WAIT_FOR_RX;
				}else if (rxFlag) {
					//on first packet, do not start timer, just wait for a packet.
					ppState = PP_STATE_WAIT_FOR_RX;
					first_packet = FALSE;
				}
				break;

			case PP_STATE_WAIT_FOR_RX:
				//rxFlag is set from within usrRxPacketCallback
				if (rxFlag || halMACTimerNowDelta(my_timer) > MSECS_TO_MACTICKS( RX_PING_TIMEOUT *1000 )) {
					if (!rxFlag) numTimeouts++;     //got tired of waiting for a response, send again
					rxFlag = 0; //clear flag
					if (EVB_LED1_STATE()) EVB_LED1_OFF(); else EVB_LED1_ON();
					//start timer
					my_timer= halGetMACTimer();
					ppState = PP_STATE_SEND;

				}
				break;


			case PP_STATE_SEND:
				if ((halMACTimerNowDelta(my_timer))> MSECS_TO_MACTICKS(PING_DELAY*1000)){
			//		MemDump();

					//increment ping counter
					ping_cnt++; //this was value received by this node
					//received packet, ping it back
					//format the packet
					payload[0] = (BYTE) ping_cnt;
					payload[1] =  (BYTE) (ping_cnt>>8);
					ppState = PP_STATE_WAIT_FOR_TX;
					last_tx_start = halGetMACTimer();

					conPrintROMString("Pingpong: Send packet to :\n");
					conPrintIP6ADDR(&dstIPADDR, 1);
					conPrintROMString("\n");

					aplSendIPMSG(dstIPADDR,
						122,
						2, //dst EP
						0, //cluster is ignored for direct message
						121,
						1, //src EP
						&payload[0],
						2,  //msg length
						apsGenTSN());  //No APS ack requested

					ppState = PP_STATE_WAIT_FOR_TX;
				}
				break;

			case PP_STATE_WAIT_FOR_TX:
				if (apsBusy()) break; //status not ready yet if busy.
				if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
					ppState = PP_STATE_START_RX;
					//compute the latency of this TX send operation
					//aplGetLastTxTime gets the time that the LAST tx operation finished.
					//this will be the latency of the TX stack operation only if no mac retries were required
					last_tx_start = aplMacTicksToUs(aplGetLastTxTime() - last_tx_start);
					conPrintROMString("TX Stack latency(us): ");
					conPrintUINT32(last_tx_start);
					conPCRLF();
				}else {
					conPrintROMString("Pingping: Send failed! Try again\n");
					my_timer= halGetMACTimer();
					ppState = PP_STATE_SEND;
				}
				break;
	}
}



void main (void){


	//this initialization set our SADDR to 0xFFFF,
	//PANID to the default PANID

	//HalInit, evbInit will have to be called by the user

	numTimeouts = 0;
	my_timer = 0;
	first_packet = TRUE;

	halInit();
	evbInit();

	aplInit();  //init the stack
	conPrintConfig();
	ENABLE_GLOBAL_INTERRUPT();  //enable interrupts


	EVB_LED1_OFF();
	EVB_LED2_OFF();

	ping_cnt = 0;
	rxFlag = 0;
	debug_level = DBG_MAX_LEVEL;


#ifdef LOWSN_COORDINATOR

	aplFormNetwork();
	while(apsBusy()) {apsFSM();} //wait for finish
	conPrintROMString("Network is formed. \n");
	EVB_LED1_ON();

	lowsn_ip6addr(&network_prefix,0x0066,0x0077,0x1234,0x5678,0,0,0,0);
	prefix_len = LOWSN_DEFAULT_PREFIX_LEN;
	aplSetPrefix(&network_prefix, prefix_len);
	conPrintROMString("Set global prefix:  ");
	conPrintIP6ADDR(&network_prefix, 1);
	conPrintROMString("\n Prefix Len: ");
	conPrintUINT8(prefix_len);
	conPrintROMString("\n");
	
	ppState = PP_STATE_START_RX;
#else
	do {
		aplJoinNetwork();
		while(apsBusy()) {apsFSM();} //wait for finish
		if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
			EVB_LED1_ON();
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

#endif

#ifdef LOWSN_RFD
	//now send packets

	aplSetLinkLocalAddr16(&dstIPADDR, aplGetPanID(), 0);
	
	ppState = PP_STATE_SEND;
	conPrintROMString("Begin to send data!\n");

#endif

#if (defined(LOWSN_RFD) || defined(LOWSN_COORDINATOR))
	//WARNING - this is only for latency testing, max MAC retries is normally
	//set to aMaxFrameRetries (value=3) as defined in mac.h. Setting this to 0 means
	//that there will be no automatic retransmissions of frames if we do not get a MAC ACK back.
	//only do this in your normal code if you want to disable automatic retries
	aplSetMacMaxFrameRetries(0);

	while (1) {
		PingPong();
	}
#endif


#ifdef LOWSN_ROUTER
	//router does nothing, just routes
	DEBUG_PRINTNEIGHBORS(DBG_INFO);
	conPrintROMString("Router, doing its thing.!\n");
	while(1) {apsFSM();}
#endif


}

//########## Callbacks ##########


//callback from APS when packet is received
//user must do something with data as it is freed
//within the stack upon return.

LOWSN_STATUS_ENUM  usrRxPacketCallback(void) {

	BYTE len, *ptr;

	//IPADDR srcIP;
			
	conPrintROMString("User Data Packet Received: \n");
	conPrintROMString("SrcSADDR: ");
	conPrintUINT16(aplGetRxSrcSADDR());

	conPrintROMString(", DstPort: ");
	conPrintUINT16(aplGetRxDstPort());

	conPrintROMString(", SrcPort: ");
	conPrintUINT16(aplGetRxSrcPort());

	//srcIP = aplGetRxSrcIPAddr();
	conPrintROMString("\n Source IP is : ");
	conPrintIP6ADDR(&(aplGetRxSrcIPAddr()), 1);
	conPrintROMString("\n");
	
	conPrintROMString(", MsgLen: ");
	
	len = aplGetRxMsgLen();
	conPrintUINT8(len);
	
	conPrintROMString(",RSSI: ");
	conPrintUINT8(aplGetRxRSSI());

	conPCRLF();
	conPrintROMString("PingCnt: ");
	ptr = aplGetRxMsgData();
	ping_cnt = *ptr;
	ptr++;
	ping_cnt += ((UINT16)*ptr)<<8;
	conPrintUINT16(ping_cnt);

	conPrintROMString(", RxTimeouts: ");
	conPrintUINT16(numTimeouts);
	rxFlag = 1;//signal that we got a packet
	
	//use this source address as the next destination address
	dstIPADDR = aplGetRxSrcIPAddr();
	
	conPCRLF();
	return LOWSN_STATUS_SUCCESS;
}

#ifdef LOWSN_FFD
//Callback to user level to see if OK for this node
//to join - implement Access Control Lists here based
//upon IEEE address if desired
BOOL usrJoinVerifyCallback(LADDR *ptr, BYTE capinfo){\

#if 0      //set this to '1' if you want to test through a router
//only accept routers.
//only let routers join us if we are coord
#ifdef LOWSN_COORDINATOR
if (LOWSN_GET_CAPINFO_DEVTYPE(capinfo)) {
	//this is a router, let it join
	conPrintROMString("Accepting router\n");
	return TRUE;
}else {
	conPrintROMString("Rejecting non-router\n");
	return FALSE;
}
#else
return TRUE;
#endif

#else

return TRUE;

#endif

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

//called when the slow timer interrupt occurs
#ifdef LOWSN_ENABLE_SLOW_TIMER
void usrSlowTimerInt(void ) {}
#endif


//general interrupt callback , when this is called depends on the HAL layer.
void usrIntCallback(void){}
