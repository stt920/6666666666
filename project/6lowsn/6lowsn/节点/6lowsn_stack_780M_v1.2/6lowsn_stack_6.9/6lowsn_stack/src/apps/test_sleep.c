

#include "6lowsn.h"

#ifdef LOWSN_COORDINATOR

#define ROUTER_WAIT_TIME  3   //·������Ծ�ĳ����ȴ�ʱ��
#define ROUTER_SLEEP_TIME 3  // ·����˯��ʱ��
#endif


#ifdef LOWSN_RFD
#define NODE_SLEEP_TIME  3   //�ڵ�˯��ʱ��
#endif



UINT32 my_timer;
LADDR_UNION dstADDR;

IPADDR dstIPADDR;
IPADDR network_prefix;
UINT8 prefix_len;

//  EUI64��ַʹ��ʾ��(��������δʹ�ó���ַ��Ŀ���ַ)
// ���Ķ�˳�����У�ע���һ���ֽڵ�LSB������2λ����Ϊ0����U/LԭʼλΪ0.
// ������1λҲΪ0����ʾ�ǵ����������һ���ֽ�Ϊ0x00.
// ������buf�趨Ŀ��ڵ��EUI64��ַ����Ҫ�趨�ڵ��Լ���EUI64����ѡ��IAR�Ĺ����ļ�����ѡ��
// �е�Ԥ�����н��ж��壬��û���ڱ�������ж��壬�򻹿���6lowsn_config.h��
// ��aExtendedAddress_B0-aExtendedAddress_B7���趨��������������Ѿ����壬�����ļ��ж��壬
// �ͻ�����ض���ľ��档

UINT8 coordEUI[8] = {0x00,0x10,0x20,0x30,0x40,0x50,0x51,0x00};
UINT8 rfdEUI[8] = {0x00,0x10,0x20,0x30,0x40,0x50,0x53,0x00};


#ifdef LOWSN_COORDINATOR
typedef enum _ROUTER_STATE_ENUM {
	ROUTER_STATE_SLEEP,
	ROUTER_STATE_WAIT_FOR_RX,
	ROUTER_STATE_SEND,
	ROUTER_STATE_WAIT_FOR_TX
}ROUTER_STATE_ENUM;

ROUTER_STATE_ENUM RouterState;
BYTE rxFlag;
BYTE RouterAdvFrame[3] = {0x66, 0x77, 0xAA};
#endif

#ifdef LOWSN_RFD
typedef enum _NODE_STATE_ENUM {
	NODE_STATE_SLEEP,
	NODE_STATE_WAIT_SEND_FLAG,
	NODE_STATE_SEND,
	NODE_STATE_WAIT_FOR_TX
}NODE_STATE_ENUM;

NODE_STATE_ENUM NodeState;
BYTE sendFlag;
// �ٶ�������������������5���ֽڹ��ɣ���һ���ֽ�Ϊ��־λ����2,3���ֽ���
// �������ӵ���ֵ, ��������ֽڹ̶�.
BYTE SensorDataFrame[5] = {0x88, 0, 0, 0x11, 0x22};
UINT16 sensor_cnt;

#endif


#ifdef LOWSN_COORDINATOR
void RouterAction (void ) {

	apsFSM();

	switch (RouterState) {

			case ROUTER_STATE_SEND:
					conPrintROMString("Send Router Adv! \n");
					aplSendIPMSG(dstIPADDR,
						122,
						2, //dst EP
						0, //cluster is ignored for direct message
						122,
						1, //src EP
						&RouterAdvFrame[0],
						3,  //msg length
						apsGenTSN());  
					RouterState = ROUTER_STATE_WAIT_FOR_TX;
					
				break;

			case ROUTER_STATE_WAIT_FOR_TX:
				if (apsBusy()) break; //status not ready yet if busy.
				if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
					rxFlag = 0;
					my_timer= halGetMACTimer();
					conPrintROMString("Router Adv Sent Successfully! \n");
					RouterState =ROUTER_STATE_WAIT_FOR_RX;

				}else {
					conPrintROMString("Router Adv Send failed! Retry! \n");
					RouterState = ROUTER_STATE_SEND;
				}
				break;

			case ROUTER_STATE_WAIT_FOR_RX:
				//rxFlag is set from within usrRxPacketCallback
				if (rxFlag)  {
					// �յ�����֡���ȴ�ʱ�����¼���
					rxFlag = 0;
					my_timer= halGetMACTimer();
				}

				else if (halMACTimerNowDelta(my_timer) > MSECS_TO_MACTICKS( ROUTER_WAIT_TIME *1000))  {  
					//����ʱ�����޶�����ȥ����
					rxFlag = 0;
					RouterState = ROUTER_STATE_SLEEP;
					
				}
				else  {
					;

				}	
				break;	
				
			case  ROUTER_STATE_SLEEP:
			    conPrintROMString("Router begin to sleep! \n");
			    // �뱣��sleepʱ������, ����RAM���ݣ�����������������
			    halSleep(POWER_MODE_2, ROUTER_SLEEP_TIME *1000);
			    conPrintROMString("Router wake up!\n");
			    RouterState = ROUTER_STATE_SEND;
				break;		

			default:
				conPrintROMString("Never run to this state, something must be wrong! \n");
				break;

	}
}
#endif


#ifdef LOWSN_RFD
void NodeAction (void ) {

	apsFSM();

	switch (NodeState) {

			case NODE_STATE_SEND:
				
					conPrintROMString("Send sensor data ! \n");
					sensor_cnt++; 
					SensorDataFrame[1] = (BYTE) sensor_cnt;
					SensorDataFrame[2] =  (BYTE) (sensor_cnt>>8);
					aplSendIPMSG(dstIPADDR,
						122,
						2, //dst EP
						0, //cluster is ignored for direct message
						122,
						1, //src EP
						&SensorDataFrame[0],
						5,  //msg length
						apsGenTSN());  //No APS ack requested
					NodeState = NODE_STATE_WAIT_FOR_TX;
					
				break;

			case NODE_STATE_WAIT_FOR_TX:
				if (apsBusy()) break; //status not ready yet if busy.
				if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
					sendFlag = 0;
					conPrintROMString("Sensor Data Sent Successfully! \n");
					NodeState =NODE_STATE_SLEEP;

				}else {
					conPrintROMString("Sensor Data Send failed! Retry! \n");
					NodeState = NODE_STATE_SEND;
				}
				break;

			case NODE_STATE_WAIT_SEND_FLAG:
				//rxFlag is set from within usrRxPacketCallback
				if (sendFlag)  {
					//�յ���Router Adv, router �����ˣ����Է���������
					conPrintROMString("Get adv flag! \n");
					sendFlag = 0;
					NodeState = NODE_STATE_SEND;
				}
				
				break;	
				
			case  NODE_STATE_SLEEP:
			    conPrintROMString("Node begin to sleep! \n");
			    // �뱣��sleepʱ������, ����RAM���ݣ�����������������
			    halSleep(POWER_MODE_1, NODE_SLEEP_TIME *1000);
			    conPrintROMString("Node wake up!\n");
			    NodeState = NODE_STATE_WAIT_SEND_FLAG;
				break;		

			default:
				conPrintROMString("Never run to this state, something must be wrong! \n");
				break;

	}
}
#endif


void main (void){

	my_timer = 0;
	halInit();
	evbInit();

	aplInit();  //init the stack
	conPrintConfig();
	ENABLE_GLOBAL_INTERRUPT();  //enable interrupts


	EVB_LED1_OFF();
	EVB_LED2_OFF();

	//debug_level = 10;
	debug_level = 10;

	#ifdef LOWSN_COORDINATOR
	rxFlag = 0;
	#endif

	#ifdef LOWSN_RFD
	sensor_cnt = 0;
	sendFlag = 0;
	#endif	

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

	aplSetMacMaxFrameRetries(0);


#ifdef LOWSN_RFD
	//now send packets
	aplSetLinkLocalAddr16(&dstIPADDR, aplGetPanID(), 0);
	// �ڵ��ʼ̬Ϊ�ȴ�����·����������ADV֡
	NodeState = NODE_STATE_WAIT_SEND_FLAG;
	conPrintROMString("Node FSM Start!\n");
	while (1) {
		NodeAction();
	}
	
#endif

#ifdef LOWSN_COORDINATOR
	// ·����ʹ�ù㲥��ַ��ΪĿ���ַ�������κ�RFD�������յ�ADV֡
	aplSetLinkLocalAddr16(&dstIPADDR, aplGetPanID(), LOWSN_BCAST_SADDR);
	RouterState = ROUTER_STATE_SEND;
	conPrintROMString("Router FSM Start!\n");
      // ·������ʼ̬Ϊ����ADV֡
	while (1) {
		RouterAction();
	}
#endif


}

//########## Callbacks ##########

//callback from APS when packet is received
//user must do something with data as it is freed
//within the stack upon return.

LOWSN_STATUS_ENUM  usrRxPacketCallback(void) {

	BYTE len, *ptr;

	#ifdef LOWSN_COORDINATOR
	UINT16 rcv_sensor_count;
	#endif

	conPrintROMString("Sensor Data Received: \n");

	conPrintROMString("Source IP: \n");
	conPrintIP6ADDR(&(aplGetRxSrcIPAddr()), 1);
	conPrintROMString("\n");
	conPrintROMString("Source Port: ");
	conPrintUINT16(aplGetRxSrcPort());

	conPrintROMString(", Message Length: : ");
	len = aplGetRxMsgLen();
	conPrintUINT8(len);
	//conPrintROMString(",RSSI: ");
	//conPrintUINT8(aplGetRxRSSI());

	conPCRLF();

	ptr = aplGetRxMsgData();

	#ifdef LOWSN_COORDINATOR
	if (*ptr == 0x88)  {
		rxFlag = 1;
		conPrintROMString("##### Sensor Data:    ");	
		rcv_sensor_count = *(ptr+1);
		rcv_sensor_count += ((UINT16)*(ptr+2))<<8;
		conPrintUINT16(rcv_sensor_count);
		conPrintROMString(" #####");
	}
	else  {
		conPrintROMString("Received Unwanted Sensor Data \n ");
	}	
	#endif

	#ifdef LOWSN_RFD
	if ((*ptr == 0x66) && (*(ptr+1)==0x77) && (*(ptr+2)==0xAA))  {
		sendFlag = 1;
		conPrintROMString("Received Router Advertisement!\n");	
	}
	else  {
		conPrintROMString("Received is not a router adv! ");
	}	
	#endif

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
