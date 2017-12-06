
/******************************************************************************************************
*
* �� �� ����test_ping.c
*
* �ļ��������������Բ�������1
*
* �� �� �ߣ�
*
* ��ǰ�汾:  

* �� �� �ߣ�
*
* �޸���ʷ��


********************************************************************************************************/

#include "6lowsn.h"



// ������LOWSN_SIMULATED_SENSOR_DATA������ô�0-99ѭ���������¶�ģ�����ݣ��������
// ��ȡ���ú�Ķ��壬������CC2530��ʵ���¶ȴ��������ܶ�ȡ�����¶�ֵ��Ϊ����.
#define LOWSN_SIMULATED_SENSOR_DATA


#ifdef LOWSN_RFD
#define NODE_SLEEP_TIME  3   //�ڵ�˯��ʱ��
#endif

UINT32 my_timer;
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

#ifdef LOWSN_RFD

#define REMOTE_PORT    0xF0B2
#define LOCAL_PORT      0xF0B3
#define DEFAULT_SLEEP_PERIOD  100   //��msΪ��λ	
#define DEFAULT_RX_PERIOD  2000   //��msΪ��λ
#define REMOTE_OBJECT_ID    1
#define OBJECT_ID_TEMP    1
#define ATTRIBUTE_ID_CURRENT_TEMP    1
#define ATTRIBUTE_ID_TEMP_UPPER    2
#define ATTRIBUTE_ID_TEMP_LOWER    3						
#define OBJECT_ID_LIGHT   2

typedef enum _NODE_STATE_ENUM {
	NODE_STATE_SENSE_DATA, 
	NODE_STATE_SEND_DATA, 
	NODE_STATE_SLEEP,
	NODE_STATE_SEND,
	NODE_STATE_WAIT_FOR_TX, 
	NODE_STATE_WAIT_FOR_RX, 
	NODE_STATE_WAIT_FOR_RESPONSE_TX
}NODE_STATE_ENUM;

typedef struct _RESPONSE_PARAM {
	IPADDR dstIPADDR;
	UINT16 dstPort;
	BYTE dstObjID;
	UINT16 srcPort;
	BYTE srcObjID; 
	BYTE serviceID; 
	BYTE actDirection; 
	BYTE actType;
	BYTE *pload;
	BYTE plen;
}RESPONSE_PARAM;

NODE_STATE_ENUM NodeState;
RESPONSE_PARAM ResponseParam;
INT8 temp_upper_bound, temp_lower_bound;
INT8 current_temp;
INT8 SensorPayload[20];
INT8 ResponsePayload[20];
UINT32 SleepPeriod, RxPeriod;
BOOL sendResponseFlag;
INT8 temp_value=0;

#ifdef  LOWSN_SIMULATED_SENSOR_DATA

INT8 getTempValue()
{
	temp_value++;
	
	if (temp_value > 100)  {
		temp_value=0;
	}

	return (temp_value);

}

#else

#define HAL_ADC_REF_125V    0x00    /* Internal 1.25V Reference */
#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_CHN_VDD3    0x0f    /* Input channel: VDD/3 */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */


INT8 getTempValue()
{
  static UINT16 voltageAtTemp22;
  static UINT8 bCalibrate=TRUE;
  UINT16 value;
  INT8 temp;

  ATEST = 0x01;
  TR0  |= 0x01; 
  
  ADCIF = 0;

  ADCCON3 = (HAL_ADC_REF_125V | HAL_ADC_DEC_512 | HAL_ADC_CHN_TEMP);

  while ( !ADCIF );

  value = ADCL;
  value |= ((UINT16) ADCH) << 8;

  value >>= 4;
  
    /* Assume ADC = 1480 at 25C and ADC = 4/C */
  #define VOLTAGE_AT_TEMP_25        1480
  #define TEMP_COEFFICIENT          4

  if(bCalibrate) {
    voltageAtTemp22=value;
    bCalibrate=FALSE;
  }
  
  temp = 22 + ( (value - voltageAtTemp22) / TEMP_COEFFICIENT );
  
  // �¶�ֻ֧��0-100��֮�䣬���¶Ȳ�֧��
  if( temp >= 100) 
  {
    return 100;
  }
  else if (temp <= 0) {
    return 0;
  }
  else { 
    return temp;
  }

}

#endif


void NodeAction (void ) {

	apsFSM();

	switch (NodeState) {

			case NODE_STATE_SENSE_DATA:
				

					current_temp = getTempValue();

					//conPrintROMString("Current temperature is : ");
					//conPrintUINT8((UINT8)current_temp);
                                   //conPrintROMString("\n");
				
					NodeState = NODE_STATE_SEND_DATA;

					// ���¶�ֵ�����˲��ȴ���

				break;
				
			case NODE_STATE_SEND_DATA:
				

					// ׼���¶ȷ��͵ĸ���
					SensorPayload[0] = 1;  // Attribute Number
					SensorPayload[1] = ATTRIBUTE_ID_CURRENT_TEMP; //  Attribute identifier
					SensorPayload[2] = 0; //  Attribute secondary identifier
					SensorPayload[3] = 1; // Length 
					SensorPayload[4] = current_temp; // Value 
					

					#if 1
					// �ж��¶�ֵ�Ƿ񳬱�
					if ((current_temp >= temp_upper_bound) || (current_temp <= temp_lower_bound))  {

						// �����صı�����������˸LED


						// ��report/alarm��ʽ��������
  						aplSendReportData(dstIPADDR, REMOTE_PORT, REMOTE_OBJECT_ID, LOCAL_PORT, OBJECT_ID_TEMP, 
  						                                  apsGenServiceID(), 0, (BYTE *)(&SensorPayload[0]), 5); 						
						
					}	
					else  {
						
						// ��publish��ʽ��������
  						aplSendPublishData(dstIPADDR, REMOTE_PORT, REMOTE_OBJECT_ID, LOCAL_PORT, OBJECT_ID_TEMP, 
  						                                  apsGenServiceID(), 0, (BYTE *)(&SensorPayload[0]), 5); 
									
					}	

					#endif

					conPrintROMString("#");

					NodeState = NODE_STATE_WAIT_FOR_TX;

				break;

			case NODE_STATE_WAIT_FOR_TX:
				if (apsBusy()) break; 
				if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
					conPrintROMString("Sensor Data Sent Successfully! \n");
					my_timer= halGetMACTimer();
					NodeState =NODE_STATE_WAIT_FOR_RX;

				}else {
					conPrintROMString("Sensor Data Send failed! Retry! \n");
					my_timer= halGetMACTimer();
					NodeState =NODE_STATE_WAIT_FOR_RX;  //�����ش�, Э��ջ�ڲ����ش�
					//NodeState = NODE_STATE_SEND_DATA;
				}
				break;

			case  NODE_STATE_WAIT_FOR_RX:
				
				if (halMACTimerNowDelta(my_timer) > MSECS_TO_MACTICKS(RxPeriod)) {
					NodeState =NODE_STATE_SLEEP;
					
					break;
					
				}

				if (sendResponseFlag)  {

					sendResponseFlag = 1;
				
					
					aplSendCSData(ResponseParam.dstIPADDR, ResponseParam.dstPort, ResponseParam.dstObjID, ResponseParam.srcPort, ResponseParam.srcObjID, 
					                          ResponseParam.serviceID, 0, ResponseParam.pload, ResponseParam.plen, ResponseParam.actDirection, ResponseParam.actType); 		

					NodeState = NODE_STATE_WAIT_FOR_RESPONSE_TX;
	
				}	
				
				break;	

			case NODE_STATE_WAIT_FOR_RESPONSE_TX:
				if (apsBusy()) break; 
				if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
					
					conPrintROMString("Response Sent Successfully! \n");
					NodeState =NODE_STATE_WAIT_FOR_RX;

				}else {
				
					conPrintROMString("Response Send failed! Retry! \n");
					NodeState =NODE_STATE_WAIT_FOR_RX;

				}
				break;
				
			case  NODE_STATE_SLEEP:
			    conPrintROMString("Node begin to sleep! \n");
			    // �뱣��sleepʱ������, ����RAM���ݣ�����������������
			    halSleep(POWER_MODE_2, SleepPeriod);
			    conPrintROMString("Node wake up!\n");
			    NodeState = NODE_STATE_SENSE_DATA;
				break;		

			default:
				conPrintROMString("Never run to this state, something must be wrong! \n");
				break;

	}
}

// ���ظ��ص�ʵ�ʳ���
UINT8 FmtCurrentTempResponsePayload(void)
{
	INT8 temp;
	
	temp = getTempValue();

	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_CURRENT_TEMP;
	ResponsePayload[3] = 0;
	ResponsePayload[4] = 1;
	ResponsePayload[5] = temp;

	return 6;
}

UINT8 FmtTempUpperResponsePayload(void)
{

	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_TEMP_UPPER;
	ResponsePayload[3] = 0;
	ResponsePayload[4] = 1;
	ResponsePayload[5] = temp_upper_bound;

	return 6;
}

UINT8 FmtTempLowerResponsePayload(void)
{
	ResponsePayload[0] = 1;
	ResponsePayload[1] = APS_RESULT_OK;
	ResponsePayload[2] = ATTRIBUTE_ID_TEMP_LOWER;
	ResponsePayload[3] = 0;
	ResponsePayload[4] = 1;
	ResponsePayload[5] = temp_lower_bound;

	return 6;
}

#endif

void main (void){

	my_timer = 0;
	halInit();
	evbInit();

	aplInit(); 

	#ifdef LOWSN_COORDINATOR
	#ifdef LOWSN_SLIP_TO_HOST
	slipInit();
       #endif
	#endif
		
	conPrintConfig();
	ENABLE_GLOBAL_INTERRUPT();  //enable interrupts


	EVB_LED1_OFF();
	EVB_LED2_OFF();


	debug_level = DBG_MAX_LEVEL;

	#ifdef LOWSN_COORDINATOR

	#ifdef LOWSN_SLIP_TO_HOST

	// First, get a prefix from host.
	do {
			my_timer= halGetMACTimer();
			while(slipTxLocked());
			conPrintROMString("Request a golbal prefix ...  \n");
			slipGrabTxLock(); 
			slipRequestPrefix();
			slipReleaseTxLock(); 

			while ((!slipReady()) && (halMACTimerNowDelta(my_timer) < MSECS_TO_MACTICKS(2*1000)));

			if (slipReady())  {
		
				slipRcv();

				if (slipParsePrefix(&network_prefix, &prefix_len)  == 0)  {
					conPrintROMString("Get global prefix successfully! \n");
					aplSetPrefix(&network_prefix, prefix_len);
					conPrintROMString("Set global prefix:  ");
					conPrintIP6ADDR(&network_prefix, 1);
					conPrintROMString("\n Prefix Len: ");
					conPrintUINT8(prefix_len);
					conPrintROMString("\n");
					break;

				}	

				else  {
					conPrintROMString("Not a prefix slip command. Try again! \n");
				}
			}	
		
	 } while (1);

	#else

	lowsn_ip6addr(&network_prefix,0x0066,0x0077,0x1234,0x5678,0,0,0,0);
	prefix_len = LOWSN_DEFAULT_PREFIX_LEN;
	aplSetPrefix(&network_prefix, prefix_len);
	conPrintROMString("Set global prefix:  ");
	conPrintIP6ADDR(&network_prefix, 1);
	conPrintROMString("  Prefix Len: ");
	conPrintUINT8(prefix_len);
	conPrintROMString("\n");

	#endif
	
	// Second, form a subnet.

	#ifdef  LOWSN_NO_JOIN_PROCESS
	aplFormNetworkDirectly(); 
	#else
	aplFormNetwork();
	#endif
	while(apsBusy()) {apsFSM();} //wait for finish
	conPrintROMString("Network is formed. \n");
	EVB_LED1_ON();
	

	#else

	#ifdef LOWSN_NO_JOIN_PROCESS
	aplJoinNetworkDirectly(0x1699, 0, &coordEUI[0], 1);
	while(apsBusy()) {apsFSM();} 
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
	#endif

	#ifdef LOWSN_NO_PREFIX_PROCESS
	// �ֶ����ýڵ��ȫ��ǰ׺
	lowsn_ip6addr(&network_prefix,0x0066,0x0077,0x1234,0x5678,0,0,0,0);
	prefix_len = LOWSN_DEFAULT_PREFIX_LEN;
	aplSetPrefix(&network_prefix, prefix_len);
	conPrintROMString("Manually Set global prefix:  ");
	conPrintIP6ADDR(&network_prefix, 1);
	conPrintROMString("  Prefix Len: ");
	conPrintUINT8(prefix_len);
	conPrintROMString("\n");
	#else
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


	#endif

	aplSetMacMaxFrameRetries(0);

	#ifdef LOWSN_COORDINATOR

	while (1) {
		
		apsFSM();

		#ifdef LOWSN_SLIP_TO_HOST
		slipFSM();
		#endif
	}	
	#endif

	#ifdef LOWSN_ROUTER

	while (1) {
		
		apsFSM();

	}	
	#endif

	#ifdef LOWSN_RFD
	aplSetLinkLocalAddr16(&dstIPADDR, aplGetPanID(), 0);

	conPrintROMString("RFD Object IP is: \n");
	conPrintIP6ADDR(&dstIPADDR, 1);
	conPrintROMString("\n");

	SleepPeriod = DEFAULT_SLEEP_PERIOD;
	RxPeriod = DEFAULT_RX_PERIOD;

	NodeState = NODE_STATE_SENSE_DATA;
	
	conPrintROMString("Node FSM Start!\n");
	
	while (1) {
		NodeAction();
	}

	#endif
	
}


LOWSN_STATUS_ENUM  usrRxPacketCallback(void) 
{

	BYTE *ptr;
	UINT8 plen;

	#if 1  //��ӡһЩ���հ�����Ϣ����������
	conPrintROMString("Data Received. \n");
	conPrintROMString("Source IP: \n");
	conPrintIP6ADDR(&(aplGetRxSrcIPAddr()), 1);
	conPrintROMString("\n");
	conPrintROMString("Source Port: ");
	conPrintUINT16(aplGetRxSrcPort());
	conPrintROMString(", Service Type: ");
	conPrintUINT8(aplGetRxServiceType());
	conPCRLF();


	ptr = aplGetRxMsgData();

	#ifdef LOWSN_COORDINATOR
	#ifndef  LOWSN_SLIP_TO_HOST
	conPrintROMString(", Msg Length: ");
	plen = aplGetRxMsgLen();
	conPrintUINT8(plen);
	conPCRLF();
	conPrintROMString("Msg:  ");
	while(plen){
		conPrintUINT8(*ptr);
		conPrintROMString("  ");
		ptr++; plen--;
	}
	conPCRLF();
	#endif
	#endif

	#endif
	
	
	#ifdef LOWSN_RFD

	conPrintROMString("111. \n");

	// ���˿ں�
	if ((aplGetRxSrcPort() != REMOTE_PORT) || (aplGetRxDstPort() != LOCAL_PORT)) {
		conPrintROMString("invalid port, discard it. ");
		return LOWSN_STATUS_SUCCESS;
	}	

	conPrintROMString("222. \n");

	// ��ڵ�ֻ֧��C/S��д , ��һ��ֻ��/дһ������
	if (aplGetRxServiceType() != APS_SERVICE_TYPE_CS) {
		conPrintROMString("Not a client/server type, discard it. ");
		return LOWSN_STATUS_SUCCESS;
	}	

	//  ׼��������Ӧ֡�Ĳ��ֲ���
	ResponseParam.dstIPADDR = aplGetRxSrcIPAddr();
	ResponseParam.dstPort = aplGetRxSrcPort();
	ResponseParam.dstObjID = aplGetRxSrcObjectID();
	ResponseParam.srcPort = LOCAL_PORT;
	ResponseParam.serviceID = aplGetRxServiceType();
	ResponseParam.actDirection = APS_ACTION_RESPONSE;
	ResponseParam.actType = aplGetRxActType();

	//  ����C/S����
	

	conPrintROMString("333. \n");
	
	switch (ResponseParam.actType)  {
		
		case APS_ACTION_TYPE_READ:

			if ((*ptr == 1)  && (aplGetRxDstObjectID() ==  OBJECT_ID_TEMP))  {
				if (*(ptr+1) == ATTRIBUTE_ID_CURRENT_TEMP) {
					
					plen = FmtCurrentTempResponsePayload();
					conPrintROMString("444. \n");
				}	

				else if (*(ptr+1) == ATTRIBUTE_ID_TEMP_UPPER) {
					plen = FmtTempUpperResponsePayload();
				}	

				else if (*(ptr+1) == ATTRIBUTE_ID_TEMP_LOWER) {
					plen = FmtTempLowerResponsePayload();
				}	

				else
				{
					conPrintROMString("invalid attribute ID for temperature, discard it. ");
					return LOWSN_STATUS_SUCCESS;

				}

						
				ResponseParam.srcObjID = OBJECT_ID_TEMP;
				ResponseParam.pload = (BYTE *)(&ResponsePayload[0]);
				ResponseParam.plen = plen;
				sendResponseFlag = 1;
					
			}	


		break;
		
		case APS_ACTION_TYPE_WRITE:

			if (*ptr == 1)  {

				if (aplGetRxDstObjectID() ==  OBJECT_ID_TEMP)  {

					
					if (*(ptr+1) == ATTRIBUTE_ID_TEMP_UPPER) {
					
						temp_upper_bound = *(ptr+4);
					}	

					else if (*(ptr+1) == ATTRIBUTE_ID_TEMP_LOWER) {
					
						temp_lower_bound = *(ptr+4);
					}	

					else
					{
						conPrintROMString("invalid attribute ID for temperature bound write, discard it. \n");
						return LOWSN_STATUS_SUCCESS;
					}

				}	

				else if  (aplGetRxDstObjectID() ==  OBJECT_ID_LIGHT)  {
					

						 conPrintROMString("Not support light control now, discard it. \n ");
						return LOWSN_STATUS_SUCCESS;

				}	
				
				ResponsePayload[0] = 1;
				ResponsePayload[1] = APS_RESULT_OK;
				
				ResponseParam.srcObjID = aplGetRxDstObjectID();
				ResponseParam.pload = (BYTE *)(&ResponsePayload[0]);
				ResponseParam.plen = 2;
				sendResponseFlag = 1;
					
			}	

		break;		

		default: 

			conPrintROMString("invalid action type, discard it. ");
			break;
		
	}

	#endif 
	
	return LOWSN_STATUS_SUCCESS;
	
}

#ifdef LOWSN_FFD
//Callback to user level to see if OK for this node
//to join - implement Access Control Lists here based
//upon IEEE address if desired
BOOL usrJoinVerifyCallback(LADDR *ptr, BYTE capinfo)
{

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
	#ifndef LOWSN_SLIP_TO_HOST
	conPrintROMString("Node joined: ");
	conPrintLADDR(ptr);
	conPCRLF();
	DEBUG_PRINTNEIGHBORS(DBG_INFO);
	#endif
	
	return TRUE;
}
#endif


#ifndef LOWSN_COORDINATOR
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
#endif

//called when the slow timer interrupt occurs
#ifdef LOWSN_ENABLE_SLOW_TIMER
void usrSlowTimerInt(void ) {}
#endif


//general interrupt callback , when this is called depends on the HAL layer.
void usrIntCallback(void){}
