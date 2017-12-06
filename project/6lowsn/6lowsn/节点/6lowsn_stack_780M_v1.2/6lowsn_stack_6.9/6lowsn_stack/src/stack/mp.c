
/******************************************************************************************************
*
* �� �� ����mp.c
*
* �ļ�����������������
*
* �� �� �ߣ�Wang Heng
*
* ��ǰ�汾��0.50
*
* �� �� �ߣ�
*
* �޸���ʷ��



********************************************************************************************************/

#include "compiler.h"               //compiler specific
#include "6lowsn_common_types.h"
#include "6lowsn_config.h"
#include "ieee_lrwpan_defs.h"
#include "hal.h"
#include "halStack.h"
#include "console.h"
#include "debug.h"
#include "phy.h"
#include "mac.h"
#include "adp.h"
#include "nwk.h"
#include "aps.h"
#include "mp.h"
#include "neighbor.h"
#include "evboard.h"
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
#include "slip.h"
#endif
#endif


MP_PIB mp_pib;
MP_RESPONSE_PARAM mp_rsp_param;

static void mpCommonFmt(IPADDR dst_saddr, BYTE command_id, BYTE aps_fcf, BYTE len);


void mpInit(void)
{
	mp_pib.flags.val = 0;
	mp_pib.eventType = MP_EVENT_NONE; 
	mp_pib.sensorType = LOWSN_SENSOR_TYPE;
	// ����Э����ΪĬ�ϵĹ�����Ϣ���͵�ַ, ��Ȼ����ݾ�������ݰ�ѡ��Ŀ���ַ
	aplSetLinkLocalAddr16(&mp_pib.dstAddr, aplGetPanID(), 0);
	
	return;
}


LOWSN_STATUS_ENUM mpHandleRxReport(void) 
{
	LOWSN_STATUS_ENUM rstatus;
        BYTE *ptr;

	rstatus = LOWSN_STATUS_SUCCESS;
	ptr = aplGetRxMsgData();

	if (a_aps_rx_data.dstObjID >= APS_SHORTCUT_BASE)  {

		// ������SHORTCUT ����
		DEBUG_STRING(DBG_INFO,"MP: Find Shortcut. \n");
		switch (a_aps_rx_data.srcObjID) {
			
			case MP_COMMAND_DEVICE_INFORMATION:
				   rstatus = mpHandleDeviceInfo();
				   break;

			case MP_COMMAND_TREE_NEIGHBOR_INFORMATION:
				   rstatus = mpHandleTreeNeighborInfo();
				   break;

		       default:
				   DEBUG_STRING(DBG_INFO,"MP: Unhandled MP Command, discarding.\n");
		}


	}
	else  {

		//��ͨ�����д

		DEBUG_STRING(DBG_INFO,"MP: Not support common object read/wirte now. \n");

	}
	 
	return(rstatus);
	
}


// ���Ǻ�̨��������ʱ��������
LOWSN_STATUS_ENUM mpHandleDeviceInfo(void)
{
	BYTE *ptr;
	SADDR saddr;

	ptr = aplGetRxMsgData();  

	return(LOWSN_STATUS_SUCCESS);
}


// ���Ǻ�̨��������ʱ��������
LOWSN_STATUS_ENUM mpHandleTreeNeighborInfo(void)
{
	BYTE *ptr;
	SADDR saddr;

	ptr = aplGetRxMsgData();  

	return(LOWSN_STATUS_SUCCESS);
}



/*---------------------------------------------------------------------*

��������������֡��Ϣ

ע��: FCF��Ҫ�������֡�Լ������ֱ�Ӵ��ݹ���

*---------------------------------------------------------------------*/
static void mpCommonFmt(IPADDR dst_saddr, BYTE command_id, BYTE aps_fcf, BYTE len)
{

	a_aps_tx_data.flags.val = 0;
	a_aps_tx_data.srcObjID = command_id;
	a_aps_tx_data.usrPlen = len;
	a_aps_tx_data.serviceID = apsGenServiceID();
	a_aps_tx_data.seqNum= aps_pib.apsSeqNum++;
	a_aps_tx_data.dstObjID = APS_SHORTCUT_BASE;
	
	a_aps_tx_data.dstPort = LOWSN_BASE_PORT;
	a_aps_tx_data.dstIPADDR = dst_saddr;
	a_aps_tx_data.srcPort = LOWSN_BASE_PORT;
	
	a_aps_tx_data.fcf = aps_fcf;
	a_aps_tx_data.flags.bits.loopback = 0; 

}

/*---------------------------------------------------------------------*

����C/S ��Ӧ֡��Ϣ

*---------------------------------------------------------------------*/
static void mpRspCommonFmt(BYTE len)
{

	a_aps_tx_data.flags.val = 0;
	a_aps_tx_data.srcObjID = aplGetRxDstObjectID();
	a_aps_tx_data.usrPlen = len;
	a_aps_tx_data.serviceID = aplGetRxServiceID();
	a_aps_tx_data.seqNum= aps_pib.apsSeqNum++;
	a_aps_tx_data.dstObjID = aplGetRxSrcObjectID();
	
	a_aps_tx_data.dstPort = LOWSN_BASE_PORT;
	a_aps_tx_data.dstIPADDR = aplGetRxSrcIPAddr();
	a_aps_tx_data.srcPort = LOWSN_BASE_PORT;

	a_aps_tx_data.fcf = 0;
	APS_SET_SERVICE_TYPE(a_aps_tx_data.fcf, APS_SERVICE_TYPE_CS);
	APS_SET_ACTION_TYPE(a_aps_tx_data.fcf, aplGetRxActType());
	APS_SET_ACTION_RESPONSE(a_aps_tx_data.fcf);

	//APS_SET_FCF_ACK_FLAG(a_aps_tx_data.fcf);  

	a_aps_tx_data.flags.bits.loopback = 0; 

}


/*---------------------------------------------------------------------*

�����豸��Ϣ

��ʽΪ:  Ӧ������д

�ڵ�̵�ַ��2 Bytes
�ڵ���ȣ�1 Byte
�ڵ㸸�ڵ�̵�ַ��2 Bytes
�ڵ�EUI64��ַ�� 8 Bytes
�ڵ�������1 Byte  ���ں��ڵ��ɫ��FFD/RFD���Ƿ�ȫ���Ƿ��ع���ȣ�
�ڵ�������ţ��û��Զ���  1 Byte


*---------------------------------------------------------------------*/
void mpFmtDeviceInfo(IPADDR dst_ipaddr)
{
	BYTE *ptr;
	BYTE len=0, fcf=0;
	UINT32 timer_temp;        
	timer_temp = halGetMACTimer();
	//wait for 2 seconds
	#ifdef RFD
		while ((halMACTimerNowDelta(timer_temp))< MSECS_TO_MACTICKS(50))
        	{}
	#endif
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];	

	// �ڵ�ǰ׺�б�
	// TO DO.

	// �ڵ�Ӧ�����к�

	// �ڵ��������
	ptr--;
	*ptr = LOWSN_SENSOR_TYPE;
	len++;

	//�ڵ�����
	ptr--;
	*ptr = mac_pib.macCapInfo;
	len++;

	// �ڵ㳤��ַ
	ptr = ptr - 8;
	halGetProcessorIEEEAddress(ptr);
	len = len + 8;

	// �ڵ㸸�ڵ�̵�ַ
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress());
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress()>>8);
	len = len + 2;

	//�ڵ����
	ptr--;
	*ptr = aplGetMyDepth();
	len++;
	

	// �ڵ�̵�ַ
	ptr--;
	*ptr = (BYTE) (macGetShortAddr());
	ptr--;
	*ptr = (BYTE) (macGetShortAddr()>>8);
	len = len + 2;


	// ���������˷������ͣ��Ժ��ǸĽ�
	APS_SET_SERVICE_TYPE(fcf, APS_SERVICE_TYPE_REPORT);
	//APS_SET_FCF_ACK_FLAG(fcf);  

	mpCommonFmt(dst_ipaddr, MP_COMMAND_DEVICE_INFORMATION, fcf, len);
		
}


/*---------------------------------------------------------------------*

����������Ϣ(�ھӽڵ���Ϣ)

��ʽΪ:  Ӧ������д

�ڵ�����̵�ַ��2 Bytes
�ڵ���ȣ�1 Bytes
�ڵ�EUI64��ַ�� 8 Bytes
�ڵ�������1 Bytes ����Ҫ�����жϽڵ���·���������ն��豸��
�ڵ㸸�ڵ�̵�ַ��2 Bytes
�ڵ㸸�ڵ�EUI64��ַ��8 Bytes
�ڵ��ӽڵ������2 Bytes
�ڵ��ӽڵ�̵�ַ�� 2 Bytes*����
�ڵ��ӽڵ㳤��ַ��8 Bytes*����





*---------------------------------------------------------------------*/
void mpFmtTreeNeighborInfo(IPADDR dst_ipaddr)
{
	BYTE *ptr;
	BYTE len=0, fcf=0;
	BYTE i;

	
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];	

	#if 0  //�ݲ��㱨�ӽڵ�
	#ifdef LOWSN_RFD
	ptr--;
	*ptr = 0;
	len++;

	#else



	#endif
	#endif

	// �ڵ㸸�ڵ㳤��ַ
	for (i=0; i<8; i++) {
		ptr--;
		*ptr = mac_pib.macCoordExtendedAddress.bytes[7-i];
	}	
	len = len + 8;

	// �ڵ㸸�ڵ�̵�ַ
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress());
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress()>>8);
	len = len + 2;

	//�ڵ�����
	ptr--;
	*ptr = mac_pib.macCapInfo;
	len++;

	// �ڵ㳤��ַ
	ptr = ptr - 8;
	halGetProcessorIEEEAddress(ptr);
	len = len + 8;
	
	//�ڵ����
	ptr--;
	*ptr = aplGetMyDepth();
	len++;
	
	// �ڵ�̵�ַ
	ptr--;
	*ptr = (BYTE) (macGetShortAddr());
	ptr--;
	*ptr = (BYTE) (macGetShortAddr()>>8);
	len = len + 2;

	
	// ���������˷������ͣ��Ժ��ǸĽ�
	APS_SET_SERVICE_TYPE(fcf, APS_SERVICE_TYPE_REPORT);
	//APS_SET_FCF_ACK_FLAG(fcf);  

	mpCommonFmt(dst_ipaddr, MP_COMMAND_TREE_NEIGHBOR_INFORMATION, fcf, len);
		
}



// ��鷢����ȡ������Ϣ��ָ���Ƿ���Ч
// TRUE: ��Ч;  FALSE: ��Ч
BOOL mpReadCheck(void)
{
	BYTE *ptr;
	ptr = aplGetRxMsgData();

	// ��ʱֻ֧�ֵ��������Ĳ���
	if (*ptr != 1)  {
		
		DEBUG_STRING(DBG_INFO,"MP: Only support single value opertion. \n");
		return FALSE;
	}

	// ���ڴ˶�֧�ֵĹ������ID���н�һ�������� 

	
	return TRUE;
}


// ��鷢��д������Ϣ��ָ���Ƿ���Ч
// TRUE: ��Ч;  FALSE: ��Ч
BOOL mpWriteCheck(void)
{
	BYTE *ptr;
	ptr = aplGetRxMsgData();

	// ��ʱֻ֧�ֵ��������Ĳ���
	if (*ptr != 1)  {
		
		DEBUG_STRING(DBG_INFO,"MP: Only support single value opertion. \n");
		return FALSE;
	}

	// ���ڴ˶�֧�ֵĹ������ID���н�һ�������� 

	
	return TRUE;
}


// ��鷢����ִ��ָ���Ƿ���Ч
// TRUE: ��Ч;  FALSE: ��Ч
BOOL mpExecuteCheck(void)
{
	BYTE *ptr;
	ptr = aplGetRxMsgData();

	// ��ʱֻ֧�ֵ�������
	if (*ptr != 1)  {
		
		DEBUG_STRING(DBG_INFO,"MP: Only support single value opertion. \n");
		return FALSE;
	}

	// ���ڴ˶�֧�ֵĹ������ID���н�һ�������� 

	
	return TRUE;
}



UINT8 mpFmtCurrentVoltageResponsePayload(void)
{

	BYTE *ptr;
	BYTE len=0;
	UINT16 voltage;

	voltage = getVoltageValue();

	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];	

	// ��д��ص�ѹֵ
	ptr--;
	*ptr = (BYTE) (voltage);
	ptr--;
	*ptr = (BYTE) (voltage >> 8);
	len = len + 2;

	//��д����ֵ����
	ptr--;
	*ptr = 2;
	len++;

	// ��д��������ID������ID
	ptr--;
	*ptr = 0;
	ptr--;
	*ptr = MP_ATTRIBUTE_ID_CURRENT_VOLTAGE;
	len = len + 2;

	// ��дִ�гɹ��Ľ����ִ�н������
	ptr--;
	*ptr = APS_RESULT_OK;
	ptr--;
	*ptr = 1;
	len = len + 2;

	return len;

}

void mpFmtReadResponse(void)
{
	BYTE *ptr;
	UINT8 plen;

	ptr = aplGetRxMsgData();
	
	if  (aplGetRxDstObjectID() ==  MP_OBJECT_ID_POWER)   {

		if (*(ptr+1) == MP_ATTRIBUTE_ID_CURRENT_VOLTAGE) {
					
			plen = mpFmtCurrentVoltageResponsePayload();
		}	

		else
		{
			// ��Ӧ��ִ�и���䣬��ϵͳ��ʱ��֧�ֵĶ�������ԣ�
			// mpReadCheck()����Ӧ���Ѿ���������
			// Ӧ֪ͨϵͳ������
			DEBUG_STRING(DBG_INFO,"MP: Sytem Error, Invalid attribute ID for  power management. \n");
			plen = 0;
			

		}
	}	

	else  {
		
			DEBUG_STRING(DBG_INFO,"MP: Sytem Error, Invalid object ID for READ. \n");
			plen = 0;
	}
					
	if (plen > 0)  {
		
		mpRspCommonFmt(plen);
		
	} 
	else  {

		// ��Ӧ�ó��ֵķ�֧���Ժ����ϵͳ������
	}	
		
	return;
	
}


// ��mpFmtReadResponse()д������
void mpFmtWriteResponse(void)
{
	BYTE *ptr;

	ptr = aplGetRxMsgData();
	
	return;

}

// ��mpFmtReadResponse()д������
void mpExcuteMethod(void)
{

	BYTE *ptr;

	ptr = aplGetRxMsgData();
		
	return;

}


#if 0

void zepFSM(void) {
	//does not do much right now

	if (zep_pib.flags.bits.Alarm) {
		//flash LED1 at 200ms rate
		if ((halMACTimerNowDelta(zep_pib.alarm_timer)) > MSECS_TO_MACTICKS(200)) {
			//toggle LED1
			if (EVB_LED1_STATE()) {
		     EVB_LED1_OFF();
			 } else {
                  EVB_LED1_ON();
				 }
            zep_pib.alarm_timer = halGetMACTimer();
		}


	}

}


//send a payload that has our short address, so that a ping
//can be used to send our short address to somebody
//The main goal is to the APS ACK back
//as verification that this was delivered
//this ping packet is also forwarded to the PC client
//if PC binding is being used.

void zepFmtPing(SADDR dst_saddr) {
	BYTE *ptr;
   	a_aps_tx_data.usrPlen = 3;
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
    *ptr = (BYTE) (macGetShortAddr()>>8);
    --ptr;
    *ptr = (BYTE) (macGetShortAddr());
    --ptr;
	*ptr = ZEP_EXT_PING;
	zepCommonFmt(dst_saddr,ZEP_EXTENDED_CMD);
}

//turns an alarm on at the node
//if mode is nonzero, turn on the alarm
//if mode is zero, turn off the alarm
void zepFmtAlarm(SADDR dst_saddr, BYTE mode) {
	BYTE *ptr;
   	a_aps_tx_data.usrPlen = 2;
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
    *ptr = mode;
    --ptr;
	*ptr = ZEP_EXT_SEND_ALARM;
	zepCommonFmt(dst_saddr,ZEP_EXTENDED_CMD);
}



//put our long address, short address into the tmpTxBuffer
//so that it can be sent to the coordinator.
void zepFmtEndDeviceAnnounce(SADDR dst_saddr){
	BYTE *ptr;

	//first, do the payload
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE] - 8;	
	//copy in the long address
	halGetProcessorIEEEAddress(ptr);
	//now put our short address
	--ptr;
    *ptr = (BYTE) (macGetShortAddr()>>8);

    --ptr;
    *ptr = (BYTE) (macGetShortAddr());
	
    a_aps_tx_data.usrPlen = ZEP_PLEN_END_DEVICE_ANNOUNCE;

	//now, the rest
	zepCommonFmt(dst_saddr,ZEP_END_DEVICE_ANNOUNCE);
		
}

/* this is a custom Zero End point command that sends data that is spread
across several different Zigbee response commands into one. This info
is needed by the PC client in the binding demo
*/
void zepFmtNodeInfoRsp(SADDR dst_saddr){

	BYTE *ptr;
	BYTE i;

	//endpoints first
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];
	//do the endpoints first.
	for (i=0;i<aps_pib.activeEPs;i++){
		--ptr;
		*ptr = apsEndPoints[i].epNum;
	}
    --ptr;
	*ptr = aps_pib.activeEPs;
	
	//MAC capability code
    --ptr;
	*ptr = mac_pib.macCapInfo;

	//Node type
    --ptr;
#if defined(LOWSN_COORDINATOR)
	*ptr = NODE_TYPE_COORD;
#elif defined(LOWSN_FFD)
	*ptr = NODE_TYPE_ROUTER;
#else
    *ptr = NODE_TYPE_ENDDEVICE;
#endif
    //my parent
	  --ptr;
	*ptr = (mac_pib.macCoordShortAddress >> 8) & 0xFF;
	

	--ptr;	
    *ptr = mac_pib.macCoordShortAddress & 0xFF;


	//my short address
    --ptr;
	*ptr = (macGetShortAddr() >> 8) & 0xFF;
	

	--ptr;	
    *ptr = macGetShortAddr() & 0xFF;

	//copy in the long address
	ptr = ptr - 8;
	halGetProcessorIEEEAddress(ptr);

	//indentify this extended command
	--ptr;
	*ptr = ZEP_EXT_NODE_INFO_RSP;

	a_aps_tx_data.usrPlen = aps_pib.activeEPs + ZEP_PLEN_NODE_INFO_RSP ;

	//now, the rest
	zepCommonFmt(dst_saddr,ZEP_EXTENDED_CMD);

}

#endif



