

#ifndef APS_H
#define APS_H

#include "ds.h"
#include "adp.h"

#define APS_RXBUFF_SIZE LOWSN_MAX_INDIRECT_RX_PKTS


// 6LoWSN Ӧ�ò�
#define APS_SERVICE_TYPE_CS			0
#define APS_SERVICE_TYPE_PUBLISH        	1
#define APS_SERVICE_TYPE_REPORT        	2
#define APS_SERVICE_TYPE_ACKFRAME       3

#define APS_ACTION_REQUEST                     0
#define APS_ACTION_RESPONSE                   1

#define APS_ACTION_TYPE_NONE                 0
#define APS_ACTION_TYPE_READ                 1
#define APS_ACTION_TYPE_WRITE               2
#define APS_ACTION_TYPE_EXECUTE           3

#define APS_SHORTCUT_BASE                 0xE0

#define APS_SET_SERVICE_TYPE(x,f)         (x=x |(f<<5))
#define APS_SET_FCF_ACK_FLAG(x)          BITSET(x,4) 
#define APS_SET_ACTION_RESPONSE(x)   BITSET(x,2) 
#define APS_SET_ACTION_TYPE(x, f)         (x=x |f)

#define APS_GET_FCF_ACK_FLAG(x)          ((x>>4)&0x1)
#define APS_GET_SERVICE_TYPE(x)           ((x>>5)&0x7)
#define APS_GET_ACTION_DIRECTION(x)   ((x>>2)&0x1)
#define APS_GET_ACTION_TYPE(x)             (x&0x3)

#define APS_DSTMODE_NONE           0
#define APS_DSTMODE_SHORT         1
#define APS_DSTMODE_LONG           2

#define APS_RESULT_OK                                       0x20
#define APS_RESULT_REQUEST_FORMAT_ERROR  0x30
#define APS_RESULT_OBJECT_NOT_EXIST            0x31
#define APS_RESULT_NO_AUTH                             0x32
#define APS_RESULT_SERVICE_INVALID               0x33
#define APS_RESULT_EXE_TIMEOUT                      0x34
#define APS_RESULT_EXE_FAILED                         0x35


#define LOWSN_IPMODE_INDEPENDENT  0
#define LOWSN_IPMODE_FROM_SADDR   1
#define LOWSN_IPMODE_FROM_LADDR   2


#define APS_CS_RESPONSE_WAIT_TIME      MSECS_TO_MACTICKS(3000)

#define LOWSN_SLIP_APS_HEADER_LEN  33

//right now, all this structure has in it is the endpoint number.
typedef struct _APS_EP_ELEM {
	BYTE epNum;
}APS_EP_ELEM;

//only use for this right now is to allow user to
//regiser endpoints with the APS level so that they
//can be reported to the coordinator.
extern APS_EP_ELEM apsEndPoints[LOWSN_MAX_ENDPOINTS];


typedef struct _APS_RX_DATA {
	MACPKT orgpkt;
	BYTE apsOffset;
	BYTE afOffset;
	BYTE attribOffset;   // δ������attribute��ʼλ��, �����UDP��ʼλ��
	IPADDR srcAddress;
	UINT16 dstPort;
	UINT16 srcPort;
	UINT16 udp_plen;
	
	union _APS_RX_DATA_FLAGS{
		BYTE val;
		struct {
			unsigned srcEP:1;
			unsigned dstEP:1;
		}bits;
	}flags;
	//parse these out of the packet for reference
	BYTE dstEP;
	BYTE dstObjID;
	BYTE srcEP;
	BYTE srcObjID;
	BYTE serviceID; 
	BYTE seqNum; 
	BYTE fcf;
	BYTE cluster;
	UINT16 profile;
	SADDR dstSADDR;   //used for indirect msgs
	SADDR srcSADDR;
	IPADDR dstIPADDR;
	IPADDR srcIPADDR;
	BYTE af_fcf;
	BYTE aps_fcf;
//these fields are for AF MSG data
	BYTE   tsn;
	BYTE   *usrPload;
	BYTE   usrPlen;
}APS_RX_DATA;



typedef struct _APS_TX_DATA {
	union _APS_TX_DATA_FLAGS{
		BYTE val;
		struct {
			unsigned loopback:1;
			unsigned isackframe:1;
			unsigned needresponse:1; 
		}bits;
	}flags;
	BYTE aps_fcf;
	BYTE af_fcf;	
	BYTE fcf; 
	BYTE dstEP;
	BYTE dstObjID;
	BYTE dstMode;
	SADDR dstSADDR;
	BYTE *dstLADDR;
	IPADDR dstIPADDR;
	IPADDR srcIPADDR;
	UINT16 dstPort;
	UINT16 srcPort;
	SADDR srcSADDR;
	BYTE cluster;
	BYTE srcEP;
	BYTE srcObjID; 
	BYTE  tsn;
	BYTE serviceID; 
	BYTE seqNum; 
	BYTE aps_flen;  //used for retries.
	BYTE *aps_ptr;  //used for retries
	BYTE *usrPload;
	BYTE usrPlen;
}APS_TX_DATA;

//arguments for extended ZEP commands
typedef union _ZEP_EXT_ARGS {
	struct {
		BYTE   mode;
	}alarm;
}ZEP_EXT_ARGS;

typedef union _APS_ARGS {
	struct {
		BYTE   commandID; 
		IPADDR dstIPAddr; 
		ZEP_EXT_ARGS ext;		
	}mp_tx;
}APS_ARGS;

typedef enum _APS_STATE_ENUM {
  APS_STATE_IDLE,
  APS_STATE_COMMAND_START,
  APS_STATE_GENERIC_TX_WAIT,
  APS_STATE_CS_REQ_TX_WAIT1, 
  APS_STATE_CS_REQ_TX_WAIT2, 
  APS_STATE_NWK_PASSTHRU_WAIT,
  APS_STATE_INDIRECT_GETDST,
  APS_STATE_INDIRECT_TX,
#ifdef LOWSN_COORDINATOR
  APS_STATE_INJECT_INDIRECT,
#endif
  APS_STATE_ACK_SEND_START,
  APS_STATE_MP_READ_RESPONSE, 
  APS_STATE_MP_WRITE_RESPONSE, 
  APS_STATE_MP_EXECUTE_RESPONSE, 
  APS_STATE_INDIRECT_TX_WAIT,
  APS_STATE_INJECT_LOOPBACK,
  APS_STATE_INDIRECT_LOOPBACK
 } APS_STATE_ENUM;


typedef enum _APS_RXSTATE_ENUM {
	APS_RXSTATE_IDLE,
	APS_RXSTATE_START,
	APS_RXSTATE_RESOLVE_INDIRECT,
	APS_RXSTATE_MP_REQUEST_PENDING, 
	#ifdef LOWSN_COORDINATOR
  	#ifdef LOWSN_SLIP_TO_HOST
  	APS_RXSTATE_FORWARD_HOST, 
  	#endif
  	#endif
	APS_RXSTATE_ACK_SEND_WAIT
} APS_RXSTATE_ENUM;

typedef struct _APS_SERVICE {
  LOWSN_SVC_ENUM cmd;
  APS_ARGS args;
  LOWSN_STATUS_ENUM status;
}APS_SERVICE;


typedef struct _APS_PIB{
	union _APS_PIB_FLAGS{
		UINT16 val;
		struct {
			unsigned indirectPending:1;
			unsigned ackPending:1;         //expecting an ACK
			unsigned ackSendPending:1;     //we need to send an APS ack!
			unsigned TxInProgress:1;       //TX in progress
			unsigned IsUsrBufferFree:1;
			unsigned IsGotCsResponse:1;
			unsigned WaitCsResponse:1;
		}bits;
	}flags;
	BYTE activeEPs;          // num of active EPs
	UINT32 tx_start_time;
	BYTE apsTSN; //transaction sequence number
	BYTE apsServiceID; 
	BYTE apsSeqNum; 
	UINT32 apscAckWaitDuration; // in mac TICs
	UINT16 apsAckWaitMultiplier;
	UINT16 apsAckWaitMultiplierCntr;
	BYTE apscMaxFrameRetries;
	BYTE currentAckRetries;
#ifdef LOWSN_COORDINATOR
	BYTE rxCnt;          //number of packets currently in buffer
    BYTE rxTail;        //tail pointer for buffer
	BYTE rxHead;        //head pointer for buffer
	//fifo for RX pkts, holds LOWSN_MAX_NWK_RX_PKTS
	APS_RX_DATA  rxBuff[APS_RXBUFF_SIZE];  //buffer for APS packets to be redirected
#endif

}APS_PIB;

extern APS_SERVICE a_aps_service;
extern APS_TX_DATA a_aps_tx_data;
void apsFSM(void);
void apsInit(void);
extern APS_STATE_ENUM apsState;
extern APS_PIB aps_pib;
extern APS_RX_DATA a_aps_rx_data;

extern BOOL apsRxBusy(void);
extern void apsRxHandoff(void);

extern LOWSN_STATUS_ENUM usrRxPacketCallback(void);


void aplShutdown(void);
void aplWarmstart(void);

#define apsIdle() (apsState == APS_STATE_IDLE)
#define apsBusy() (apsState != APS_STATE_IDLE)

#define apsDoService() \
   a_aps_service.status = LOWSN_STATUS_APS_INPROGRESS;\
   apsState = APS_STATE_COMMAND_START;\
   apsFSM();


#if 0
void aplFmtSendMSG (BYTE dstMode,
				 LADDR_UNION *dstADDR,
				 BYTE dstEP,
				 BYTE cluster,
				 BYTE srcEP,
				 BYTE* pload,
				 BYTE  plen,
				 BYTE  tsn,
				 BYTE  reqack);

#define aplSendMSG(dstMode, dstADDR, dstEP, cluster,srcEP,pload, plen, tsn, reqack)\
	aplFmtSendMSG(dstMode, dstADDR, dstEP, cluster,srcEP,pload, plen, tsn, reqack);\
	apsDoService();

void aplFmtSendIPMSG (IPADDR dstIPADDR,
				 UINT16 dstPort,
				 BYTE dstEP,
				 BYTE cluster,
				 UINT16 srcPort,
				 BYTE srcEP,
				 BYTE* pload,
				 BYTE  plen,
				 BYTE  tsn);

#define aplSendIPMSG(dstIPADDR, dstPort, dstEP, cluster,srcPort, srcEP, pload, plen, tsn)\
	aplFmtSendIPMSG(dstIPADDR, dstPort, dstEP, cluster,srcPort, srcEP, pload, plen, tsn);\
	apsDoService();

#endif

// Publish����: ���������Ե����ݵķ���

void aplFmtSendPublishData (IPADDR dstIPADDR,
				 UINT16 dstPort,
				 BYTE dstObjID,
				 UINT16 srcPort,
				 BYTE srcObjID,
				 BYTE  serviceID, 
				 BYTE  reqACK, 
				 BYTE* pload,
				 BYTE  plen
				 );

#define aplSendPublishData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendPublishData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen);\
	apsDoService();

// Report����: ���ڷ����������ݻ򱨾����ݵķ���
// ˵��:  Ŀǰ��δ��ȷ����publish��report�������г���ܣ��Ժ��ٸ��Ը��Ի�

void aplFmtSendReportData (IPADDR dstIPADDR,
				 UINT16 dstPort,
				 BYTE dstObjID,
				 UINT16 srcPort,
				 BYTE srcObjID,
				 BYTE  serviceID, 
				 BYTE  reqACK, 
				 BYTE* pload,
				 BYTE  plen
				 );

#define aplSendReportData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendReportData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen);\
	apsDoService();



// Client/Server����: ���ڵ㵽��֮���˫�򽻻�

void aplFmtSendCSData (IPADDR dstIPADDR,
				 UINT16 dstPort,
				 BYTE dstObjID,
				 UINT16 srcPort,
				 BYTE srcObjID,
				 BYTE  serviceID, 
				 BYTE  reqACK, 
				 BYTE* pload,
				 BYTE  plen, 
				 BYTE actDirection, 
				 BYTE actType
				 );

// C/Sģʽͳһָ��
#define aplSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, actDirection, actType)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, actDirection, actType);\
	apsDoService();

// C/Sģʽ������
#define aplReadData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, APS_ACTION_REQUEST, APS_ACTION_TYPE_READ);\
	apsDoService();

// C/Sģʽ д����
#define aplWriteData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, APS_ACTION_REQUEST, APS_ACTION_TYPE_WRITE);\
	apsDoService();

// C/Sģʽ ִ������
#define aplExeMethod(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, APS_ACTION_REQUEST, APS_ACTION_TYPE_EXECUTE);\
	apsDoService();

// C/Sģʽ����Ӧ
#define aplReadResponse(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, APS_ACTION_RESPONSE, APS_ACTION_TYPE_READ);\
	apsDoService();

// C/Sģʽ д��Ӧ
#define aplWriteResponse(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, APS_ACTION_RESPONSE, APS_ACTION_TYPE_WRITE);\
	apsDoService();

// C/Sģʽ ִ����Ӧ
#define aplExeResponse(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, APS_ACTION_RESPONSE, APS_ACTION_TYPE_EXECUTE);\
	apsDoService();





#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST

#define apsDoServiceNoFSM() \
   a_aps_service.status = LOWSN_STATUS_APS_INPROGRESS;\
   apsState = APS_STATE_COMMAND_START;

#define aplSendPublishDataNoFSM(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendPublishData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen);\
	apsDoServiceNoFSM();

#define aplSendReportDataNoFSM(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen)\
	aplFmtSendReportData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen);\
	apsDoServiceNoFSM();

#define aplSendCSDataNoFSM(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, actDirection, actType)\
	aplFmtSendCSData(dstIPADDR, dstPort, dstObjID, srcPort, srcObjID, serviceID, reqACK, pload, plen, actDirection, actType);\
	apsDoServiceNoFSM();


void apsFormatSlipApsHeader(void);


#endif
#endif


// �������: �����豸��Ϣ����
#define mpSendDeviceInfo(dstIPADDR)\
      while(apsBusy()) apsFSM();\
      a_aps_service.cmd = LOWSN_SVC_APS_MP_TX;\
	  a_aps_service.args.mp_tx.commandID = MP_COMMAND_DEVICE_INFORMATION;\
	  a_aps_service.args.mp_tx.dstIPAddr = dstIPADDR;\
      apsDoService();


// �����������: ��������(�ھӽڵ�)  ��Ϣ 
#define mpSendNeighborInfo(dstIPADDR)\
      while(apsBusy()) apsFSM();\
      a_aps_service.cmd = LOWSN_SVC_APS_MP_TX;\
	  a_aps_service.args.mp_tx.commandID = MP_COMMAND_NEIGHBOR_INFORMATION;\
	  a_aps_service.args.mp_tx.dstIPAddr = dstIPADDR;\
      apsDoService();


/*
#define aplSendNodeInfo(saddr)\
      while(apsBusy()) apsFSM();\
      a_aps_service.cmd = LOWSN_SVC_APS_MP_TX;\
      a_aps_service.args.zep_tx.clusterID = ZEP_EXTENDED_CMD;\
	  a_aps_service.args.zep_tx.extID = ZEP_EXT_NODE_INFO_RSP;\
	  a_aps_service.args.zep_tx.dst = saddr;\
      apsDoService();


#define aplSendAlarm(saddr,mde)\
      while(apsBusy()) apsFSM();\
      a_aps_service.cmd = LOWSN_SVC_APS_MP_TX;\
      a_aps_service.args.zep_tx.clusterID = ZEP_EXTENDED_CMD;\
	  a_aps_service.args.zep_tx.extID = ZEP_EXT_SEND_ALARM;\
	  a_aps_service.args.zep_tx.dst = saddr;\
	  a_aps_service.args.zep_tx.ext.alarm.mode = mde;\
      apsDoService();


#define aplPingParent()\
      while(apsBusy()) apsFSM();\
      a_aps_service.cmd = LOWSN_SVC_APS_MP_TX;\
      a_aps_service.args.zep_tx.clusterID = ZEP_EXTENDED_CMD;\
	  a_aps_service.args.zep_tx.extID = ZEP_EXT_PING;\
	  a_aps_service.args.zep_tx.dst = mac_pib.macCoordShortAddress;\
	  apsDoService();

#define aplPingNode(saddr)\
      while(apsBusy()) apsFSM();\
      a_aps_service.cmd = LOWSN_SVC_APS_MP_TX;\
      a_aps_service.args.zep_tx.clusterID = ZEP_EXTENDED_CMD;\
	  a_aps_service.args.zep_tx.extID = ZEP_EXT_PING;\
	  a_aps_service.args.zep_tx.dst = saddr;\
	  apsDoService();
*/



// �����IPv6 ping����
#define aplPing(dstMode, dstIPADDR, payloadLen)\
      while(apsBusy()) apsFSM();\
	a_aps_service.cmd = LOWSN_SVC_APS_NWK_PASSTHRU;\
	a_nwk_service.cmd = LOWSN_SVC_NWK_PING6_TX;\
	a_nwk_service.args.tx_ping6.dstaddr = dstIPADDR;\
	a_nwk_service.args.tx_ping6.plen = payloadLen;\
	a_nwk_service.args.ip_kind.dstmode = dstMode; \
	apsDoService();

// ��ȡglobalǰ׺, �����ݵĲ���û��ʹ�ã��õ��ǹ̶��ȴ�ʱ��ֵ
#define aplGetPrefix() \
      while(apsBusy()) apsFSM();\
	a_aps_service.cmd = LOWSN_SVC_APS_NWK_PASSTHRU;\
	a_nwk_service.cmd = LOWSN_SVC_NWK_GET_RA;\
	a_nwk_service.args.get_ra.WaitDuration = LOWSN_DEFAULT_RS_WAIT_DURATION;\
	apsDoService();

#ifdef LOWSN_FFD
// Э�����趨ǰ׺
#define aplSetPrefix(ptprefix, len)    lowsn_ds6_prefix_add(ptprefix, len, 1, 0x40, 0, 0)
#else
#define aplSetPrefix(ptprefix, len)    lowsn_ds6_prefix_add(ptprefix, len, 0)
#endif	


#ifdef LOWSN_COORDINATOR
//only the coordinator can form a network
//example of a passthru from APS to NETWORK
#define aplFormNetwork()\
    while(apsBusy()) apsFSM();\
	a_aps_service.cmd = LOWSN_SVC_APS_NWK_PASSTHRU;\
	a_nwk_service.cmd = LOWSN_SVC_NWK_ADP_PASSTHRU;\
	a_adp_service.cmd = LOWSN_SVC_ADP_FORM_NETWORK;\
	a_adp_service.args.form_network.ScanChannels = LOWSN_DEFAULT_CHANNEL_MASK;\
	a_adp_service.args.form_network.ScanDuration = LOWSN_DEFAULT_CHANNEL_SCAN_DURATION;\
	apsDoService();

#else
//router, RFDs can join a formed network
#define aplJoinNetwork()\
    while(apsBusy()) apsFSM();\
	a_aps_service.cmd = LOWSN_SVC_APS_NWK_PASSTHRU;\
	a_nwk_service.cmd = LOWSN_SVC_NWK_ADP_PASSTHRU;\
	a_adp_service.cmd = LOWSN_SVC_ADP_JOIN_NETWORK;\
	a_adp_service.args.join_network.ScanChannels = LOWSN_DEFAULT_CHANNEL_MASK;\
	a_adp_service.args.join_network.ScanDuration = LOWSN_DEFAULT_CHANNEL_SCAN_DURATION;\
	a_adp_service.args.join_network.RejoinNetwork = FALSE;\
	apsDoService();  

#define aplRejoinNetwork()\
    while(apsBusy()) apsFSM();\
	a_aps_service.cmd = LOWSN_SVC_APS_NWK_PASSTHRU;\
	a_nwk_service.cmd = LOWSN_SVC_NWK_ADP_PASSTHRU;\
	a_adp_service.cmd = LOWSN_SVC_ADP_JOIN_NETWORK;\
	a_adp_service.args.join_network.ScanChannels = LOWSN_DEFAULT_CHANNEL_MASK;\
	a_adp_service.args.join_network.ScanDuration = LOWSN_DEFAULT_CHANNEL_SCAN_DURATION;\
	a_adp_service.args.join_network.RejoinNetwork = TRUE;\
	apsDoService();


#define aplGetRA()\
    while(apsBusy()) apsFSM();\
	a_aps_service.cmd = LOWSN_SVC_APS_NWK_PASSTHRU;\
	a_nwk_service.cmd = LOWSN_SVC_NWK_GET_RA;\
	a_nwk_service.args.get_ra.WaitDuration = LOWSN_DEFAULT_GET_RA_DURATION;\
	apsDoService();

#endif


/****************************************************************

// Ӧ�ò��������

*****************************************************************/

UINT32 aplMacTicksToUs(UINT32 ticks);   //utility function

#define aplInit()               apsInit()
#define apsGenTSN()             (aps_pib.apsTSN++)
#define apsGenServiceID()             (aps_pib.apsServiceID++)

#define aplGetRxSrcSADDR()      (a_aps_rx_data.srcSADDR)
#define aplGetRxMsgLen()        (a_aps_rx_data.usrPlen)
#define aplGetRxMsgData()       (a_aps_rx_data.usrPload)
#define aplGetRxRSSI()          (a_aps_rx_data.orgpkt.rssi)



#define aplGetRxDstPort()              (a_aps_rx_data.dstPort)
#define aplGetRxSrcPort()              (a_aps_rx_data.srcPort)
#define aplGetRxSrcIPAddr()         (a_aps_rx_data.srcAddress)
#define aplGetRxServiceType()         ((UINT8)(APS_GET_SERVICE_TYPE(a_aps_rx_data.fcf)))
#define aplGetRxServiceID()           (a_aps_rx_data.serviceID)
#define aplGetRxActDirection()         ((UINT8)(APS_GET_ACTION_DIRECTION(a_aps_rx_data.fcf)))
#define aplGetRxActType()                ((UINT8)(APS_GET_ACTION_TYPE(a_aps_rx_data.fcf)))
#define aplGetRxDstObjectID()           (a_aps_rx_data.dstObjID)
#define aplGetRxSrcObjectID()           (a_aps_rx_data.srcObjID)
#define aplGetRxShortcutFlag()          (a_aps_rx_data.dstObjID >= APS_SHORTCUT_BASE)
#define aplGetRxShortcutID()             (a_aps_rx_data.srcObjID)   // ��aplGetRxShortcutFlag() ���Ϊ1ʱ����Ч


#define aplGetLastTxTime()     (phy_pib.txStartTime)


#define aplGetStatus()            (a_aps_service.status)
#define aplGetMyShortAddress()    (macGetShortAddr())
#define aplGetParentLongAddress() (&mac_pib.macCoordExtendedAddress)
#define aplGetParentShortAddress() (mac_pib.macCoordShortAddress)
#define aplGetMyDepth()                    (mac_pib.depth)

#define aplSetMacMaxFrameRetries(x)  (mac_pib.macMaxAckRetries=x)
#define aplSetApsMaxFrameRetries(x)  (aps_pib.apscMaxFrameRetries=x)

//indicates when the user buffer has been copied and is now free
#define aplIsUsrBufferFree()  (aps_pib.flags.bits.IsUsrBufferFree == 1)

#ifdef LOWSN_FFD
void aplFormNetworkDirectly();
#endif

/********************************************************
��������:
my_saddr: �ڵ�����Ķ̵�ַ
parent_addr: ָ�����ڵ�Ķ̵�ַ
parent_laddr: ָ��ڵ㸸�ڵ㳤��ַ��ָ��
my_depth: �ڵ�����

ע��: �ڵ�������ַ��ͨ���궨��ֱ���趨��, ���ڴ˽��г�ʼ��.
��������������д��ʱ�򣬿���ָ��Ϊ0, 1
***********************************************************/
void aplJoinNetworkDirectly(UINT16 my_saddr, UINT16 parent_saddr, UINT8 *parent_laddr, UINT8 my_depth);

/****************************************************************

// Ӧ�ò��IPv6��������

*****************************************************************/

/*---------------------------------------------------------------------*

ÿ��16λ����IPv6��ַ:
addr: IPADDR * : Ҫ�趨��IPv6��ַ
addr0-addr7: 16λ��������

����:
IPADDR myip;
aplSetAddr6(&myip,0x0066,0x0077,0x1234,0x5678,0,0,0,1122);

*---------------------------------------------------------------------*/
#define aplSetAddr(addr, addr0,addr1,addr2,addr3,addr4,addr5,addr6,addr7)\
        lowsn_ip6addr(addr, addr0,addr1,addr2,addr3,addr4,addr5,addr6,addr7)


/*---------------------------------------------------------------------*

���ֽ�����IPv6��ַ:

addr: IPADDR * : Ҫ�趨��IPv6��ַ
addr0-addr15: 8λ��������


*---------------------------------------------------------------------*/
#define  aplSetAddrByBytes(addr, addr0,addr1,addr2,addr3,addr4,addr5,addr6,addr7,addr8,addr9,addr10,addr11,addr12,addr13,addr14,addr15) \
	      lowsn_ip6addr_u8(addr, addr0,addr1,addr2,addr3,addr4,addr5,addr6,addr7,addr8,addr9,addr10,addr11,addr12,addr13,addr14,addr15)


/*---------------------------------------------------------------------*

���ݸ�����ǰ׺��EUI64��ַ����IPv6��ַ

addr: IPADDR *, Ҫ�趨��IPv6��ַ
prefix: IPADDR *, һ��ֻ���ǰ׺��IPv6��ַ
(ע��: ǰ׺�����Ѱ����ڸõ�ַ�У��Ҳ��ܳ���64λ)
pEUI64:  BYTE *, ָ��EUI64 8���ֽڻ������ʼ��ַ��EUI64�Ļ��尴���Ķ�˳������

*---------------------------------------------------------------------*/
#define aplSetAddr64(addr, prefix, pEUI64)     do { \
    lowsn_ipaddr_copy(addr, prefix);                          \
    ds6GenInterfaceID_EUI(addr,  pEUI64, 0);             \
  } while(0)


/*---------------------------------------------------------------------*

���ݸ�����ǰ׺��MAC 48��ַ����IPv6��ַ

addr: IPADDR *, Ҫ�趨��IPv6��ַ
prefix: IPADDR *, һ��ֻ���ǰ׺��IPv6��ַ
(ע��: ǰ׺�����Ѱ����ڸõ�ַ�У��Ҳ��ܳ���64λ)
pEUI64:  BYTE *, ָ��MAC 8���ֽڻ������ʼ��ַ��MAC��ַ�Ļ��尴���Ķ�˳������

*---------------------------------------------------------------------*/
#define aplSetAddr48(addr, prefix, pMAC48)     do { \
   lowsn_ipaddr_copy(addr, prefix);                           \
    ds6GenInterfaceID_EUI(addr,  pMAC48, 1);             \
  } while(0)


/*---------------------------------------------------------------------*

���ݸ����ĸ�����ID�Ͷ̵�ַ����link-local IPv6 ��ַ

addr: IPADDR *, Ҫ�趨��IPv6��ַ
prefix: IPADDR *, һ��ֻ���ǰ׺��IPv6��ַ
(ע��: ǰ׺�����Ѱ����ڸõ�ַ�У��Ҳ��ܳ���64λ)
panid: ������ID
short_addr: 16λ�̵�ַ

*---------------------------------------------------------------------*/
#define aplSetAddr16(addr, prefix, panid, short_addr)     do { \
    lowsn_ipaddr_copy(addr, prefix);                                      \
    ds6GenInterfaceID16(addr,  panid, short_addr);             \
  } while(0)





/*---------------------------------------------------------------------*

���ݸ�����EUI64��ַ����link-local IPv6��ַ

addr: IPADDR *, Ҫ�趨��IPv6��ַ
pEUI64:  BYTE *, ָ��EUI64 8���ֽڻ������ʼ��ַ��EUI64�Ļ��尴���Ķ�˳������

*---------------------------------------------------------------------*/
#define aplSetLinkLocalAddr64(addr, pEUI64)     do { \
    lowsn_create_linklocal_prefix(addr);                     \
    ds6GenInterfaceID_EUI(addr,  pEUI64, 0);             \
  } while(0)


/*---------------------------------------------------------------------*

���ݸ�����MAC 48��ַ����link-local IPv6��ַ

addr: IPADDR *, Ҫ�趨��IPv6��ַ
pEUI64:  BYTE *, ָ��MAC 8���ֽڻ������ʼ��ַ��MAC��ַ�Ļ��尴���Ķ�˳������

*---------------------------------------------------------------------*/
#define aplSetLinkLocalAddr48(addr, pMAC48)     do { \
    lowsn_create_linklocal_prefix(addr);                     \
    ds6GenInterfaceID_EUI(addr,  pMAC48, 1);             \
  } while(0)


/*---------------------------------------------------------------------*

���ݸ����ĸ�����ID�Ͷ̵�ַ����link-local IPv6 ��ַ

addr: IPADDR *, Ҫ�趨��IPv6��ַ
panid: ������ID
short_addr: 16λ�̵�ַ

*---------------------------------------------------------------------*/
#define aplSetLinkLocalAddr16(addr, panid, short_addr)     do { \
    lowsn_create_linklocal_prefix(addr);                                  \
    ds6GenInterfaceID16(addr,  panid, short_addr);             \
  } while(0)


/*---------------------------------------------------------------------*

��ȡ�������������PANID

*---------------------------------------------------------------------*/
#define aplGetPanID()   macGetPanID()


/*---------------------------------------------------------------------*

��ȡ�Լ���16λ�̵�ַ��

*---------------------------------------------------------------------*/
#define aplGetShortAddr()  macGetShortAddr()





#endif


