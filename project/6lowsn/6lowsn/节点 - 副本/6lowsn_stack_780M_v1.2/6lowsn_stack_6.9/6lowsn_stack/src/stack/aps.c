
/******************************************************************************************************
*
* 文 件 名：aps.c
*
* 文件描述： 应用层
*
* 创 建 者：Wang Heng
*
* 当前版本：0.50
*
* 修 改 者：
*
* 修改历史：



********************************************************************************************************/

#include "compiler.h"
#include "6lowsn_config.h"         //user configurations
#include "6lowsn_common_types.h"   //types common acrosss most files
#include "ieee_lrwpan_defs.h"
#include "console.h"
#include "debug.h"
#include "memalloc.h"
#include "neighbor.h"
#include "ds.h"
#include "hal.h"
#include "halStack.h"
#include "phy.h"
#include "mac.h"
#include "adp.h"
#include "icmpv6.h"
#include "nwk.h"
#include "mp.h"
#include "aps.h"
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
#include "slip.h"
#endif
#endif

/*********************************2014-09-19 zdp***********************************/
#define Remote_Port 0xF0B4
#define Local_Port 0xF0B3
/********************************************************************************/


#define apsTXIdle() (!aps_pib.flags.bits.TxInProgress)
#define apsTXBusy() (aps_pib.flags.bits.TxInProgress)
#define apsSetTxBusy() aps_pib.flags.bits.TxInProgress = 1
#define apsSetTxIdle() aps_pib.flags.bits.TxInProgress = 0




APS_PIB aps_pib;
APS_SERVICE a_aps_service;
APS_STATE_ENUM apsState;
APS_EP_ELEM apsEndPoints[LOWSN_MAX_ENDPOINTS];

//there can only be one TX in progress at a time, so
//a_aps_tx_data contains the arguments for that TX on the APS layer.
APS_TX_DATA a_aps_tx_data;
APS_RX_DATA a_aps_rx_data;
APS_RX_DATA indirect_data;  //used for holding an indirect packet while being reflected.

static UINT32 aps_utility_timer; 

#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
// 用于准备APS转发至HOST的头部使用
BYTE aps_header_buf[LOWSN_SLIP_APS_HEADER_LEN]; 
#endif
#endif



void apsFSM(void);
void apsTxData(BOOL copy_payload);
void apsTxIPData(BOOL copy_payload);

//locals
static APS_RXSTATE_ENUM apsRxState;
static LOWSN_STATUS_ENUM apsTxFSM_status;

static void apsParseHdr(BYTE *ptr);
static void apsRxFSM(void);
static BOOL apsCheckAck(void);
static BOOL apsCheckCsResponse(void);
static void apsFormatAck(void);
static void apsTxFSM(void);

#ifdef LOWSN_COORDINATOR
void apsRxBuffInit(void);
BOOL apsRxBuffFull(void);
BOOL apsRxBuffEmpty(void);
APS_RX_DATA *apsGetRxPacket(void);
void apsFreeRxPacket(BOOL freemem);
void apsRxBuffAdd (APS_RX_DATA *ptr);

#endif

#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
void apsFormatSlipApsHeader(void);
#endif
#endif


//converts MAC TICKs to Microseconds
UINT32 aplMacTicksToUs(UINT32 ticks){

	return(halMacTicksToUs(ticks));

}

//this sleeps, then restarts the stack and the radio
void aplShutdown(void){
	//wait until stack is idle
	while(apsBusy()) apsFSM();
	DISABLE_GLOBAL_INTERRUPT();  //disable interrupts, if halSleep wants to reenable them, let it.
	halShutdown();
	a_aps_service.status = LOWSN_STATUS_SUCCESS;

}

void aplWarmstart(void){
	a_aps_service.status = macWarmStartRadio();
}



//this is top level init, does inits for other layers
void apsInit(void){

	//debug_level = 0;
	//debug_level = DBG_MAX_LEVEL;
	apsState = APS_STATE_IDLE;
	apsRxState = APS_RXSTATE_IDLE;
#ifdef LOWSN_COORDINATOR
	aps_pib.rxCnt = 0;
	aps_pib.rxTail = 0;
	aps_pib.rxHead = 0;
#endif

	aps_pib.apsTSN = 0;
	aps_pib.apsServiceID = 0;
	aps_pib.apsSeqNum = 0;
	
	aps_pib.flags.val = 0;
	aps_pib.apscAckWaitDuration = MSECS_TO_MACTICKS(LOWSN_APS_ACK_WAIT_DURATION) ; //convert to MAC Ticks
	aps_pib.apsAckWaitMultiplier = 10;  // 目前设置2s内收到ACK，否则认为超时，并未根据跳数设置ACK动态超时时间
	aps_pib.apscMaxFrameRetries = LOWSN_APS_MAX_FRAME_RETRIES ;

	//initialize the endpoints
	aps_pib.activeEPs = 0;  //only tracks user endpoints.
	phyInit();
	macInit();
	adpInit();
	nwkInit();
	mpInit();

}


void aplFmtSendPublishData (IPADDR dstIPADDR,
							UINT16 dstPort,
							BYTE dstObjID,
							UINT16 srcPort,
							BYTE srcObjID,
							BYTE serviceID, 
							BYTE reqACK, 
							BYTE* pload,
							BYTE  plen)
{

	while(apsBusy()) apsFSM();

	a_aps_service.cmd = LOWSN_SVC_APS_GENERIC_TX;
	a_aps_tx_data.flags.val = 0;
	a_aps_tx_data.srcObjID = srcObjID;
	a_aps_tx_data.usrPlen = plen;
	a_aps_tx_data.usrPload = pload;
	a_aps_tx_data.serviceID = serviceID;
	a_aps_tx_data.seqNum= aps_pib.apsSeqNum++;
	a_aps_tx_data.dstObjID = dstObjID;
	
	a_aps_tx_data.dstPort = dstPort;
	a_aps_tx_data.dstIPADDR = dstIPADDR;
	a_aps_tx_data.srcPort = srcPort;

	a_aps_tx_data.fcf = 0;
	APS_SET_SERVICE_TYPE(a_aps_tx_data.fcf, APS_SERVICE_TYPE_PUBLISH);

	a_aps_tx_data.flags.bits.loopback = 0;  //暂不考虑loopback地址

	if (reqACK)  {
		APS_SET_FCF_ACK_FLAG(a_aps_tx_data.fcf);  
	}	

}


void aplFmtSendReportData (IPADDR dstIPADDR,
							UINT16 dstPort,
							BYTE dstObjID,
							UINT16 srcPort,
							BYTE srcObjID,
							BYTE serviceID, 
							BYTE reqACK, 
							BYTE* pload,
							BYTE  plen)
{

	while(apsBusy()) apsFSM();

	a_aps_service.cmd = LOWSN_SVC_APS_GENERIC_TX;
	a_aps_tx_data.flags.val = 0;
	a_aps_tx_data.srcObjID = srcObjID;
	a_aps_tx_data.usrPlen = plen;
	a_aps_tx_data.usrPload = pload;
	a_aps_tx_data.serviceID = serviceID;
	a_aps_tx_data.seqNum= aps_pib.apsSeqNum++;
	a_aps_tx_data.dstObjID = dstObjID;
	
	a_aps_tx_data.dstPort = dstPort;
	a_aps_tx_data.dstIPADDR = dstIPADDR;
	a_aps_tx_data.srcPort = srcPort;

	a_aps_tx_data.fcf = 0;
	APS_SET_SERVICE_TYPE(a_aps_tx_data.fcf, APS_SERVICE_TYPE_REPORT);

	a_aps_tx_data.flags.bits.loopback = 0;  //暂不考虑loopback地址

	if (reqACK)  {
		APS_SET_FCF_ACK_FLAG(a_aps_tx_data.fcf);  
	}	

}



void aplFmtSendCSData(IPADDR dstIPADDR,
				 		UINT16 dstPort,
				 		BYTE dstObjID,
				 		UINT16 srcPort,
				 		BYTE srcObjID,
				 		BYTE  serviceID, 
				 		BYTE  reqACK, 
				 		BYTE* pload,
				 		BYTE  plen, 
				 		BYTE actDirection, 
				 		BYTE actType)
{

	while(apsBusy()) apsFSM();

	a_aps_service.cmd = LOWSN_SVC_APS_GENERIC_TX;
	a_aps_tx_data.flags.val = 0;
	a_aps_tx_data.srcObjID = srcObjID;
	a_aps_tx_data.usrPlen = plen;
	a_aps_tx_data.usrPload = pload;
	a_aps_tx_data.serviceID = serviceID;
	a_aps_tx_data.seqNum= aps_pib.apsSeqNum++;
	a_aps_tx_data.dstObjID = dstObjID;
	
	a_aps_tx_data.dstPort = dstPort;
	a_aps_tx_data.dstIPADDR = dstIPADDR;
	a_aps_tx_data.srcPort = srcPort;

	a_aps_tx_data.fcf = 0;
	APS_SET_SERVICE_TYPE(a_aps_tx_data.fcf, APS_SERVICE_TYPE_CS);
	APS_SET_ACTION_TYPE(a_aps_tx_data.fcf, actType);

	if ( actDirection == APS_ACTION_RESPONSE)  {
		APS_SET_ACTION_RESPONSE(a_aps_tx_data.fcf);
	}	

	if (reqACK)  {
		APS_SET_FCF_ACK_FLAG(a_aps_tx_data.fcf);  
	}	

	if ((actDirection == APS_ACTION_REQUEST) && (actType != APS_ACTION_TYPE_NONE))  {
		a_aps_tx_data.flags.bits.needresponse = 1;
	}	

	a_aps_tx_data.flags.bits.loopback = 0;  //暂不考虑loopback地址





}



void apsFSM(void) 
{
  
	nwkFSM();
	//if TxFSM is busy we need to call it
	if (apsTXBusy()) apsTxFSM();
	apsRxFSM(); //check RX


apsFSM_start: 


	switch (apsState) {
	 case APS_STATE_IDLE:

		 //ackSendPending check must come before the indirect Pending check!
		 if (aps_pib.flags.bits.ackSendPending) {
			 apsState = APS_STATE_ACK_SEND_START;
			 goto apsFSM_start;
		 }

		// 查询是否收到管理进程REQUEST请求
		 if (mpIsEventPending()) {

			switch (mp_pib.eventType)  {
				

			case MP_EVENT_RECEIVE_REQUEST:

				// 判断是读/写/执行
				if (APS_GET_ACTION_TYPE(a_aps_rx_data.fcf) == APS_ACTION_TYPE_READ)  {

                 			if (mpReadCheck()) {
                   				apsState = APS_STATE_MP_READ_RESPONSE;
                   				goto apsFSM_start;
                 			}	
					else
					{
						DEBUG_STRING(DBG_INFO,"MP: Invalid READ request. \n");
					}
								
				}	
				else if (APS_GET_ACTION_TYPE(a_aps_rx_data.fcf) == APS_ACTION_TYPE_WRITE)  {

                 			if (mpWriteCheck()) {
                   				apsState = APS_STATE_MP_WRITE_RESPONSE;
                   				goto apsFSM_start;
                 			}	
					else
					{
						DEBUG_STRING(DBG_INFO,"MP: Invalid WRITE request. \n");
					}		
				}	
				else if (APS_GET_ACTION_TYPE(a_aps_rx_data.fcf) == APS_ACTION_TYPE_EXECUTE)  {
                 			if (mpExecuteCheck()) {
                   				apsState = APS_STATE_MP_EXECUTE_RESPONSE;
                   				goto apsFSM_start;
                 			}	
					else
					{
						DEBUG_STRING(DBG_INFO,"MP: Invalid EXECUTE request. \n");
					}	
				}		
				else {
					
					DEBUG_STRING(DBG_INFO,"MP: Invalid operation method. \n");
				}	
				
			mpClearEventFlag();
			
			break;

			case MP_EVENT_SEND_DEVICE_INFO:
                 			if (phyTxUnLocked()) {
                   				phyGrabTxLock(); 
						mpFmtDeviceInfo(mp_pib.dstAddr);
						mpClearEventFlag();
						
			 			phy_pib.currentTxFlen = 0;  //set frame length to zero, build from scratch
			 			apsTxData(FALSE);           //payload already in TX buffer
						 apsState = APS_STATE_GENERIC_TX_WAIT;
                   				goto apsFSM_start;
                 			}			

			break;

			case MP_EVENT_SEND_TREE_TOPO_INFO:
                 			if (phyTxUnLocked()) {
                   				phyGrabTxLock(); 
						mpFmtTreeNeighborInfo(mp_pib.dstAddr);
						mpClearEventFlag();
						
			 			phy_pib.currentTxFlen = 0;  //set frame length to zero, build from scratch
			 			apsTxData(FALSE);           //payload already in TX buffer
						apsState = APS_STATE_GENERIC_TX_WAIT;
                   				goto apsFSM_start;
                 			}			

			break;

			default:
				mpClearEventFlag();

			break;	

			}
		
		 }

	 break;
	 
	 case APS_STATE_COMMAND_START:

		 switch(a_aps_service.cmd) {
	 case LOWSN_SVC_APS_GENERIC_TX:
		 aps_pib.flags.bits.IsUsrBufferFree = 0;

		 if (a_aps_tx_data.flags.bits.loopback) {
			 apsState = APS_STATE_INJECT_LOOPBACK;  //we are sending a direct packet to ourself!
			 goto apsFSM_start;
		 }

		 //Indirect vs Direct is handled within the apsTxData() function
		 //at this point ready to begin formating packet.
		 //do not exit this state until we can grab the TX buffer.
		 
		 if (phyTxUnLocked()) {
			 phyGrabTxLock(); //prevents lower stack layers from grabbing TX buffer.
			 phy_pib.currentTxFlen = 0;  //set frame length to zero, build from scratch

			 apsTxData(TRUE);		 
			 aps_pib.flags.bits.IsUsrBufferFree = 1;

			 if (a_aps_tx_data.flags.bits.needresponse == 1)  {
				 aps_pib.flags.bits.IsGotCsResponse = 0;
				  aps_pib.flags.bits.WaitCsResponse = 1;
				  apsState = APS_STATE_CS_REQ_TX_WAIT1;
			 }
			 else {
			 	apsState = APS_STATE_GENERIC_TX_WAIT;
			 }	
		 }
		 
		 break;

	 case LOWSN_SVC_APS_MP_TX:
		 if (phyTxUnLocked()) {
			 phyGrabTxLock(); 
			 switch(a_aps_service.args.mp_tx.commandID)
			 {
	           		case MP_COMMAND_DEVICE_INFORMATION:
			        	mpFmtDeviceInfo(a_aps_service.args.mp_tx.dstIPAddr);
					break;
					
			   	case MP_COMMAND_TREE_NEIGHBOR_INFORMATION:
			   		mpFmtTreeNeighborInfo(a_aps_service.args.mp_tx.dstIPAddr);
					break;
					
			   	default: break;
			 }

			 phy_pib.currentTxFlen = 0;  //set frame length to zero, build from scratch
			 apsTxData(FALSE);           //payload already in TX buffer
			 apsState = APS_STATE_GENERIC_TX_WAIT;
		 }

		 break;

	
	 case LOWSN_SVC_APS_NWK_PASSTHRU:
		 //for NWK calls that have to pass thru the APS layer
		 //this just serves to lock the APS layer while the
		 //the NWK layer is doing its thing
		 if (nwkBusy()) break;  //wait until nwk is idle
		 nwkDoService();
		 apsState = APS_STATE_NWK_PASSTHRU_WAIT;
		 break;

	 default: break;
		 }//end switch(a_aps_service.cmd)

		 break;

	 case APS_STATE_ACK_SEND_START:
		 if (phyTxLocked()) break;
		 //send an ACK
		 //lock the TX buffer
		 phyGrabTxLock();
		 //we are now ready
		 apsFormatAck();
		 phy_pib.currentTxFlen = 0;  //set frame length to zero, build from scratch
		 //因为ACK没有属性负载，所以填充时只需要填充头部，通过isackframe标志位来
		 // 告诉TxData函数ACK帧的不同之处，目前没有编写属性填充的部分，所以该标志位
		 // 并未真正工作.
		 a_aps_tx_data.flags.bits.isackframe = 1;  
		 apsTxData(TRUE);
		 a_aps_tx_data.flags.bits.isackframe = 0;  
		 //data sent, release the RX buffer, will let RX FSM resume
		 aps_pib.flags.bits.ackSendPending = 0;
		 apsState = APS_STATE_GENERIC_TX_WAIT;

		 break;

	 case APS_STATE_GENERIC_TX_WAIT:
		 if (!apsTXIdle()) break;
		 //TX is finished, copy status
		 a_aps_service.status = apsTxFSM_status;
		 //release the TX buffer lock before exiting.
		 phyReleaseTxLock();
		 apsState = APS_STATE_IDLE;
		 if (aps_pib.flags.bits.indirectPending) {
			 //have used this state to wait for finishing sending an
			 //ACK back to the source of an indirect transmit. Now
			 //finish resolving the indirect
			 goto apsFSM_start;
		 }


		 break;


	 case APS_STATE_MP_READ_RESPONSE:
	 	
		if (phyTxLocked())  break;
		phyGrabTxLock();
		
		DEBUG_STRING(DBG_INFO,"Sending MP Read Response\n");
		
		// 解析Read的属性ID，并构造response包
           	mpFmtReadResponse();

		mpClearEventFlag();
		phy_pib.currentTxFlen = 0;  //必须设置，长度为0才表明不是重传数据
		apsTxData(FALSE);	
           	apsState = APS_STATE_GENERIC_TX_WAIT;
		   
           	break;


	 case APS_STATE_MP_WRITE_RESPONSE:
	 	
		if (phyTxLocked())  break;
		phyGrabTxLock();

		DEBUG_STRING(DBG_INFO,"Sending MP Write Response\n");
		
		// 解析Write的属性ID，并构造response包
           	mpFmtWriteResponse();

		mpClearEventFlag();

		phy_pib.currentTxFlen = 0;  //必须设置，长度为0才表明不是重传数据
		apsTxData(FALSE);
	
           	apsState = APS_STATE_GENERIC_TX_WAIT;
		   
           	break;


	 case APS_STATE_MP_EXECUTE_RESPONSE:

		//应该 执行完毕再获取发送缓冲锁    待完善
		if (phyTxLocked())  break;
		phyGrabTxLock();
		
		DEBUG_STRING(DBG_INFO,"Sending MP Execute Response\n");
		
		// 解析要执行的方法并进行执行
           	mpExcuteMethod();

		mpClearEventFlag();

		phy_pib.currentTxFlen = 0;  //必须设置，长度为0才表明不是重传数据
		apsTxData(FALSE);
	
             	apsState = APS_STATE_GENERIC_TX_WAIT;
		   
           	break;


	 case APS_STATE_CS_REQ_TX_WAIT1:
		 if (!apsTXIdle()) break;
		 //TX is finished, copy status
		
		 phyReleaseTxLock();

		 if (apsTxFSM_status != LOWSN_STATUS_SUCCESS)  {
		 	a_aps_service.status = apsTxFSM_status;
			aps_pib.flags.bits.WaitCsResponse = 0;
		 	apsState = APS_STATE_IDLE;
			break;
		 }	

           	 aps_utility_timer = halGetMACTimer();
           	 apsState = APS_STATE_CS_REQ_TX_WAIT2;
	 break;

	 case APS_STATE_CS_REQ_TX_WAIT2:
		if (aps_pib.flags.bits.IsGotCsResponse) {
              	a_aps_service.status = LOWSN_STATUS_SUCCESS;
             		aps_pib.flags.bits.WaitCsResponse = 0;
             		apsState = APS_STATE_IDLE;	
             }else if ((halMACTimerNowDelta(aps_utility_timer))>APS_CS_RESPONSE_WAIT_TIME){
             //timeout, give it up
             a_aps_service.status = LOWSN_STATUS_APS_CS_RESPONSE_TIMEOUT;
             DEBUG_STRING(DBG_INFO,"APS: Client/Server wait response timeout\n");
             aps_pib.flags.bits.WaitCsResponse = 0;
             apsState = APS_STATE_IDLE;
           }
			 
	    break;

	 case APS_STATE_NWK_PASSTHRU_WAIT:
		 //for split-phase passthrus
		 if (nwkBusy()) break;
		 a_aps_service.status = a_nwk_service.status;
		 apsState = APS_STATE_IDLE;
		 break;

	 default:  break;


	}//end switch(apsState)


}


void apsTxData(BOOL copy_payload) {

	BYTE *src;

	//if currentTxFlen is zero, we need to build the frame, else, it is
	// a retransmission
	if (phy_pib.currentTxFlen == 0) {
		//assume that the frame is just now being built.
		//use temporary space for building frame
		if (copy_payload){
			//copy user payload into tmpTxBuff space
			//if userPlen is 0, nothing is copied into the payload area
			phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];
			//get a pointer to the end of the payload
			src = a_aps_tx_data.usrPload + a_aps_tx_data.usrPlen;
			phy_pib.currentTxFlen = a_aps_tx_data.usrPlen;
			//now copy the user payload to the frame
			while (phy_pib.currentTxFlen) {
				src--;                //decrement to first src location with data
				phy_pib.currentTxFrm--;     //decrement to free location
				phy_pib.currentTxFlen--;    //decrement length
				*(phy_pib.currentTxFrm) = *src;
			}
		} else {
			//assume that TXBuff already has the payload, the ZEP
			//commands build their payload in this space
			//point currentTxFrm to this payload
			phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE] - a_aps_tx_data.usrPlen;
		}
		//restore length
		phy_pib.currentTxFlen = a_aps_tx_data.usrPlen;
        //2017.05.11
		// 构造6LOWSN应用层头部
	/*
		// Source Object Identifier
		--phy_pib.currentTxFrm; 
		*phy_pib.currentTxFrm = a_aps_tx_data.srcObjID;
		phy_pib.currentTxFlen++;

		// Destination Object Identifier
		--phy_pib.currentTxFrm; 
		*phy_pib.currentTxFrm = a_aps_tx_data.dstObjID;
		phy_pib.currentTxFlen++;

		// Service Identifier
		--phy_pib.currentTxFrm; 
		*phy_pib.currentTxFrm = a_aps_tx_data.serviceID;
		phy_pib.currentTxFlen++;

		// Sequence Number
		--phy_pib.currentTxFrm; 
		*phy_pib.current
		TxFrm = a_aps_tx_data.seqNum;
		phy_pib.currentTxFlen++;

		// Frame Control
		--phy_pib.currentTxFrm; 
		*phy_pib.currentTxFrm = a_aps_tx_data.fcf;	
		phy_pib.currentTxFlen++;

*/
		// add UDP header
		// Checksum暂时填0，填充完网络层头部后再计算
		phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = 0;
              phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = 0;				
               phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;
		//Length
		// Length is the length in octets of this user datagram including this header and the data. 						
		phy_pib.currentTxFrm--;
               *phy_pib.currentTxFrm = (BYTE)((UINT16)(phy_pib.currentTxFlen+6));
               phy_pib.currentTxFrm--;
               *phy_pib.currentTxFrm = (BYTE)(((UINT16)(phy_pib.currentTxFlen+6)) >> 8);			
                phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;
		//Destination Port
		phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = (BYTE)(a_aps_tx_data.dstPort);
               phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = (BYTE)(a_aps_tx_data.dstPort >> 8);				
               phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;
		//Source Port
		phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = (BYTE)(a_aps_tx_data.srcPort);
              phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = (BYTE)(a_aps_tx_data.srcPort >> 8);				
              phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;								
					

		if (a_aps_tx_data.flags.bits.loopback) {
			//Zep commands to ourselves have to go all the way through formatting
			//before we inject them into stack
			//apsInjectTxPacket();
						
		} else {

			//setup the network layer
			a_nwk_tx_data.Version = LOWSN_IP6_VER;
			a_nwk_tx_data.TrafficClass=0x00;
			a_nwk_tx_data.FlowLabel = 0x00;
			a_nwk_tx_data.PayloadLength = phy_pib.currentTxFlen;
			a_nwk_tx_data.NextHeader = LOWSN_PROTO_UDP;
			a_nwk_tx_data.HopLimit = lowsn_ds6_if.cur_hop_limit;
			a_nwk_tx_data.DstAddress = a_aps_tx_data.dstIPADDR; 
			ds6FindSrcIP(&a_nwk_tx_data.SrcAddress, &a_nwk_tx_data.DstAddress);
			//a_nwk_tx_data.IPStartPtr = phy_pib.currentTxFrm - LOWSN_IPH_LEN;

	
			//Send via the network layer
			a_nwk_service.cmd = LOWSN_SVC_NWK_GENERIC_TX;

			
			// at this point, we will attempt a TX
			if (APS_GET_FCF_ACK_FLAG(a_aps_tx_data.fcf)){
							//need an ACK back. set ackPending bit, start timer.
							aps_pib.flags.bits.ackPending = 1;
							aps_pib.tx_start_time = halGetMACTimer();
							//lets compute our Ack Wait duration
							
							//aps_pib.apsAckWaitMultiplier = adpGetHopsToDest(a_nwk_tx_data.dstSADDR);
							// aps_pib.apsAckWaitMultiplier目前固定设定为10，暂不使用动态方式
							
							aps_pib.apsAckWaitMultiplierCntr = aps_pib.apsAckWaitMultiplier;
						}
						else aps_pib.flags.bits.ackPending = 0;
						apsSetTxBusy();
						aps_pib.currentAckRetries = aps_pib.apscMaxFrameRetries; //set retry count
						apsTxFSM_status = LOWSN_STATUS_APS_INPROGRESS;

						//we need to remember this offset in case of a retry, as we
						//will have to reset the flen to this point
						a_aps_tx_data.aps_flen = phy_pib.currentTxFlen;
						a_aps_tx_data.aps_ptr = phy_pib.currentTxFrm;
					}

	}

	if (!a_aps_tx_data.flags.bits.loopback)   {
		nwkDoService();
	}
	
}


//handle RX of packets at APS level
static void apsRxFSM(void)
{
	LOWSN_STATUS_ENUM callback_status;
/*********************2014-06-26 zdp*********************/

  IPADDR SrcIpAddr_G1;                                                                  
	IPADDR DstIpAddr_G1;                                                                  
  int i,j;
  UINT8 buffer_tmp[128];
/******************************************************/  

apsRxFSM_start:

 switch(apsRxState) {
  case APS_RXSTATE_IDLE:
	  break;
  case APS_RXSTATE_START:
	  //we have a packet, lets check it out.
	#if 0
	  if (APS_IS_RSV(a_aps_rx_data.aps_fcf)) {
		  //unknown packet type
		  DEBUG_STRING(DBG_INFO,"APS: Received APS RSV packet, discarding.\n");
		  MemFree(a_aps_rx_data.orgpkt.data);
		  apsRxState = APS_RXSTATE_IDLE;
		  break;
	  }
	  if ((APS_GET_FRM_DLVRMODE(a_aps_rx_data.aps_fcf) == APS_FRM_DLVRMODE_BCAST) ||
		  (APS_GET_FRM_DLVRMODE(a_aps_rx_data.aps_fcf) == APS_FRM_DLVRMODE_RSV)){
			  //Delivery mode not handled.
			  DEBUG_STRING(DBG_INFO,"APS: Received APS packet with BCAST or RSV delivery mode, discarding.\n");
			  MemFree(a_aps_rx_data.orgpkt.data);
			  apsRxState = APS_RXSTATE_IDLE;
			  break;
		  }

	#endif
		  
		  apsParseHdr(a_aps_rx_data.orgpkt.data + a_aps_rx_data.apsOffset);
			
/*		  if (APS_GET_SERVICE_TYPE(a_aps_rx_data.fcf) == APS_SERVICE_TYPE_ACKFRAME) {
			  if (!aps_pib.flags.bits.ackPending) {
				  //not currently expecting an ACK so discard
				  DEBUG_STRING(DBG_INFO,"APS: Received unexpected ACK, discarding.\n");
			  }else {
				  //lets see if this is our ack.
				  if (apsCheckAck()) {
					  DEBUG_STRING(DBG_INFO,"APS: Received APS ack\n");
					  //this is our ACK, clear the ackPending bit
					  aps_pib.flags.bits.ackPending = 0;
				  } else {
					  DEBUG_STRING(DBG_INFO,"APS: Received ACK, did not match expected.\n");
				  }

			  }
			  //NWK,MAC resource already free; need to free the MEM resource
			  MemFree(a_aps_rx_data.orgpkt.data);
			  apsRxState = APS_RXSTATE_IDLE;
			  break;
		  }


		  // 检查Client/Server模式下的response
		  if ((APS_GET_SERVICE_TYPE(a_aps_rx_data.fcf) == APS_SERVICE_TYPE_CS) &&  
		  	(APS_GET_ACTION_DIRECTION(a_aps_rx_data.fcf) == APS_ACTION_RESPONSE))   {
			
			  if (aps_pib.flags.bits.WaitCsResponse)  {
        			if(apsCheckCsResponse()) {
					DEBUG_STRING(DBG_INFO,"APS: Received Client/Server Response!\n");
					aps_pib.flags.bits.IsGotCsResponse = 1;
        			}
				else {
					DEBUG_STRING(DBG_INFO,"APS: Received Client/Server Respons, not match, discarding!\n");
					MemFree(a_aps_rx_data.orgpkt.data);
			  		apsRxState = APS_RXSTATE_IDLE;
			  		break;
				}				
      			  }

			  else {

				DEBUG_STRING(DBG_INFO,"APS: Received Client/Server Respons, not request before, discarding!\n");
				MemFree(a_aps_rx_data.orgpkt.data);
			  	apsRxState = APS_RXSTATE_IDLE;
			  	break;

			  }	

		  }	


   		#ifdef LOWSN_COORDINATOR
    		#ifdef LOWSN_SLIP_TO_HOST
    		if (slip_pib.aps_forward == 1)  {

			apsFormatSlipApsHeader();
        		apsRxState = APS_RXSTATE_FORWARD_HOST;
        		goto apsRxFSM_start;
    		}		
    		#endif
    		#endif


#if 0
		  //check for indirect message
		  if (!a_aps_rx_data.flags.bits.dstEP){
			  //no dest endpoint, must be an indirect message.
#ifdef LOWSN_COORDINATOR
			  //see if we have room to buffer this packet
			  if (apsRxBuffFull()) {
				  //no room, discard this buffer
				  DEBUG_STRING(DBG_INFO,"APS: No room for indirect packet storage, discarding.\n");
				  MemFree(a_aps_rx_data.orgpkt.data);
				  apsRxState = APS_RXSTATE_IDLE;
				  break;
			  }
			  //copy this packet to the APS RX buffer
			  apsRxBuffAdd (&a_aps_rx_data);
			  //first, check to see if an ACK is requested.
			  if (APS_GET_FRM_ACKREQ(a_aps_rx_data.aps_fcf)){
				  //ack request send has to be done in the main FSM
				  aps_pib.flags.bits.ackSendPending = 1;
				  //will have both ackPending and indirectPending set,
				  //the ack pending will be handled first.
			  }

			  //I am the coordinator, must resolve binding
			  aps_pib.flags.bits.indirectPending = 1;
			  apsRxState = APS_RXSTATE_RESOLVE_INDIRECT;
			  goto apsRxFSM_start;
#else
			  //this is an ERROR. Non-coordinator has an unresolved indirect packet.
			  DEBUG_STRING(DBG_INFO,"APS: Non-Coord Received indirect packet, should not happen, check code.\n");
			  MemFree(a_aps_rx_data.orgpkt.data);
			  apsRxState = APS_RXSTATE_IDLE;
			  break;
#endif
		  }

#endif


		  //如果短端口号为0，则转入管理进程
		  if (a_aps_rx_data.dstPort - LOWSN_BASE_PORT == 0) {
			  //not a user endpoint, handle this
			  DEBUG_STRING(DBG_INFO,"APS: Received MP frame.\n");

			 if ((APS_GET_SERVICE_TYPE(a_aps_rx_data.fcf) == APS_SERVICE_TYPE_PUBLISH) ||  
		  	(APS_GET_SERVICE_TYPE(a_aps_rx_data.fcf) == APS_SERVICE_TYPE_REPORT))   {

				callback_status = mpHandleRxReport();
			 }

			 else if (APS_GET_SERVICE_TYPE(a_aps_rx_data.fcf) == APS_SERVICE_TYPE_CS)  {

				if (APS_GET_ACTION_DIRECTION(a_aps_rx_data.fcf) == APS_ACTION_REQUEST)  {

					// 收到请求，置标志位交给主状态机处理
					mpSetEventType(MP_EVENT_RECEIVE_REQUEST);
					mpSetEventFlag();  
      					apsRxState = APS_RXSTATE_MP_REQUEST_PENDING;
      					break;
				}
				else  {

					//处理收到的响应
					DEBUG_STRING(DBG_INFO,"MP: Not support handle CS response now.\n");

				}	
			 }

			 else  {
			 	DEBUG_STRING(DBG_INFO,"MP: Wrong service type.\n");
			 }	
			 	  
			 // callback_status = zepHandleRxPacket();
			  goto apsRxFSM_freepkt;
		  }

*/
		// 其它端口号则转给用户处理
		// 目前尚未支持用户多进程，所以暂未核对端口号是否匹配，以后需增加多进程和
		// 多端口号轮询支持. 
		
		  callback_status = usrRxPacketCallback();

apsRxFSM_freepkt:
		  //finished, free the space
		  MemFree(a_aps_rx_data.orgpkt.data);

		  //see if an ACK is requested. Only send back the ack if the callback status
		  //returned as success!
		  if (APS_GET_FCF_ACK_FLAG(a_aps_rx_data.fcf) && callback_status == LOWSN_STATUS_SUCCESS) {
			  //ack request send has to be done in the main FSM
			  aps_pib.flags.bits.ackSendPending = 1;
			  apsRxState = APS_RXSTATE_ACK_SEND_WAIT;
			  break;
		  }

		  apsRxState = APS_RXSTATE_IDLE;
		  break;

#if 0
#ifdef LOWSN_COORDINATOR
  case APS_RXSTATE_RESOLVE_INDIRECT:
	  if (aps_pib.flags.bits.indirectPending) break;
	  //the main FSM will free up the memory associated with the RX packet

	  apsRxState = APS_RXSTATE_IDLE;
	  break;
#endif
#endif

  case APS_RXSTATE_ACK_SEND_WAIT:
	if (aps_pib.flags.bits.ackSendPending) break;  //waiting for ACK to be sent
	  //main FSM is finished, can now proceed with new RX
	apsRxState = APS_RXSTATE_IDLE;
	break;

  case APS_RXSTATE_MP_REQUEST_PENDING:
	if (mpIsEventPending())  break;
	MemFree(a_aps_rx_data.orgpkt.data);
	apsRxState = APS_RXSTATE_IDLE;
	break;

  #ifdef LOWSN_COORDINATOR
  #ifdef LOWSN_SLIP_TO_HOST
  case APS_RXSTATE_FORWARD_HOST:
  	if (slipTxLocked())  break;

	DEBUG_STRING(DBG_INFO,"APS: Forward APS packet to the host.\n");
	slipGrabTxLock();
#if 0	
	slipSend(&aps_header_buf[0], LOWSN_SLIP_APS_HEADER_LEN, 2);
	slipSend(a_aps_rx_data.usrPload, a_aps_rx_data.usrPlen, 1);
	slipReleaseTxLock();
#endif

#if 1
  for(i=0;i<LOWSN_SLIP_APS_HEADER_LEN;i++)
  	buffer_tmp[i]=aps_header_buf[i];
  if(a_aps_rx_data.usrPlen > 20){
  	MemFree(a_aps_rx_data.orgpkt.data);
  	apsRxState = APS_RXSTATE_IDLE;
  	slipReleaseTxLock();
  	break;
  }
  for(j=0;j<a_aps_rx_data.usrPlen;j++)
  {
	  buffer_tmp[i++]=*(a_aps_rx_data.usrPload+j);
  }
    buffer_tmp[i]='\0';
 /**********************************************/
  #if 0
		DstIpAddr_G1.u8[0] = (0x20);
		DstIpAddr_G1.u8[1] = (0x02);
		DstIpAddr_G1.u8[2] = (0x03);
		DstIpAddr_G1.u8[3] = (0xe8);
		DstIpAddr_G1.u8[4] = (0x90);
		DstIpAddr_G1.u8[5] = (0x0a);
		DstIpAddr_G1.u8[6] = (0x00);
		DstIpAddr_G1.u8[7] = (0x02);
		DstIpAddr_G1.u8[8] = (0x00);
		DstIpAddr_G1.u8[9] = (0x00);
		DstIpAddr_G1.u8[10] = (0x5e);
		DstIpAddr_G1.u8[11] = (0xfe);
		DstIpAddr_G1.u8[12] = (0xac);
		DstIpAddr_G1.u8[13] = (0x16);
		DstIpAddr_G1.u8[14] = (0x8c);
		DstIpAddr_G1.u8[15] = (0x60);
  #endif		

 /**********************************************/
  #if 1
		DstIpAddr_G1.u8[0] = (0xfe);
		DstIpAddr_G1.u8[1] = (0x80);
		DstIpAddr_G1.u8[2] = (0x00);
		DstIpAddr_G1.u8[3] = (0x00);
		DstIpAddr_G1.u8[4] = (0x00);
		DstIpAddr_G1.u8[5] = (0x00);
		DstIpAddr_G1.u8[6] = (0x00);
		DstIpAddr_G1.u8[7] = (0x00);
		DstIpAddr_G1.u8[8] = (0x00);
		DstIpAddr_G1.u8[9] = (0x00);
		DstIpAddr_G1.u8[10] = (0x00);
		DstIpAddr_G1.u8[11] = (0x00);
		DstIpAddr_G1.u8[12] = (0xac);
		
		DstIpAddr_G1.u8[13] = (0x16);
		DstIpAddr_G1.u8[14] = (0x8c);
		DstIpAddr_G1.u8[15] = (0x60);
  #endif
                if(i > 100) {
                	MemFree(a_aps_rx_data.orgpkt.data);
                	apsRxState = APS_RXSTATE_IDLE;
                	slipReleaseTxLock();
                	break;
                }
                else
		  	Nwk_Forward_Outside(0x82, SrcIpAddr_G1, DstIpAddr_G1, Remote_Port, Local_Port, buffer_tmp, i, 17);	
#endif
	MemFree(a_aps_rx_data.orgpkt.data);
	apsRxState = APS_RXSTATE_IDLE;

	break;
  #endif
  #endif

  default:
	  break;
	}



}

//see if this matches expected ack.
static BOOL apsCheckAck(void)
{

	if ((a_aps_rx_data.srcObjID == a_aps_tx_data.dstObjID) && 
	     (a_aps_rx_data.dstObjID == a_aps_tx_data.srcObjID) && 
	     (a_aps_rx_data.serviceID == a_aps_tx_data.serviceID) && 
	     (a_aps_rx_data.seqNum == a_aps_tx_data.seqNum))  {
		//this is our ack
		return TRUE;
	}

	else  {
		return FALSE;
	}	
}


static BOOL apsCheckCsResponse(void)
{

	if (a_aps_rx_data.serviceID == a_aps_tx_data.serviceID )  {
		return TRUE;
	}

	else  {
		return FALSE;
	}	
}


#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
void apsFormatSlipApsHeader(void)
{
	UINT8 i;
	
	aps_header_buf[0] = SLIP_APS_FIRST_FLAG;
	aps_header_buf[1] = SLIP_APS_SECOND_FLAG;
	aps_header_buf[2] = SLIP_APS_COORD_TO_HOST;

	if (a_aps_rx_data.dstPort - LOWSN_BASE_PORT == 0) {
		aps_header_buf[3] = SLIP_APS_MP_FRAME;
	}
	else {
		aps_header_buf[3] = SLIP_APS_DATA_FRAME;
	}	

	aps_header_buf[4] = aplGetRxServiceType();

	for(i=0; i<16; i++) {
              aps_header_buf[5+i] = a_aps_rx_data.srcAddress.u8[i];
       }

	aps_header_buf[21] = (BYTE)(a_aps_rx_data.srcPort >> 8);
       aps_header_buf[22]  = (BYTE)(a_aps_rx_data.srcPort);
	
	aps_header_buf[23] = aplGetRxSrcObjectID();
	
	aps_header_buf[24] = (BYTE)(a_aps_rx_data.dstPort >> 8);
       aps_header_buf[25]  = (BYTE)(a_aps_rx_data.dstPort);

	aps_header_buf[26] = aplGetRxDstObjectID();
	aps_header_buf[27] = aplGetRxServiceID();
	aps_header_buf[28] = aplGetRxRSSI(); 
	aps_header_buf[29] = aplGetRxActDirection();
	aps_header_buf[30] = aplGetRxActType();
	aps_header_buf[31] = 0;  //目前暂时不使用16位数据长度的高位
	aps_header_buf[32] = aplGetRxMsgLen();

}
#endif
#endif





//Callback from NWK Layer
//Returns TRUE if aps is still busy with last RX packet.

BOOL apsRxBusy(void){
	return(apsRxState != APS_RXSTATE_IDLE);
}

void apsRxHandoff(void){
	
	a_aps_rx_data.orgpkt.data = a_nwk_rx_data.orgpkt.data;
	a_aps_rx_data.orgpkt.rssi = a_nwk_rx_data.orgpkt.rssi;
	a_aps_rx_data.apsOffset = a_nwk_rx_data.pload_offset;
	a_aps_rx_data.srcAddress = a_nwk_rx_data.SrcAddress;
	a_aps_rx_data.udp_plen= a_nwk_rx_data.PayloadLength - LOWSN_UDPH_LEN;  //UDP头部本身也带长度域，仍然传递上来，以备将来压缩UDP时使用
	
	a_aps_rx_data.srcSADDR = a_nwk_rx_data.srcSADDR;
	a_aps_rx_data.aps_fcf = *(a_aps_rx_data.orgpkt.data + a_aps_rx_data.apsOffset + LOWSN_UDPH_LEN);//get first byte
	a_aps_rx_data.flags.val = 0;
	apsRxState = APS_RXSTATE_START;

}



//parse the APS header by IP packet
static void apsParseHdr(BYTE *ptr)
{

	// get source port
	//a_aps_rx_data.srcPort = ((UINT16)(*ptr)) << 8 | (UINT16)(*(ptr+1));

	a_aps_rx_data.srcPort = (((UINT16)*ptr) << 8);
	ptr++;
	a_aps_rx_data.srcPort += *ptr;
	ptr++;

	// get destination port
	a_aps_rx_data.dstPort = (((UINT16)*ptr) << 8);
	ptr++;
	a_aps_rx_data.dstPort += *ptr;
	ptr++;
	
	// 应判断UDP端口的可达性，如果不可达，应返回ICMP报文，待以后实现

	// get udp length, only payload
	a_aps_rx_data.udp_plen = (((UINT16)*ptr) << 8);
	ptr++;
	a_aps_rx_data.udp_plen += *ptr;
	ptr++;

	a_aps_rx_data.udp_plen -= LOWSN_UDPH_LEN;

	
	ptr = ptr + 2;

	//get the aps fcf byte
	a_aps_rx_data.fcf = *ptr;
	ptr++;

	// get the Sequence Number
	a_aps_rx_data.seqNum= *ptr;
	ptr++;

	// get the Service Identifier
	a_aps_rx_data.serviceID= *ptr;
	ptr++;

	// get the Destination Object Identifier
	a_aps_rx_data.dstObjID= *ptr;
	ptr++;

	// get the Source Object Identifier
	a_aps_rx_data.srcObjID= *ptr;
	ptr++;

	a_aps_rx_data.attribOffset = LOWSN_UDPH_LEN + 5;

	//get the length of the payload
	a_aps_rx_data.usrPlen = a_aps_rx_data.udp_plen - 5;
	//save the pointer to the payload
	a_aps_rx_data.usrPload = ptr;	

	
	#if 0
	if (!((APS_GET_FRM_DLVRMODE(a_aps_rx_data.aps_fcf) == APS_FRM_DLVRMODE_INDIRECT) &&
		(APS_GET_FRM_INDIRECT_SUBMODE(a_aps_rx_data.aps_fcf)))) {
			//have a destination EP
			a_aps_rx_data.flags.bits.dstEP = 1;
			a_aps_rx_data.dstEP = *ptr;
			ptr++;
			len++;

		}
		//get the cluster ID
		if (APS_GET_FRM_TYPE(a_aps_rx_data.aps_fcf) == APS_FRM_TYPE_DATA){
			a_aps_rx_data.cluster = *ptr;
			ptr++;
		}

		if ((APS_GET_FRM_TYPE(a_aps_rx_data.aps_fcf) == APS_FRM_TYPE_DATA) ||
			(APS_GET_FRM_TYPE(a_aps_rx_data.aps_fcf) == APS_FRM_TYPE_ACK)
			){
				//get the profile ID
				a_aps_rx_data.profile = *ptr;
				ptr++;
				a_aps_rx_data.profile += (((UINT16)*ptr) << 8);
				ptr++;
			}

			len = len +3;

			//get the SRC EP
			if (!((APS_GET_FRM_DLVRMODE(a_aps_rx_data.aps_fcf) == APS_FRM_DLVRMODE_INDIRECT) &&
				(!APS_GET_FRM_INDIRECT_SUBMODE(a_aps_rx_data.aps_fcf)))) {
					//have a SRC EP
					a_aps_rx_data.flags.bits.srcEP = 1;
					a_aps_rx_data.srcEP = *ptr;
					ptr++;
					len++;
				}

				//parse AF frame, assume MSG type
				a_aps_rx_data.afOffset = len;

				//save the af_fcf byte
				a_aps_rx_data.af_fcf = *ptr;
				ptr++;

				//get the transaction number
				a_aps_rx_data.tsn = *ptr;
				ptr++;

				//get the length of the payload
				a_aps_rx_data.usrPlen = *ptr;
				ptr++;
				//save the pointer to the payload
				a_aps_rx_data.usrPload = ptr;
	#endif		

}

//this does not actually format the ACK, just sets
// the aps_tx fields correctly
static void apsFormatAck(void)
{

	a_aps_tx_data.usrPlen = 0;
	a_aps_tx_data.usrPload = NULL;
	a_aps_tx_data.fcf = 0;
	APS_SET_SERVICE_TYPE(a_aps_tx_data.fcf, APS_SERVICE_TYPE_ACKFRAME);
	a_aps_tx_data.seqNum= a_aps_rx_data.seqNum;
	a_aps_tx_data.serviceID= a_aps_rx_data.serviceID;
	a_aps_tx_data.dstObjID= a_aps_rx_data.srcObjID;
	a_aps_tx_data.srcObjID= a_aps_rx_data.dstObjID;

	a_aps_tx_data.dstPort = a_aps_rx_data.srcPort;
	a_aps_tx_data.dstIPADDR = a_aps_rx_data.srcAddress;
	a_aps_tx_data.srcPort = a_aps_rx_data.dstPort;

	a_aps_tx_data.flags.val = 0;

	DEBUG_STRING(DBG_INFO,"APS:Formatted ack\n");


}

//handles retries for APS packets that require ACKs
static void apsTxFSM(void) {
	if(!apsTXIdle()) {
		//we are not idle
		if (nwkIdle()) {
			//cannot check anything until NWK is idle
			if (a_nwk_service.status != LOWSN_STATUS_SUCCESS) {
				//don't bother waiting for ACK, TX did not start correctly
				aps_pib.flags.bits.ackPending = 0;
				apsSetTxIdle();  //mark TX as idle
				apsTxFSM_status = a_nwk_service.status; //return status
			} else if (!aps_pib.flags.bits.ackPending) {
				//either no ACK requested or ACK has been received
				apsSetTxIdle();  //finished successfully, mark as idle
				apsTxFSM_status = LOWSN_STATUS_SUCCESS;
			}
			//check timeout
			else if (halMACTimerNowDelta(aps_pib.tx_start_time)> aps_pib.apscAckWaitDuration)
			{
				//first, check the apsAckWaitMultiplier
				if(aps_pib.apsAckWaitMultiplierCntr) aps_pib.apsAckWaitMultiplierCntr--;  //this is based on number of hops for the apsAck
				if (aps_pib.apsAckWaitMultiplierCntr) {
					//reset the timer.
					aps_pib.tx_start_time = halGetMACTimer();
				}else {
					DEBUG_STRING(1,"APS: TX retry\n");
					// ACK timeout
					aps_pib.currentAckRetries--;
					if (!aps_pib.currentAckRetries) {
						//retries are zero. We have failed.
						apsSetTxIdle();
						DEBUG_STRING(1,"APS TX Retry exceeded\n");
					} else {
						//retry...
						//must reset the len, frm pointers to the beginning of
						// the formatted aps header before retry
						phy_pib.currentTxFlen = a_aps_tx_data.aps_flen;
						phy_pib.currentTxFrm = a_aps_tx_data.aps_ptr;
						aps_pib.tx_start_time = halGetMACTimer();  //reset the timer
						aps_pib.apsAckWaitMultiplierCntr = aps_pib.apsAckWaitMultiplier;
						
						// 对于重传的包，需要将其UDP校验位置0，
						// 否则网络层Tx函数计算校验和时会出错.
						*(phy_pib.currentTxFrm+7) = 0;
						*(phy_pib.currentTxFrm+6) = 0;
						
						apsTxData(TRUE);  //reuse the last packet.
					}
				}
			}

		}

	}

}



#ifdef LOWSN_COORDINATOR

void apsRxBuffInit(void);
BOOL apsRxBuffFull(void);
BOOL apsRxBuffEmpty(void);
APS_RX_DATA *apsGetRxPacket(void);
void apsFreeRxPacket(BOOL freemem);
void apsRxBuffAdd (APS_RX_DATA *ptr);

//copies data into RX buffer
void apsRxBuffAdd (APS_RX_DATA *ptr){
	if (aps_pib.rxCnt == APS_RXBUFF_SIZE) {
		DEBUG_STRING(DBG_ERR,"APS:Trying to add to full buffer in apsRxBuffAdd\n");
		return;
	}
	halUtilMemCopy((BYTE *)&aps_pib.rxBuff[aps_pib.rxHead], (BYTE *)ptr, sizeof(APS_RX_DATA));
	aps_pib.rxCnt++;
	aps_pib.rxHead++; //head points to next free location
	//wrap index
	if (aps_pib.rxHead == APS_RXBUFF_SIZE) aps_pib.rxHead = 0;
}



void apsRxBuffInit(void){
	aps_pib.rxCnt = 0;
	aps_pib.rxTail = 0;
	aps_pib.rxHead = 0;
}

BOOL apsRxBuffFull(void){
	return(aps_pib.rxCnt == APS_RXBUFF_SIZE);
}


BOOL apsRxBuffEmpty(void){
	return(aps_pib.rxCnt == 0);
}

//this does NOT remove the packet from the buffer
APS_RX_DATA *apsGetRxPacket(void) {
	return(&aps_pib.rxBuff[aps_pib.rxTail]);
}

//frees the first packet in the buffer.
void apsFreeRxPacket(BOOL freemem) {
	if (aps_pib.rxCnt == 0) {
		DEBUG_STRING(DBG_ERR,"APS:Trying to free empty buffer in apsFreeRxPacket\n");
		return;
	}
	if (freemem)MemFree(aps_pib.rxBuff[aps_pib.rxTail].orgpkt.data);
	aps_pib.rxCnt--;
	aps_pib.rxTail++;
	if (aps_pib.rxTail == APS_RXBUFF_SIZE) aps_pib.rxTail = 0;

}
#endif

#ifdef LOWSN_FFD
void aplFormNetworkDirectly() 
{
 	adpFormNetworkDirectly(); 
}
#endif

void aplJoinNetworkDirectly(UINT16 my_saddr, UINT16 parent_saddr, UINT8 *parent_laddr, UINT8 my_depth) 
{
            adpJoinNetworkDirectly(my_saddr, parent_saddr, parent_laddr, my_depth);
}

