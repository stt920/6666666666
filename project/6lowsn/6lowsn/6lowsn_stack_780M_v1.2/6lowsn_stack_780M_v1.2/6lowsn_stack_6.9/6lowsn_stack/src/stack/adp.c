
/******************************************************************************************************
*
* 文 件 名：adp.c
*
* 文件描述： 适配层
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
#include "ds.h"
#include "console.h"
#include "debug.h"
#include "memalloc.h"
#include "neighbor.h"
#include "hal.h"
#include "halStack.h"
#include "phy.h"
#include "mac.h"
#include "adp.h"
#include "nwk.h"
#include "mp.h"

#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
#include "slip.h"
#endif
#endif



typedef enum _ADP_RXSTATE_ENUM {
	ADP_RXSTATE_IDLE,
	ADP_RXSTATE_START,
	ADP_RXSTATE_NWK_HANDOFF,
	ADP_RXSTATE_DOROUTE
} ADP_RXSTATE_ENUM;

static ADP_RXSTATE_ENUM adpRxState;
static void adpParseHdr(BYTE *ptr);


ADP_SERVICE a_adp_service;
ADP_STATE_ENUM adpState;
ADP_RX_DATA a_adp_rx_data;

BYTE adpDSN;

ADP_PIB adp_pib;

static void adpRxFSM(void);


//locals
#ifndef LOWSN_COORDINATOR
static UINT32 adp_utility_timer;   //utility timer
static UINT8 adp_retries;       //utility retry counter
#endif

#ifdef LOWSN_FFD
void adpCopyFwdPkt(void);
static BOOL adpRxBuffFull(void);
static BOOL adpRxBuffEmpty(void);
static ADP_FWD_PKT *adpGetRxPacket(void);
static void adpFreeRxPacket(BOOL freemem);
#endif

void nwkRxHandoff(void);


//there can only be one TX in progress at a time, so
//a_nwk_tx_data contains the arguments for that TX on the ADP layer.
ADP_TX_DATA a_adp_tx_data;

void adpTxData(BOOL fwdFlag);

void adpInit(void){
	adpDSN = 0;
	adp_pib.flags.val = 0;
	adp_pib.adpHCMethod = ADP_RX_AUTO_COMPRESS;
	//adp_pib.adpHCMethod = ADP_TX_ALWAYS_UNCOMPRESS;
	adpState = ADP_STATE_IDLE;
	adpRxState= ADP_RXSTATE_IDLE;
#ifdef LOWSN_FFD
	adp_pib.rxTail = 0;
	adp_pib.rxHead = 0;
#endif
}

void adpFSM(void){



	macFSM();
	adpRxFSM();

adpFSM_start:

	switch (adpState) {
	 case ADP_STATE_IDLE:
#ifdef LOWSN_FFD
		 //see if we have packets to forward and can grab the TX buffer
		 if (!adpRxBuffEmpty() && phyTxUnLocked()) {

			 //grab the lock and forward the packet
			 phyGrabTxLock();
			 adpCopyFwdPkt();
			 //transmit it
			 adpTxData(TRUE); //use TRUE as this is a forwarded packet
			 adpState = ADP_STATE_FWD_WAIT;

		 }

#endif
		 break;

	 case ADP_STATE_COMMAND_START:
		 switch(a_adp_service.cmd) {
	 case LOWSN_SVC_ADP_GENERIC_TX:
		 //at this point, need to check  if INDIRECT or DIRECT, take action
		 //send a generic packet with arguments specified by upper level
		 //this assumes the upper level has grabbed the TX buffer lock
		
		a_adp_tx_data.HC1Encoding = 0;
		
		 if (adp_pib.adpHCMethod != ADP_TX_ALWAYS_UNCOMPRESS)  {
		 	
			// 设置HC1 Header
			adpFmtCompHeader();

		 }	

		// 根据压缩情况, 重新排列IPv6头部, 并添加LOWPAN DISPATCH
		// 同时为apdTxData() 函数准备必要的参数.
		 adpTxCompData();
		
		 adpTxData(FALSE);
		 adpState = ADP_STATE_GENERIC_TX_WAIT;
		 break;
#ifdef LOWSN_COORDINATOR
	 case LOWSN_SVC_ADP_FORM_NETWORK:
		 //if forming network, restart everything
		 //if we are forming a network, then we need to restart
		 //the PHY, MAC layers from scratch
		 phyInit();
		 macInit();

		 ntInitTable();  //init neighbor table

		 a_adp_service.status = macInitRadio();  //turns on the radio
		 if (a_adp_service.status != LOWSN_STATUS_SUCCESS) {
			 DEBUG_STRING(DBG_ERR, "ADP Formation failed!\n");
			 adpState = ADP_STATE_IDLE;
			 break;
		 }

		 //this is where we form a network.
		 adpState = ADP_STATE_FORM_NETWORK_START;

		 goto adpFSM_start;
#else

	 case LOWSN_SVC_ADP_JOIN_NETWORK:
		 // see if this is a rejoin or join
		 if (a_adp_service.args.join_network.RejoinNetwork) {
			 mac_pib.flags.bits.macIsAssociated = 0; //if doing rejoin, unsure of association, clear it.
			 adpState = ADP_STATE_REJOIN_NETWORK_START;
			 goto adpFSM_start;  //do not do any other initialization
		 }
		 else adpState = ADP_STATE_JOIN_NETWORK_START;
		 //if joining/rejoining network, restart everything
		 //if we are forming a network, then we need to restart
		 //the PHY, MAC layers from scratch
		 phyInit();
		 macInit();
#ifdef LOWSN_FFD
		 ntInitTable();  //init neighbor table
#endif

		 a_adp_service.status = macInitRadio();  //turns on the radio
		 if (a_adp_service.status != LOWSN_STATUS_SUCCESS) {
			 DEBUG_STRING(DBG_ERR, "ADP JOIN/REJOIN failed!\n");
			 adpState = ADP_STATE_IDLE;
			 break;
		 }

		 goto adpFSM_start;
#endif



	 default: break;

		 }//end switch(a_adp_service.cmd)

	 case ADP_STATE_GENERIC_TX_WAIT:
		 if (macBusy()) break;
		 //mac finished, copy status.
		 a_adp_service.status = a_mac_service.status;
		 adpState = ADP_STATE_IDLE;
		 break;


#ifdef LOWSN_FFD
	 case ADP_STATE_FWD_WAIT:
		 if (macBusy()) break;
		 //mac finished, this is only used for fwding packets
		 //will ignore status for now since there is not much
		 //can do about it. Eventally, with more advanced routing,
		 // will record the status of this link and try an alternate
		 //route on failure.
		 phyReleaseTxLock(); //release the lock
		 adpState = ADP_STATE_IDLE;
		 break;
#endif

	 case ADP_STATE_REJOIN_NETWORK_START:
		 if (macBusy()) break;
		 //send an orphan notification
		 a_mac_service.cmd = LOWSN_SVC_MAC_ORPHAN_NOTIFY;
		 adpState = ADP_STATE_REJOIN_WAIT;
		 macDoService();
		 break;

	 case ADP_STATE_REJOIN_WAIT:
		 if (macBusy()) break;
		 //at this point, rejoin is finished, get status
		 a_adp_service.status = a_mac_service.status;
		 adpState = ADP_STATE_IDLE;
		 break;

#ifdef LOWSN_COORDINATOR
	 case ADP_STATE_FORM_NETWORK_START:

		 //here is where we would scan the channels, etc.
		 //instead, we will just set the channel, and indicate
		 //that the network is formed, and init the neighbor table.
		 //macSetPANID(a_adp_service.args.form_network.PANid);
		 macSetChannel(LOWSN_DEFAULT_START_CHANNEL);
		 macSetShortAddr(0);
		 //initialize address assignment, must be done after the
		 //short address is set
		 ntInitAddressAssignment();

		 //add ourselves to the
		 adp_pib.flags.bits.adpFormed = 1;
		 mac_pib.flags.bits.macIsAssociated = 1; //I am associated with myself!
		 //tell MAC to allow association
		 mac_pib.flags.bits.macAssociationPermit = 1;
		 a_adp_service.status = LOWSN_STATUS_SUCCESS;

		// 形成IPv6 link-local短地址
		ds6LocalIPFrom16();

		 adpState = ADP_STATE_IDLE;
		 break;

#endif
#ifndef LOWSN_COORDINATOR
	 case ADP_STATE_JOIN_NETWORK_START:
		 //if trying to join, do not allow association
		 
		 mac_pib.flags.bits.macAssociationPermit = 0;

#ifdef LOWSN_FORCE_ASSOCIATION_TARGET
		 //if forcing association to particular target, skip beacon request
		 //go to state that will start forced association, selecting channels
		 adpState = ADP_STATE_JOIN_MAC_ASSOC_CHANSELECT;
#else
		 //select a channel
		 a_mac_service.args.beacon_req.LogicalChannel = LOWSN_DEFAULT_START_CHANNEL;
		 //set retries
		 //we always send out at least three BEACON req to make
		 //sure that we adequately poll everybody in the region
		 adp_retries = ADP_GENERIC_RETRIES;
		 mac_pib.bcnDepth = 0xFF;  //initialze the beacon depth response
		 //start the request
		 adpState = ADP_STATE_JOIN_SEND_BEACON_REQ;
#endif
		 goto adpFSM_start;

	 case ADP_STATE_JOIN_SEND_BEACON_REQ:
		 //at this point, we would start scanning channels
		 //sending out beacon requests on each channel
		 // we will only send out a beacon on the default channel, instead of scanning
		 if (macBusy()) break;
		 a_mac_service.cmd = LOWSN_SVC_MAC_BEACON_REQ;
		 adpState = ADP_STATE_JOIN_ADP_WAIT1_BREQ;
		 macDoService();
		 break;

		 //waits for BCN request TX to finish
	 case ADP_STATE_JOIN_ADP_WAIT1_BREQ:
		 if (macBusy()) break;
		 //at this point, the packet has been sent. Now have
		 //to wait for a response. Record
		 adp_utility_timer = halGetMACTimer();
	 
		 adpState =ADP_STATE_JOIN_ADP_WAIT2_BREQ;
		 break;

	 case ADP_STATE_JOIN_ADP_WAIT2_BREQ:

		 //wait for either a BCN response or timeout.
		 if (mac_pib.flags.bits.GotBeaconResponse) {
			 //we have received a BCN response. Try joining this node

			 //for right now just print out debug message
			 DEBUG_STRING(DBG_INFO,"Got BCN Response\n");
			 //keep trying, want to poll everybody in range
			 //and pick the closest one
			 mac_pib.flags.bits.GotBeaconResponse = 0;
		 }else if (halMACTimerNowDelta(adp_utility_timer)> MSECS_TO_MACTICKS(LOWSN_ADP_JOIN_WAIT_DURATION) ) {

			 if (adp_retries) adp_retries--;
			 if (adp_retries) {
				 //retry Beacon Request
				 adpState = ADP_STATE_JOIN_SEND_BEACON_REQ;
				 goto adpFSM_start;
			 }else
			 {
				 //out of retries, check bcnDepth
				 if ( mac_pib.bcnDepth != 0xFF) {
					 //we got a response, so lets try to join
					 adpState = ADP_STATE_JOIN_MAC_ASSOC;
					 //use the same channel
					 a_mac_service.args.assoc_req.LogicalChannel  = a_mac_service.args.beacon_req.LogicalChannel;
					 macSetPANID(mac_pib.bcnPANID);  //use beacon response as PANID
					 DEBUG_STRING(DBG_INFO,"ADP trying association\n");
				 }else {
					 //indicate failure
					 DEBUG_STRING(DBG_INFO,"ADP Join Timeout\n");
					 a_adp_service.status = LOWSN_STATUS_ADP_JOIN_TIMEOUT;
					 adpState = ADP_STATE_IDLE;
				 }

				 //clear flags
				 mac_pib.flags.bits.GotBeaconResponse = 0;
				 mac_pib.flags.bits.WaitingForBeaconResponse = 0;
			 }
		 }
		 break;

	 case ADP_STATE_JOIN_MAC_ASSOC_CHANSELECT:
		 //this will eventually scan channels, for now, just select default
		 a_mac_service.args.assoc_req.LogicalChannel = LOWSN_DEFAULT_START_CHANNEL;
		 adpState = ADP_STATE_JOIN_MAC_ASSOC;
		 goto adpFSM_start;

	 case ADP_STATE_JOIN_MAC_ASSOC:
		 //do association to PANID discovered by beacon request
		 if (macBusy()) break;
		 a_mac_service.cmd = LOWSN_SVC_MAC_ASSOC_REQ;
		 adpState = ADP_STATE_JOIN_MAC_ASSOC_WAIT;
		 macDoService();

		 break;

	 case ADP_STATE_JOIN_MAC_ASSOC_WAIT:
		 if (macBusy()) break;
		 //at this point, association is finished, get status
		 a_adp_service.status = a_mac_service.status;
#ifdef LOWSN_FFD
		 if (a_adp_service.status == LOWSN_STATUS_SUCCESS) {
			 //as a router, initialize address assignment and
			 //begin allowing association
			 ntInitAddressAssignment();
			 mac_pib.flags.bits.macAssociationPermit = 1;
		 }
#endif

		// 形成IPv6 link-local短地址,  形成发送设备信息的管理标志位
		 if (a_adp_service.status == LOWSN_STATUS_SUCCESS) {
		 	ds6LocalIPFrom16();
			
			mpSetEventType(MP_EVENT_SEND_DEVICE_INFO);
			mpSetEventFlag();
		 }

		 adpState = ADP_STATE_IDLE;
		 break;

#endif


		 //these states for FORM NETWORK
	 default:  break;


	}//end switch(adpState)


}


#if 0
//Add the ADP header, then send it to MAC
//if fwdFlag is true, then packet is being forwarded, so adp header
//is already in place, and assume that currentTxFrm and currentTxPLen
//are correct as well, and that the radius byte has been decremented.
void adpTxData(BOOL fwdFlag) {

	//if we are not associated, don't bother sending ADP packet
	if (!mac_pib.flags.bits.macIsAssociated) {
		//call a dummy service that just returns an error code
		//have to do it this way since the caller is expecting a
		//mac service call
		a_mac_service.args.error.status = LOWSN_STATUS_MAC_NOT_ASSOCIATED;
		a_mac_service.cmd = LOWSN_SVC_MAC_ERROR;
		goto adpTxData_sendit;
	}

	#if 0
	if (a_adp_tx_data.radius == 0) {
		DEBUG_STRING(DBG_ERR,"Nwk Radius is zero, discarding packet.\n");
		//can no longer forward this packet.
		a_mac_service.args.error.status =  LOWSN_STATUS_ADP_RADIUS_EXCEEDED;
		a_mac_service.cmd = LOWSN_SVC_MAC_ERROR;
		goto adpTxData_sendit;
	}
	#endif
	


	if (fwdFlag) goto adpTxData_addmac;
		
	//sequence number
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = adpDSN;
	adpDSN++;

	//radius, decrement before sending, receiver will
	//get a value that is one less than this node.
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (--a_adp_tx_data.radius);

	//src address
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.srcSADDR >> 8);
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.srcSADDR);

	//dst address
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.dstSADDR >> 8);
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.dstSADDR);

	//frame control
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = a_adp_tx_data.fcfmsb;
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = a_adp_tx_data.fcflsb;

	//network header is fixed size
	phy_pib.currentTxFlen +=  8;

adpTxData_addmac:
	//fill in the MAC fields. For now, we don't support inter-PAN
	// so the PANID has to be our mac PANID
	a_mac_tx_data.DestPANID = mac_pib.macPANID;
	a_mac_tx_data.SrcPANID = mac_pib.macPANID;

	if (a_adp_tx_data.dstSADDR == LOWSN_SADDR_USE_LADDR ){
		//long address is specified from above.  We assume they know where
		//they are going no routing necessary
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_LADDR|LOWSN_FCF_SRCMODE_LADDR;
		//copy in the long address
		halUtilMemCopy(&a_mac_tx_data.DestAddr.laddr.bytes[0], a_adp_tx_data.dstLADDR, 8);
	} else {
		//lets do some routing
#ifdef LOWSN_RFD
		//RFD's are easy. Always send to parent, our SRC address is always long
		//so that parent can confirm that the RFD is still in their neighbor table
		//will use the parent short address
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
		a_mac_tx_data.DestAddr.saddr = mac_pib.macCoordShortAddress;
#else
		{
			SADDR newDstSADDR;
			//this is router. need to determine the new dstSADDR
			newDstSADDR = a_adp_tx_data.dstSADDR; //default
			DEBUG_STRING(DBG_INFO,"Routing pkt to: ");
            		DEBUG_UINT16(DBG_INFO,newDstSADDR);
			if (a_adp_tx_data.dstSADDR != LOWSN_BCAST_SADDR) {
				//not broadcast address
				newDstSADDR = ntFindNewDst(a_adp_tx_data.dstSADDR);
				DEBUG_STRING(DBG_INFO," through: ");
                		DEBUG_UINT16(DBG_INFO,newDstSADDR);
				if (newDstSADDR == LOWSN_BCAST_SADDR) {
					DEBUG_STRING(DBG_INFO,", UNROUTABLE, error!\n ");
					//error indicator. An unroutable packet from here.
					a_mac_service.args.error.status = LOWSN_STATUS_ADP_PACKET_UNROUTABLE;
					a_mac_service.cmd = LOWSN_SVC_MAC_ERROR;
					goto adpTxData_sendit;
				}
				DEBUG_STRING(DBG_INFO,"\n");
			}

			//fill it in.
			a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
			a_mac_tx_data.DestAddr.saddr = newDstSADDR;
		}
#endif

	}


	//for data frames, we want a MAC level ACK, unless it is a broadcast.
	if ( ((LOWSN_GET_DST_ADDR(a_mac_tx_data.fcfmsb)) == LOWSN_ADDRMODE_SADDR) &&
		a_mac_tx_data.DestAddr.saddr == LOWSN_BCAST_SADDR) {
			//no MAC ACK
			a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_DATA|LOWSN_FCF_INTRAPAN_MASK ;
		}else {
			a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_DATA|LOWSN_FCF_INTRAPAN_MASK |LOWSN_FCF_ACKREQ_MASK;
		}

		//send it.
		a_mac_service.cmd = LOWSN_SVC_MAC_GENERIC_TX;



adpTxData_sendit:

		macDoService();

}

#endif



#ifdef  LOWSN_NO_MESH_HEADER
void adpTxData(BOOL fwdFlag) 
{

	//fill in the MAC fields. For now, we don't support inter-PAN

	// so the PANID has to be our mac PANID
	a_mac_tx_data.DestPANID = mac_pib.macPANID;
	a_mac_tx_data.SrcPANID = mac_pib.macPANID;


	// 目前的策略: 目标地址用长地址，源地址也用长地址; 
	// 目标地址用短地址，源地址也用短地址. 
	
	if (a_adp_tx_data.dstSADDR == LOWSN_SADDR_USE_LADDR ){
		//long address is specified from above.  We assume they know where
		//they are going no routing necessary

		#if LOWSN_SRC_MAC_MODE == 1 
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_LADDR|LOWSN_FCF_SRCMODE_SADDR;
		#elif LOWSN_SRC_MAC_MODE == 2	
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_LADDR|LOWSN_FCF_SRCMODE_LADDR;
		#else
		  a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_LADDR|LOWSN_FCF_SRCMODE_LADDR;
		#endif
		
		//copy in the long address
		halUtilMemCopy(&a_mac_tx_data.DestAddr.laddr.bytes[0], a_adp_tx_data.dstLADDR, 8);
	} else {

		//RFD's are easy. Always send to parent, our SRC address is always long
		//so that parent can confirm that the RFD is still in their neighbor table
		//will use the parent short address
		//a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
		// 源地址和目标地址都使用短地址

		#if LOWSN_SRC_MAC_MODE == 1 
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_SADDR;
		#elif LOWSN_SRC_MAC_MODE == 2	
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
		#else
		 a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_SADDR;
		#endif
		
		a_mac_tx_data.DestAddr.saddr = a_adp_tx_data.dstSADDR;

	}

	//for data frames, we want a MAC level ACK, unless it is a broadcast.
	if ( ((LOWSN_GET_DST_ADDR(a_mac_tx_data.fcfmsb)) == LOWSN_ADDRMODE_SADDR) &&
		a_mac_tx_data.DestAddr.saddr == LOWSN_BCAST_SADDR) {
			//no MAC ACK
			a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_DATA|LOWSN_FCF_INTRAPAN_MASK ;
		}else {
			a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_DATA|LOWSN_FCF_INTRAPAN_MASK |LOWSN_FCF_ACKREQ_MASK;
		}

		//send it.
		a_mac_service.cmd = LOWSN_SVC_MAC_GENERIC_TX;

adpTxData_sendit:

		macDoService();

}


#else
//Add the ADP header, then send it to MAC
//if fwdFlag is true, then packet is being forwarded, so adp header
//is already in place, and assume that currentTxFrm and currentTxPLen
//are correct as well, and that the radius byte has been decremented.
/*---------------------------------------------------------------------------*

添加MESH 头部并发送

//if fwdFlag is true, then packet is being forwarded, so adp header
//is already in place, and assume that currentTxFrm and currentTxPLen
//are correct as well, and that the hops-left has been decremented.

---------------------------------------------------------------------------*/
void adpTxData(BOOL fwdFlag) {

	//if we are not associated, don't bother sending ADP packet
	if (!mac_pib.flags.bits.macIsAssociated) {
		//call a dummy service that just returns an error code
		//have to do it this way since the caller is expecting a
		//mac service call
		a_mac_service.args.error.status = LOWSN_STATUS_MAC_NOT_ASSOCIATED;
		a_mac_service.cmd = LOWSN_SVC_MAC_ERROR;
		goto adpTxData_sendit;
	}


        if (ADP_GET_MESH_HOPLFT(a_adp_tx_data.MeshType) == 0) {
                DEBUG_STRING(DBG_ERR,"Mesh hop left is zero, discarding packet.\n");
		//can no longer forward this packet.
		a_mac_service.args.error.status =  LOWSN_STATUS_ADP_RADIUS_EXCEEDED;
		a_mac_service.cmd = LOWSN_SVC_MAC_ERROR;
		goto adpTxData_sendit;
	}

	
	if (fwdFlag) goto adpTxData_addmac;
		

	//Final Destination Address
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.dstSADDR);
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.dstSADDR >> 8);

	//Originator Address
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.srcSADDR);
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (BYTE) (a_adp_tx_data.srcSADDR >> 8);

	//Mesh type, including Hops Left, decrement before sending, receiver will get a value that is one less than this node.
	--phy_pib.currentTxFrm;
	*phy_pib.currentTxFrm = (--a_adp_tx_data.MeshType);

	phy_pib.currentTxFlen +=  ADP_MESH_ALLSADDR_HEADER_LEN;

adpTxData_addmac:
	//fill in the MAC fields. For now, we don't support inter-PAN
	// so the PANID has to be our mac PANID
	a_mac_tx_data.DestPANID = mac_pib.macPANID;
	a_mac_tx_data.SrcPANID = mac_pib.macPANID;

	if (a_adp_tx_data.dstSADDR == LOWSN_SADDR_USE_LADDR ){
		//long address is specified from above.  We assume they know where
		//they are going no routing necessary
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_LADDR|LOWSN_FCF_SRCMODE_LADDR;
		//copy in the long address
		halUtilMemCopy(&a_mac_tx_data.DestAddr.laddr.bytes[0], a_adp_tx_data.dstLADDR, 8);
	} else {
		//lets do some routing
#ifdef LOWSN_RFD
		//RFD's are easy. Always send to parent, our SRC address is always long
		//so that parent can confirm that the RFD is still in their neighbor table
		//will use the parent short address
		a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
		a_mac_tx_data.DestAddr.saddr = mac_pib.macCoordShortAddress;
#else
		{
			SADDR newDstSADDR;
			//this is router. need to determine the new dstSADDR
			newDstSADDR = a_adp_tx_data.dstSADDR; //default
			DEBUG_STRING(DBG_INFO,"Routing pkt to: ");
            		DEBUG_UINT16(DBG_INFO,newDstSADDR);
			if (a_adp_tx_data.dstSADDR != LOWSN_BCAST_SADDR) {
				//not broadcast address
				newDstSADDR = ntFindNewDst(a_adp_tx_data.dstSADDR);
				DEBUG_STRING(DBG_INFO," through: ");
                		DEBUG_UINT16(DBG_INFO,newDstSADDR);
				if (newDstSADDR == LOWSN_BCAST_SADDR) {
					DEBUG_STRING(DBG_INFO,", UNROUTABLE, error!\n ");
					//error indicator. An unroutable packet from here.
					a_mac_service.args.error.status = LOWSN_STATUS_ADP_PACKET_UNROUTABLE;
					a_mac_service.cmd = LOWSN_SVC_MAC_ERROR;
					goto adpTxData_sendit;
				}
				DEBUG_STRING(DBG_INFO,"\n");
			}

			//fill it in.
			a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
			a_mac_tx_data.DestAddr.saddr = newDstSADDR;
		}
#endif

	}


	//for data frames, we want a MAC level ACK, unless it is a broadcast.
	if ( ((LOWSN_GET_DST_ADDR(a_mac_tx_data.fcfmsb)) == LOWSN_ADDRMODE_SADDR) &&
		a_mac_tx_data.DestAddr.saddr == LOWSN_BCAST_SADDR) {
			//no MAC ACK
			a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_DATA|LOWSN_FCF_INTRAPAN_MASK ;
		}else {
			a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_DATA|LOWSN_FCF_INTRAPAN_MASK |LOWSN_FCF_ACKREQ_MASK;
		}

		//send it.
		a_mac_service.cmd = LOWSN_SVC_MAC_GENERIC_TX;



adpTxData_sendit:

		macDoService();

}
#endif



static void adpRxFSM(void) {
	BYTE *ptr;

adpRxFSM_start:

	switch(adpRxState) {
		case ADP_RXSTATE_IDLE:
			break;
		case ADP_RXSTATE_START:
			//we have a packet, lets check it out.
			ptr = a_adp_rx_data.orgpkt.data + a_adp_rx_data.adpOffset;

			#if 0   //待以后增加路由命令帧后再使用
			if (ADP_IS_CMD(*ptr)) {
				//currently don't handle CMD packets. Discard.
				DEBUG_STRING(DBG_INFO,"ADP: Received ADP CMD packet, discarding.\n");
				//MAC resource already free; need to free the MEM resource
				MemFree(a_adp_rx_data.orgpkt.data);
				adpRxState = ADP_RXSTATE_IDLE;
				break;
			}
			#endif  

			#ifdef  LOWSN_NO_MESH_HEADER

			// 在无MESH头的情况下，不支持路由，所以通过MAC层检查能到这一步，
			// 目标节点已经是当前节点，无须再检查

			// MAC地址未必都是短地址，这里暂时忽略，以后再改正
			a_adp_rx_data.srcSADDR = a_mac_rx_data.SrcAddr.saddr;
			a_adp_rx_data.dstSADDR = a_mac_rx_data.DestAddr.saddr;

			adpParseCompHdr();
			adpRxState = ADP_RXSTATE_NWK_HANDOFF;
			
			#else
			if ((!ADP_IS_MESH_TYPE(*ptr)) || ADP_GET_MESH_ORIGINAL_ADDRTYPE(*ptr) != ADP_MESH_SADDR   
				  || ADP_SET_MESH_FINAL_ADDRTYPE(*ptr) != ADP_MESH_SADDR)  {
				  
				// 未采用MESH路由方式, 或MESH头中含有长地址，丢弃
				DEBUG_STRING(DBG_INFO,"ADP: Received ADP packet with long address or no mesh type, discarding.\n");
				//MAC resource already free; need to free the MEM resource
				MemFree(a_adp_rx_data.orgpkt.data);
				adpRxState = ADP_RXSTATE_IDLE;
				break;
			}
			
			//this is a data packet. do more parsing.
			adpParseHdr(ptr);

			//see if this is for us.
			if ((a_adp_rx_data.dstSADDR == LOWSN_BCAST_SADDR) ||
				(a_adp_rx_data.dstSADDR == LOWSN_SADDR_USE_LADDR) ||
				(a_adp_rx_data.dstSADDR == macGetShortAddr())) {

				adpParseCompHdr();
				//hand this off to the APS layer
				adpRxState = ADP_RXSTATE_NWK_HANDOFF;
			  }
			  else {
					//have to route this packet
					adpRxState = ADP_RXSTATE_DOROUTE;
			   }

			#endif
			
			 goto adpRxFSM_start;

		case ADP_RXSTATE_NWK_HANDOFF:

			//conPrintROMString(" nwkRxState = ");
			//conPrintUINT8(nwkRxState);
			//conPrintROMString(" \n");

			if (nwkRxBusy()) break;    //nwkRX is still busy
			//handoff the current packet
		
			nwkRxHandoff();

			//we are finished with this packet.
			//we don't need to do anything to free this resource other
			// than to change state
			adpRxState = ADP_RXSTATE_IDLE;
			break;


		case ADP_RXSTATE_DOROUTE:
#ifdef LOWSN_RFD
			//RFD somehow got a data packet not intended for it.
			//should never happen, but put code here anyway to discard it.
			DEBUG_STRING(DBG_INFO,"ADP: RFD received spurious datapacket, discarding.\n");
			MemFree(a_adp_rx_data.orgpkt.data);
			adpRxState = ADP_RXSTATE_IDLE;
#else
			//first, check the hop left, if zero, then discard.
			if (!ADP_GET_MESH_HOPLFT((*(a_adp_rx_data.orgpkt.data + a_adp_rx_data.adpOffset))))  {
				DEBUG_STRING(DBG_INFO,"ADP: Data packet is out of hops for dest: ");
				DEBUG_UINT16(DBG_INFO,a_adp_rx_data.dstSADDR);
				DEBUG_STRING(DBG_INFO,", discarding...\n");
				MemFree(a_adp_rx_data.orgpkt.data);
				adpRxState = ADP_RXSTATE_IDLE;
				break;
			}
			DEBUG_STRING(DBG_INFO,"ADP: Routing ADP Packet to: ");
			DEBUG_UINT16(DBG_INFO,a_adp_rx_data.dstSADDR);
			DEBUG_STRING(DBG_INFO,"\n");
			//this packet requires routing, not destined for us.
			if (adpRxBuffFull()) {
				//no more room. discard this packet
				DEBUG_STRING(DBG_INFO,"ADP: FWD buffer full, discarding pkt.\n");
				DEBUG_STRING(DBG_INFO,"ADP state: ");
				DEBUG_UINT8(DBG_INFO,adpState);
				DEBUG_STRING(DBG_INFO,"MAC state: ");
				DEBUG_UINT8(DBG_INFO,macState);
				DEBUG_STRING(DBG_INFO,"\n");
				MemFree(a_adp_rx_data.orgpkt.data);
				adpRxState = ADP_RXSTATE_IDLE;
			}else {
				//ok, add this pkt to the buffer
				adp_pib.rxHead++;
				if (adp_pib.rxHead == ADP_RXBUFF_SIZE) adp_pib.rxHead = 0;
				//save it.
				adp_pib.rxBuff[adp_pib.rxHead].data = a_adp_rx_data.orgpkt.data;
				adp_pib.rxBuff[adp_pib.rxHead].adpOffset = a_adp_rx_data.adpOffset;
				adpRxState = ADP_RXSTATE_IDLE;
				//this packet will be retransmitted by adpFSM
			}

#endif

			break;

		default:
			break;

	}


}



//Callback from MAC Layer
//Returns TRUE if adp is still busy with last RX packet.

BOOL adpRxBusy(void){
	return(adpRxState != ADP_RXSTATE_IDLE);
}

//Callback from MAC Layer
//Hands off parsed packet from MAC layer, frees MAC for parsing
//next packet.
void adpRxHandoff(void){

	a_adp_rx_data.orgpkt.data = a_mac_rx_data.orgpkt->data;
	a_adp_rx_data.orgpkt.rssi = a_mac_rx_data.orgpkt->rssi;
	a_adp_rx_data.adpOffset = a_mac_rx_data.pload_offset;
	adpRxState = ADP_RXSTATE_START;
}

static void adpParseHdr(BYTE *ptr) {

	// 已经进行了地址检查，前提是MESH头中的地址都是短地址
	ptr++;
	
	//get originator address
	a_adp_rx_data.srcSADDR = (UINT16)(*ptr) << 8;
	ptr++;
	a_adp_rx_data.srcSADDR += *ptr;
	ptr++;

	//get final address
	a_adp_rx_data.dstSADDR = (UINT16)(*ptr) << 8;
	ptr++;
	a_adp_rx_data.dstSADDR += *ptr;
	ptr++;
}

#ifdef LOWSN_FFD

//copies packet to forward from heap space to TXbuffer space
void adpCopyFwdPkt(void){
	BYTE *srcptr, len;
	ADP_FWD_PKT *pkt;

	phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];
	//get next PKT
	pkt = adpGetRxPacket();
	
	srcptr = pkt->data;  //points at original packet in henwkpace

	//compute bytes to copy.
	//adpoffset is the offset of the adpheader in the original packet
	len = *(srcptr) - pkt->adpOffset - PACKET_FOOTER_SIZE + 1 ;


	//point this one byte past the end of the packet
	srcptr = srcptr
		+ *(srcptr) //length of original packet, not including this byte
		+ 1         //add one for first byte which contains packet length
		- PACKET_FOOTER_SIZE; //subtract footer bytes, don't want to copy these.
	//save length
	phy_pib.currentTxFlen = len;
	//copy from heap space to TXBuffer space
	do {
		srcptr--; phy_pib.currentTxFrm--;
		*phy_pib.currentTxFrm = *srcptr;
		len--;
	}while(len);
	adpFreeRxPacket(TRUE);  //free this packet
	//some final steps
	//get the dstSADDR, needed for routing.

	a_adp_tx_data.dstSADDR = (UINT16)(*(phy_pib.currentTxFrm+3)) <<8;
	a_adp_tx_data.dstSADDR += *(phy_pib.currentTxFrm+4);

	//decrement the radius before sending it on.
	*phy_pib.currentTxFrm = *phy_pib.currentTxFrm - 1;
	a_adp_tx_data.radius = ADP_GET_MESH_HOPLFT(*phy_pib.currentTxFrm);

	
	//leave the SADDR unchanged as we want to know where this originated from!
#if 0
	//replace the SADDR with our SADDR
	*(phy_pib.currentTxFrm+4) = (BYTE) macGetShortAddr();
	*(phy_pib.currentTxFrm+5) = (BYTE) (macGetShortAddr() >>8);
#endif

}

static BOOL adpRxBuffFull(void){
	BYTE tmp;
	//if next write would go to where Tail is, then buffer is full
	tmp = adp_pib.rxHead+1;
	if (tmp == ADP_RXBUFF_SIZE) tmp = 0;
	return(tmp == adp_pib.rxTail);
}

static BOOL adpRxBuffEmpty(void){
	return(adp_pib.rxTail == adp_pib.rxHead);
}

//this does NOT remove the packet from the buffer
static ADP_FWD_PKT *adpGetRxPacket(void) {
	BYTE tmp;
	if (adp_pib.rxTail == adp_pib.rxHead) return(NULL);
	tmp = adp_pib.rxTail+1;
	if (tmp == ADP_RXBUFF_SIZE) tmp = 0;
	return(&adp_pib.rxBuff[tmp]);
}

//frees the first packet in the buffer.
static void adpFreeRxPacket(BOOL freemem) {
	adp_pib.rxTail++;
	if (adp_pib.rxTail == ADP_RXBUFF_SIZE) adp_pib.rxTail = 0;
	if (freemem) MemFree(adp_pib.rxBuff[adp_pib.rxTail].data);
}

#endif

//given a router child SADDR, find the parent router SADDR
UINT16 adpFindParentSADDR(SADDR childSADDR) {

	UINT8 currentDepth;
	SADDR currentParent;
	SADDR currentRouter;
	SADDR maxSADDR;
	UINT8 i;


	currentDepth = 1;
	currentParent = 0;
	do {
		for (i=0; i<LOWSN_MAX_ROUTERS_PER_PARENT; i++) {
			if (i==0) currentRouter = currentParent+1;
			else currentRouter += ntGetCskip(currentDepth);
			if (childSADDR == currentRouter) return(currentRouter);
			maxSADDR = ntGetMaxSADDR(currentRouter,currentDepth+1);
			if ((childSADDR > currentRouter) && (childSADDR <= maxSADDR))
				break; //must go further down the tree
		}
		currentDepth++;
		currentParent = currentRouter;
	}
	while (currentDepth < LOWSN_MAX_DEPTH-1);
	//if we reach here, could not find an address. Return 0 as an error
	return(0);
}

UINT16 adpGetHopsToDest(SADDR dstSADDR){

	UINT16 numHops;
	SADDR currentParent, maxSADDR;
	UINT8 currentDepth;
	UINT8 i;
	SADDR currentRouter;

	numHops = 1;            //return a minimum value of 1

	currentDepth = 0;
	//first compute hops up the tree then down the tree
	if ( macGetShortAddr() == 0) goto adpGetHopsToDest_down;  //this is the coordinator
	if (macGetShortAddr() == dstSADDR) return(1);  //to myself, should not happen, but return min value
	currentParent = mac_pib.macCoordShortAddress; //start with my parent address
	currentDepth = mac_pib.depth - 1; //depth of my parent.
	do {
		if (currentParent == dstSADDR) return(numHops);  //destination is one of my parent nodes.
		if (currentParent == 0) break;         //at coordinator.
		//compute the max SADDR address range of parent

		maxSADDR = ntGetMaxSADDR(currentParent,currentDepth+1);  //depth of parent's children
		if ((dstSADDR > currentParent) &&  (dstSADDR <= maxSADDR)) {
			//this address is in this router's range, stop going up.
			break;
		}
		//go up a level
		currentDepth--;
		numHops++;
		if (currentDepth == 0 ) currentParent =0;
		else { currentParent = adpFindParentSADDR(currentParent);
		if (!currentParent) {
			//could not find, set numHops to maximum and return
			return(LOWSN_MAX_DEPTH<<1);
		}
		}
	}while(1);

adpGetHopsToDest_down:
	currentDepth++; //increment depth, as this should reflect my current children
	//now search going down.
	do {
		//destination is in the current parent's range
		//see if it is one of the routers or children.
		//first see if it is one of the children of current parent
		numHops++;
		maxSADDR = ntGetMaxSADDR(currentParent,currentDepth);
		if (dstSADDR > (maxSADDR-LOWSN_MAX_NON_ROUTER_CHILDREN) &&
			dstSADDR <= maxSADDR) break;  //it is one of the children nodes
		for (i=0; i<LOWSN_MAX_ROUTERS_PER_PARENT; i++) {
			if (i==0) currentRouter = currentParent+1;
			else currentRouter += ntGetCskip(currentDepth);

			if (dstSADDR == currentRouter) return(currentRouter);
			maxSADDR = ntGetMaxSADDR(currentRouter,currentDepth+1);
			if ((dstSADDR > currentRouter) && (dstSADDR <= maxSADDR))
				break; //must go further down the tree
		}
		if (i == LOWSN_MAX_ROUTERS_PER_PARENT) {
			//must be one of my non-router children, increment hops, return
			return(numHops);
		}
		currentDepth++;
		currentParent = currentRouter;

	}while(currentDepth < LOWSN_MAX_DEPTH-1);

	if (numHops > LOWSN_ADP_MESH_MAXHOP) {
		DEBUG_STRING(DBG_ERR,"adpGetHopsToDest: Error in hop calculation: ");
		DEBUG_UINT8(DBG_ERR,numHops);
		DEBUG_STRING(DBG_ERR,"\n");
		numHops = LOWSN_ADP_MESH_MAXHOP-1;
	}
	return(numHops);
}


				
/*---------------------------------------------------------------------------*

解析6LoWPAN帧格式，支持地址压缩的解析
暂不支持HC2压缩格式.

*---------------------------------------------------------------------------*/
void adpParseCompHdr(void)
{
	UINT8 len;
	UINT8 tmp;
	BYTE *ptr;
        UINT8 i;

	ptr = a_adp_rx_data.orgpkt.data + a_adp_rx_data.adpOffset + ADP_MESH_ALLSADDR_HEADER_LEN;

	a_adp_rx_data.Dispatch = *ptr;
	ptr++;
	len = 1;	

	switch(a_adp_rx_data.Dispatch) {
						
	case ADP_DISPATCH_IPV6:   // 未压缩的IPv6数据包
						
		DEBUG_STRING(DBG_INFO,"ADP: Received uncompressed IPv6 packet.\n");
		//标准IPv6头部作为负载考虑
		a_adp_rx_data.pload_offset = a_adp_rx_data.adpOffset  + ADP_MESH_ALLSADDR_HEADER_LEN + len;
              a_adp_rx_data.hcflag = ADP_RX_IP_UNCOMPRESS;
		break;

	case ADP_DISPATCH_HC1:   // HC1格式压缩包

		DEBUG_STRING(DBG_INFO,"ADP: Received HC1 IPv6 packet.\n");

		a_adp_rx_data.HC1Encoding = *ptr;
		ptr++;
		a_adp_rx_data.HopLimit= *ptr;
		ptr++;

		len = len + 2;

		/*
		 The non-compressed IPv6 field that MUST be always present is the Hop
               Limit (8 bits). This field MUST always follow the encoding fields
               (e.g., "HC1 encoding" as shown in Figure 9), perhaps including other
               future encoding fields). Other non-compressed fields MUST follow the
               Hop Limit as implied by the "HC1 encoding" in the exact same order as
               shown above (Section 10.1): source address prefix (64 bits) and/or
               interface identifier (64 bits), destination address prefix (64 bits)
               and/or interface identifier (64 bits), Traffic Class (8 bits), Flow
                Label (20 bits) and Next Header (8 bits). The actual next header
                (e.g., UDP, TCP, ICMP, etc) follows the non-compressed fields.
            */

		if (ADP_GET_SRC_PREFIX_TYPE(a_adp_rx_data.HC1Encoding) == ADP_PREFIX_PI)  {
				for(i=0; i<8; i++) {
					a_adp_rx_data.SrcAddress.u8[i] = *ptr;
					ptr++;
				}	
				len = len + 8;
		}
		else  {
			lowsn_create_linklocal_prefix(&a_adp_rx_data.SrcAddress);
		}	
							
		if (ADP_GET_SRC_IID_TYPE(a_adp_rx_data.HC1Encoding) == ADP_IID_II)  {
			for(i=0; i<8; i++) {
				a_adp_rx_data.SrcAddress.u8[i+8] = *ptr;
				ptr++;
			}

			len = len + 8;
		}
		else  {
			// 目前暂定mesh路由均采用短地址
			// 由于6lowpan的mesh路由不支持跨PAN，所以暂时假定收到的节点
			// 均与自己处在同一PAN中，即PANID相同.
			ds6GenInterfaceID16(&a_adp_rx_data.SrcAddress, macGetPanID(), a_adp_rx_data.srcSADDR);

		}								  	

		if (ADP_GET_DST_PREFIX_TYPE(a_adp_rx_data.HC1Encoding) == ADP_PREFIX_PI)  {
				for(i=0; i<8; i++) {
					a_adp_rx_data.DstAddress.u8[i] = *ptr;
					ptr++;
				}	
				len = len + 8;
		}
		else  {
			lowsn_create_linklocal_prefix(&a_adp_rx_data.DstAddress);
		}	
							
		if (ADP_GET_DST_IID_TYPE(a_adp_rx_data.HC1Encoding) == ADP_IID_II)  {
			for(i=0; i<8; i++) {
				a_adp_rx_data.DstAddress.u8[i+8] = *ptr;
				ptr++;
			}
			len = len + 8;
							  	
		}
		else  {
			// 目前暂定mesh路由均采用短地址
			// 由于6lowpan的mesh路由不支持跨PAN，所以暂时假定收到的节点
			// 均与自己处在同一PAN中，即PANID相同.
			ds6GenInterfaceID16(&a_adp_rx_data.DstAddress, macGetPanID(), a_adp_rx_data.dstSADDR);

		}			

		if (ADP_GET_TRAFFIC_FLOW_TYPE(a_adp_rx_data.HC1Encoding) == ADP_TRAFFIC_FLOW_INLINE)  {

			a_adp_rx_data.TrafficClass = *ptr;
			ptr++;
								
			//6lowpan标准并未定义字节分裂的情况，视FlowLabel为3个字节, 尽管它只有20bits.
			a_adp_rx_data.FlowLabel = (UINT32)(*ptr) << 16;
			ptr++;
			a_adp_rx_data.FlowLabel += (UINT32)(*ptr) << 8;
			ptr++;
			a_adp_rx_data.FlowLabel += *ptr;
			ptr++;

			len = len + 4;
		}
		else  {
			a_adp_rx_data.TrafficClass= 0;
			a_adp_rx_data.FlowLabel = 0;
		}	
							
		tmp = ADP_GET_NEXT_HEADER(a_adp_rx_data.HC1Encoding);

			if (tmp == ADP_NEXT_HEADER_INLINE)  {
				a_adp_rx_data.NextHeader = *ptr;
				ptr++;
				len++;
			}
			else  if (tmp == ADP_NEXT_HEADER_UDP)  {
				a_adp_rx_data.NextHeader = LOWSN_PROTO_UDP;
			}	
			else  if (tmp == ADP_NEXT_HEADER_ICMP)  {
				a_adp_rx_data.NextHeader = LOWSN_PROTO_ICMP6;
			}								
			else  if (tmp == ADP_NEXT_HEADER_TCP)  {
				a_adp_rx_data.NextHeader = LOWSN_PROTO_TCP;
			}									
			else {
				;
			 }		

			if (ADP_GET_HC2_ENCODING(a_adp_rx_data.HC1Encoding) == ADP_HC2_ENCODING_ENABLE)  {

				DEBUG_STRING(DBG_INFO,"ADP: HC2, not support. \n");
			}

			// 此时已经不存在标准IPv6头部，适配层的pload_offset与NWK的pload_offset
			// 意义一样，都是指解析完IPv6头部后开始的负载位置
			a_adp_rx_data.pload_offset = a_adp_rx_data.adpOffset  + ADP_MESH_ALLSADDR_HEADER_LEN + len;

			// 推导出a_adp_rx_data.PayloadLength
			//  the packet length can be inferred either from layer two ("Frame Length" in the IEEE 802.15.4 PPDU) or from the
                     // "datagram_size" field in the fragment header (if present);
                     // 注意: the first byte in the a_mac_rx_data.orgpkt is the packet length, 即*(a_mac_rx_data.orgpkt->data)
                    // 而该值在handoff函数中被传递为: a_adp_rx_data.orgpkt.data = a_mac_rx_data.orgpkt->data;
		      // 注意: 硬件FIFO传递上来的第一个字节是后续数据包长度，但此长度包括2个字节的FCS，
		      // CC2530将FCS替换为RSSI+CRC结果，字节数不变。所以计算实际负载长度时需要减去这2个字节
			a_adp_rx_data.PayloadLength = (UINT16)(*(a_adp_rx_data.orgpkt.data) - a_adp_rx_data.pload_offset + 1 -2);
			a_adp_rx_data.hcflag = ADP_RX_IP_COMPRESS;
							  						
			break;	

			default:
				DEBUG_STRING(DBG_INFO,"ADP: Received unrecognized compressed IPv6 packet.\n");
				a_adp_rx_data.pload_offset = a_adp_rx_data.adpOffset  + ADP_MESH_ALLSADDR_HEADER_LEN + len;
                            a_adp_rx_data.hcflag = ADP_RX_IP_UNCOMPRESS;
				break;
						
		}
}
						



/*------------------------------------------------------------------------------------*

对已经排列好的IP头部进行压缩，在压缩的同时就改变和填充发送缓冲

压缩所需的各种参数由a_adp_service.args.hc_information 通过NWK层传递下来.

压缩方法:



*------------------------------------------------------------------------------------*/
void adpFmtCompHeader(void)
{

	//DEBUG_STRING(DBG_INFO,"adp: Format HC1 Header. ");

	if (a_adp_service.args.hc_info.NextHeader == LOWSN_PROTO_UDP)  {
		ADP_SET_NEXT_HEADER(a_adp_tx_data.HC1Encoding,ADP_NEXT_HEADER_UDP);

	}
	else if (a_adp_service.args.hc_info.NextHeader == LOWSN_PROTO_ICMP6)  {
		ADP_SET_NEXT_HEADER(a_adp_tx_data.HC1Encoding,ADP_NEXT_HEADER_ICMP);
	}	
	else if (a_adp_service.args.hc_info.NextHeader == LOWSN_PROTO_TCP)  {
		ADP_SET_NEXT_HEADER(a_adp_tx_data.HC1Encoding,ADP_NEXT_HEADER_TCP);
	}
	else
	{
		;
	}

	if ((a_adp_service.args.hc_info.TrafficClass == 0) && (a_adp_service.args.hc_info.FlowLabel == 0))  {
		ADP_SET_TRAFFIC_FLOW_COMP(a_adp_tx_data.HC1Encoding);

	}


	//注意: 本部分的地址压缩务必与ds.c中的选择源地址函数ds6FindSrcIP()
	// 和nwk.c中的选择目标短地址函数nwkFindDstDLLAddr() 所用的地址策略保持一致!

	if(lowsn_is_addr_mcast(&a_adp_service.args.hc_info.DstAddress)){

		// 目标地址是组播地址时，目标地址暂不压缩;
		// 本地地址采用短地址生成的link-local地址, 前缀和IID全部压缩

		 ADP_SET_SRC_PC(a_adp_tx_data.HC1Encoding);
		 ADP_SET_SRC_IC(a_adp_tx_data.HC1Encoding);
	}	

	else if (ds6TestOnLink(&a_adp_service.args.hc_info.DstAddress) == 1){
		
		// 同一前缀的地址，包括link-local地址

		
		// 若是link-local地址，则压缩前缀
		if (lowsn_is_addr_link_local(&a_adp_service.args.hc_info.DstAddress))  {
			ADP_SET_SRC_PC(a_adp_tx_data.HC1Encoding);
			ADP_SET_DST_PC(a_adp_tx_data.HC1Encoding);
		}	

		// 本地地址总是使用 长地址或短地址生成的，所以总压缩
		// 当目标地址是link-local时，按照对方是长是短决定本机地址的长短;
		// 当目标地址是同一前缀但非link-local时，若对方用长地址，则在mes头填写
		// 长地址标示短地址, 长地址在MAC头部传递，本机地址采用短地址生成的方式
		// 采用长地址方式时，总是只能传输一跳, mesh路由不支持多跳.
                //2017.04.19
		//ADP_SET_SRC_IC(a_adp_tx_data.HC1Encoding);

		// 若目标节点IID由长地址或短地址生成，则继续压缩IID
		if ((lowsn_is_addr_genfrom_shortaddr(&a_adp_service.args.hc_info.DstAddress))
		      || (lowsn_is_addr_genfrom_EUI64(&a_adp_service.args.hc_info.DstAddress))) {
			ADP_SET_DST_IC(a_adp_tx_data.HC1Encoding);
			
		}		
		// 若目标节点IID生成物规律，则不压缩; 但此时本地传的源地址
		else
		{
			;
		}	

	}

	else {
		// 目标地址是外网地址, 不进行任何压缩
		;
	}

}


void adpTxCompData(void)
{
	UINT8 i;

	if (a_adp_tx_data.HC1Encoding == 0)  {
		a_adp_tx_data.Dispatch = ADP_DISPATCH_IPV6;

	}
	else {

		a_adp_tx_data.Dispatch = ADP_DISPATCH_HC1;
		
		phy_pib.currentTxFrm = phy_pib.currentTxFrm + LOWSN_IPH_LEN;
		phy_pib.currentTxFlen = phy_pib.currentTxFlen - LOWSN_IPH_LEN;
	
	/*  非压缩域的排列顺序
	The non-compressed IPv6 field that MUST be always present is the Hop
	Limit (8 bits). This field MUST always follow the encoding fields
	(e.g., "HC1 encoding" as shown in Figure 9), perhaps including other
	future encoding fields). Other non-compressed fields MUST follow the
	Hop Limit as implied by the "HC1 encoding" in the exact same order as
	shown above (Section 10.1): source address prefix (64 bits) and/or
	interface identifier (64 bits), destination address prefix (64 bits)
	and/or interface identifier (64 bits), Traffic Class (8 bits), Flow
	Label (20 bits) and Next Header (8 bits). The actual next header
	(e.g., UDP, TCP, ICMP, etc) follows the non-compressed fields.
	*/

		// 暂不支持HC2压缩

		//  查询Next Header

		if (ADP_GET_NEXT_HEADER(a_adp_tx_data.HC1Encoding) == ADP_NEXT_HEADER_INLINE)  {
		
       		phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = a_adp_service.args.hc_info.NextHeader;
			phy_pib.currentTxFlen = phy_pib.currentTxFlen + 1;

		}

		
		//  查询流标记和流量类型是否压缩
		if (ADP_GET_TRAFFIC_FLOW_TYPE(a_adp_tx_data.HC1Encoding) == ADP_TRAFFIC_FLOW_INLINE)  {

			// Flow Label以完整的3个字节表示，RFC4944中没有规范字节中分割的情况
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = (BYTE) (a_adp_service.args.hc_info.FlowLabel);
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = (BYTE) (a_adp_service.args.hc_info.FlowLabel >> 8);
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = ((BYTE) (a_adp_service.args.hc_info.FlowLabel >> 16)) & 0x0F;
			
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = a_adp_service.args.hc_info.TrafficClass;

			phy_pib.currentTxFlen = phy_pib.currentTxFlen + 4;
		}

		//  查询目标地址和源地址的压缩情况
		if (ADP_GET_DST_IID_TYPE(a_adp_tx_data.HC1Encoding) == ADP_IID_II)   {
			for(i=0; i<8; i++) {
       			phy_pib.currentTxFrm--;
             		 	*phy_pib.currentTxFrm = a_adp_service.args.hc_info.DstAddress.u8[15-i];
       		}
			phy_pib.currentTxFlen = phy_pib.currentTxFlen + 8;
		}	
		
		if (ADP_GET_DST_PREFIX_TYPE(a_adp_tx_data.HC1Encoding) == ADP_PREFIX_PI)   {	
			for(i=0; i<8; i++) {
       			phy_pib.currentTxFrm--;
             		 	*phy_pib.currentTxFrm = a_adp_service.args.hc_info.DstAddress.u8[7-i];
       		}
			phy_pib.currentTxFlen = phy_pib.currentTxFlen + 8;
		}		

		if (ADP_GET_SRC_IID_TYPE(a_adp_tx_data.HC1Encoding) == ADP_IID_II)   {
			for(i=0; i<8; i++) {
       			phy_pib.currentTxFrm--;
             		 	*phy_pib.currentTxFrm = a_adp_service.args.hc_info.SrcAddress.u8[15-i];
       		}
			phy_pib.currentTxFlen = phy_pib.currentTxFlen + 8;
		}	
		
		if (ADP_GET_SRC_PREFIX_TYPE(a_adp_tx_data.HC1Encoding) == ADP_PREFIX_PI)   {	
			for(i=0; i<8; i++) {
       			phy_pib.currentTxFrm--;
             		 	*phy_pib.currentTxFrm = a_adp_service.args.hc_info.SrcAddress.u8[7-i];
       		}
			phy_pib.currentTxFlen = phy_pib.currentTxFlen + 8;
		}		

		// 添加Hop Limit
		phy_pib.currentTxFrm--;
		*phy_pib.currentTxFrm = a_adp_service.args.hc_info.HopLimit;
		phy_pib.currentTxFlen++;

		// 添加HC1 Header
		phy_pib.currentTxFrm--;
		*phy_pib.currentTxFrm = a_adp_tx_data.HC1Encoding;
		phy_pib.currentTxFlen++;
	}	

	// 添加LOWPAN DISPATCH
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = a_adp_tx_data.Dispatch;
	phy_pib.currentTxFlen++;	


	// setup the mesh param
	// 源地址暂时总是使用短地址，因为只有短地址才支持树形路由
	// 这只是适配层帧头用短地址，在MAC层帧头，会默认再自动再携带EUI64长地址
	// 当然，MAC层的EUI64仅能使用一跳.
	// MSEH Header中源地址和目标地址均使用短地址.
	
	a_adp_tx_data.srcSADDR = macGetShortAddr();
	a_adp_tx_data.MeshType = ADP_MESH_TYPE_BASE;
	ADP_SET_MESH_ORIGINAL_SADDR(a_adp_tx_data.MeshType);
	ADP_SET_MESH_FINAL_SADDR(a_adp_tx_data.MeshType);
	ADP_SET_MESH_HOPLFT(a_adp_tx_data.MeshType, LOWSN_ADP_MESH_MAXHOP);
	
	//a_adp_tx_data.radius = LOWSN_ADP_MESH_MAXHOP;
	//a_adp_tx_data.fcflsb = ADP_FRM_TYPE_DATA | ADP_PROTOCOL | ADP_SUPPRESS_ROUTE_DISCOVER ;
	
	return;

}


#ifdef LOWSN_FFD
void adpFormNetworkDirectly(void)
{
		 phyInit();
		 macInit();

		 ntInitTable();  

		 macInitRadio();

		 macSetPANID(LOWSN_DEFAULT_PANID);
		 macSetChannel(LOWSN_DEFAULT_START_CHANNEL);
		 macSetShortAddr(0);

		 ntInitAddressAssignment();

		 ds6LocalIPFrom16();

		 adp_pib.flags.bits.adpFormed = 1;
		 mac_pib.flags.bits.macIsAssociated = 1; 
		 mac_pib.flags.bits.macAssociationPermit = 1;
		 
}
#endif

/*-----------------------------------------------------------

直接设置节点入网:

参数设置:
my_saddr: 节点自身的短地址
parent_addr: 指定父节点的短地址
parent_laddr: 指向节点父节点长地址的指针
my_depth: 节点的深度

注意: 节点自身长地址是通过宏定义直接设定的, 不在此进行初始化.
后两项若无须填写的时候，可以指定为0, 1

------------------------------------------------------------*/
void adpJoinNetworkDirectly(UINT16 my_saddr, UINT16 parent_saddr, UINT8 *parent_laddr, UINT8 my_depth)
{

	phyInit();
	macInit();
	#ifdef LOWSN_FFD
	ntInitTable();  //init neighbor table
	#endif

	macInitRadio(); 

	macSetPANID(LOWSN_DEFAULT_PANID);
	macSetChannel(LOWSN_DEFAULT_START_CHANNEL);
	macSetShortAddr(my_saddr);
  	mac_pib.macCoordShortAddress = parent_saddr;
  	mac_pib.depth = my_depth;

	mac_pib.macCoordExtendedAddress.bytes[0] = parent_laddr[0];
	mac_pib.macCoordExtendedAddress.bytes[1] = parent_laddr[1];
	mac_pib.macCoordExtendedAddress.bytes[2] = parent_laddr[2];
	mac_pib.macCoordExtendedAddress.bytes[3] = parent_laddr[3];
	mac_pib.macCoordExtendedAddress.bytes[4] = parent_laddr[4];
	mac_pib.macCoordExtendedAddress.bytes[5] = parent_laddr[5];
	mac_pib.macCoordExtendedAddress.bytes[6] = parent_laddr[6];
	mac_pib.macCoordExtendedAddress.bytes[7] = parent_laddr[7];

	#ifdef LOWSN_FFD
	ntInitAddressAssignment();
	mac_pib.flags.bits.macAssociationPermit = 1;
	#endif
	
	ds6LocalIPFrom16();

	//mpSetEventType(MP_EVENT_SEND_DEVICE_INFO);
	//mpSetEventFlag();

	mac_pib.flags.bits.macIsAssociated = 1;
	
}
