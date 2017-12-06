

#include "compiler.h"
#include "6lowsn_config.h"         //user configurations
#include "6lowsn_common_types.h"   //types common acrosss most files
#include "ieee_lrwpan_defs.h"
#include "console.h"
#include "debug.h"
#include "memalloc.h"
#include "hal.h"
#include "halStack.h"
#include "phy.h"
#include "mac.h"
#include "adp.h"
#include "neighbor.h"
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
#include "slip.h"
#endif
#endif


typedef enum _MAC_RXSTATE_ENUM {
  MAC_RXSTATE_IDLE,
  MAC_RXSTATE_ADP_HANDOFF,
  #ifdef LOWSN_COORDINATOR
  #ifdef LOWSN_SLIP_TO_HOST
  MAC_RXSTATE_FORWARD_HOST, 
  #endif
  #endif
  MAC_RXSTATE_CMD_PENDING
} MAC_RXSTATE_ENUM;

static MAC_RXSTATE_ENUM macRxState;

MAC_PIB mac_pib;
MAC_SERVICE a_mac_service;
MAC_STATE_ENUM macState;
extern UINT8 g_initRadioFlag;

//there can only be one TX in progress at a time, so
//a_mac_tx_data contains the arguments for that TX.
MAC_TX_DATA a_mac_tx_data;

//this is used for parsing of current packet.
MAC_RX_DATA a_mac_rx_data;

LOWSN_STATUS_ENUM macTxFSM_status;


//locals
static UINT32 mac_utility_timer;   //utility timer

//local functions
static void macTxData(void);
static void macTxFSM(void);
static void macParseHdr(void);
static void macRxFSM(void);
static void macParseBeacon(void);
static void macFormatAssocRequest(void);
static BOOL macCheckDataRejection(void);
static BOOL macCheckDataDestAddr(void);
static void macFormatOrphanNotify(void);


#ifndef LOWSN_COORDINATOR
static void macParseOrphanResponse(void);
#endif


#ifdef LOWSN_FFD
static void macFormatBeacon(void);
static void macFormatAssociationResponse(void);
static void macFormatCoordRealign(SADDR orphan_saddr);
#endif

#ifndef LOWSN_COORDINATOR
static void macParseAssocResponse(void);
#endif

//does not turn on radio.
void macInit(void){
  macState = MAC_STATE_IDLE;
  macRxState = MAC_RXSTATE_IDLE;
  mac_pib.macCoordShortAddress = 0;
  mac_pib.flags.val = 0;
  mac_pib.rxTail = 0;
  mac_pib.rxHead = 0;
  mac_pib.macPANID = LOWSN_DEFAULT_PANID;
  mac_pib.macMaxAckRetries = aMaxFrameRetries;
  ntInitAddressMap();  //init the address map
#ifdef LOWSN_COORDINATOR
  mac_pib.depth = 0;
#else
  mac_pib.depth = 1; //depth will be at least one
#endif
  mac_pib.bcnDepth = 0xFF; //remembers depth of node that responded to beacon
  //other capability information
  mac_pib.macCapInfo = 0;
#ifdef LOWSN_ALT_COORDINATOR     //not supported, included for completeness
  LOWSN_SET_CAPINFO_ALTPAN(mac_pib.macCapInfo);
#endif
#ifdef LOWSN_FFD
  LOWSN_SET_CAPINFO_DEVTYPE(mac_pib.macCapInfo);
#endif
#ifdef LOWSN_ACMAIN_POWERED
  LOWSN_SET_CAPINFO_PWRSRC(mac_pib.macCapInfo);
#endif
#ifdef LOWSN_RCVR_ON_WHEN_IDLE
  LOWSN_SET_CAPINFO_RONIDLE(mac_pib.macCapInfo);
#endif
#ifdef LOWSN_SECURITY_CAPABLE
  LOWSN_SET_CAPINFO_SECURITY(mac_pib.macCapInfo);
#endif
  //always allocate a short address
  LOWSN_SET_CAPINFO_ALLOCADDR(mac_pib.macCapInfo);


}

LOWSN_STATUS_ENUM macWarmStartRadio(void){
 halWarmstart();
 a_phy_service.cmd = LOWSN_SVC_PHY_INIT_RADIO; //no args
  a_phy_service.args.phy_init_radio_args.radio_flags.bits.listen_mode = 0;
#ifdef LOWSN_COORDINATOR
  a_phy_service.args.phy_init_radio_args.radio_flags.bits.pan_coordinator = 1;
#else
  a_phy_service.args.phy_init_radio_args.radio_flags.bits.pan_coordinator = 0;
#endif
  phyDoService();
  halSetChannel(phy_pib.phyCurrentChannel);
  halSetRadioPANID(mac_pib.macPANID); //listen on this PANID
  halSetRadioShortAddr(macGetShortAddr());  //non-broadcast, reserved
  return(a_phy_service.status);
}

//this assumes that phyInit, macInit has previously been called.
//turns on the radio

LOWSN_STATUS_ENUM macInitRadio(void) {

  phy_pib.phyCurrentFrequency = LOWSN_DEFAULT_FREQUENCY;
  phy_pib.phyCurrentChannel = LOWSN_DEFAULT_START_CHANNEL;
  if (phy_pib.phyCurrentChannel < 11){
    mac_pib.macAckWaitDuration = SYMBOLS_TO_MACTICKS(600);
  }
  else {
    mac_pib.macAckWaitDuration = SYMBOLS_TO_MACTICKS(600);
  }

  a_phy_service.cmd = LOWSN_SVC_PHY_INIT_RADIO; //no args
  a_phy_service.args.phy_init_radio_args.radio_flags.bits.listen_mode = 0;
#ifdef LOWSN_COORDINATOR
  a_phy_service.args.phy_init_radio_args.radio_flags.bits.pan_coordinator = 1;
#else
  a_phy_service.args.phy_init_radio_args.radio_flags.bits.pan_coordinator = 0;
#endif

  phyDoService();

//#ifdef LOWSN_USE_STATIC_PANID
//  halSetRadioPANID(LOWSN_DEFAULT_PANID); //listen on this PANID
//#else
//  halSetRadioPANID(0xFFFF);      //broadcast
//#endif
 // halSetRadioShortAddr(0xFFFE);  //non-broadcast, reserved


#ifdef LOWSN_USE_STATIC_PANID
  macSetPANID(LOWSN_DEFAULT_PANID); //listen on this PANID
#else
#ifdef LOWSN_COORDINATOR
  {
   //randomly generate a panid, as of yet we do not try to detect collision
   //with another network
   UINT16 x;
   x = halGetRandomByte();
   x = x << 8;
   x = x + halGetRandomByte();
   macSetPANID(x);
   DEBUG_STRING(DBG_INFO,"MAC: Generated PANID is ");
   DEBUG_UINT16(DBG_INFO,macGetPANID());
   DEBUG_STRING(DBG_INFO,"\n");
  }
#else
  macSetPANID(0xFFFF);      //broadcast
#endif
#endif
  //halSetRadioShortAddr(0xFFFE);  //non-broadcast, reserved
  halSetRadioShortAddr(0xFFFF); 

  return(a_phy_service.status);
}

void macSetPANID(UINT16 panid){
  mac_pib.macPANID = panid;
  halSetRadioPANID(mac_pib.macPANID);
}

UINT16 macGetPANID(void){
  return(mac_pib.macPANID);
}


void macSetChannel(BYTE channel){
  phy_pib.phyCurrentChannel = channel;
  halSetChannel(channel);
}

void macSetShortAddr(UINT16 saddr) {
#ifdef LOWSN_RFD
	//when changing the short address for an RFD, always clear the map first
	//since the short address may have changed.
	//for RFDs, there is only one entry
	ntInitAddressMap();
#endif
  ntAddOurselvesToAddressTable(saddr);
  halSetRadioShortAddr(saddr);
}



void macFSM(void) {

  BYTE cmd;
#ifdef LOWSN_FFD
  NAYBORENTRY *nt_ptr;
#endif


#ifdef LOWSN_DEBUG
  //assume 2.4 GHZ
  if (debug_level == 0) {
    mac_pib.macAckWaitDuration = SYMBOLS_TO_MACTICKS(600);
  } else {
    mac_pib.macAckWaitDuration = SYMBOLS_TO_MACTICKS(600);  //give longer due to debug output
  }
#endif

  phyFSM();
  //if TxFSM is busy we need to call it
  if (macTXBusy()) macTxFSM();

  macRxFSM();
  if(g_initRadioFlag){
	   macInitRadio();
	   g_initRadioFlag = 0;
	}
#ifdef LOWSN_FFD
macFSM_start:
#endif

  //check background tasks here

  switch (macState) {
	 case MAC_STATE_IDLE:
           if (mac_pib.flags.bits.macPending ) {
             //there is a MAC CMD packet pending in the RX buffer. Handle it.
             cmd = *(a_mac_rx_data.orgpkt->data + a_mac_rx_data.pload_offset);
             switch (cmd) {
             case LOWSN_MACCMD_BCN_REQ:
               //Beacon Request
#ifdef LOWSN_RFD
               //as an RFD, I do not handle this. Release this.
               mac_pib.flags.bits.macPending = 0;
#else
               //as a Coordinator or Router, I will only respond
               //only respond if association permitted
               //as this is the stack's only use of beacons
               DEBUG_STRING(DBG_INFO,"MAC: Proceed the beacon request frame. \n");
               if (mac_pib.flags.bits.macAssociationPermit) {
                 //will keep spinning through here until TX buffer unlocked
                 if (phyTxUnLocked()) {
                   phyGrabTxLock(); //grab the lock
                   macState = MAC_STATE_SEND_BEACON_RESPONSE;
                   mac_pib.flags.bits.macPending = 0; //release packet
                   goto macFSM_start;
                 }
               }else {
                 //release packet.
                 mac_pib.flags.bits.macPending = 0;
               }


#endif
               break;

			 case LOWSN_MACCMD_ORPHAN:
               //Orphan Notify
#ifdef LOWSN_RFD
               //as an RFD, I do not handle this. Release this.
               mac_pib.flags.bits.macPending = 0;
#else
             //will keep spinning through here until TX buffer unlocked
               if (phyTxUnLocked()) {
                   phyGrabTxLock(); //grab the lock
                   macState = MAC_STATE_HANDLE_ORPHAN_NOTIFY;
                   mac_pib.flags.bits.macPending = 0; //release packet
                   goto macFSM_start;
                }
#endif
             case LOWSN_MACCMD_ASSOC_REQ:

               //Association Request
#ifdef LOWSN_RFD
               //as an RFD, I do not handle this. Release this.
               mac_pib.flags.bits.macPending = 0;
#else
               //as a Coordinator or Router, I can respond
               //only respond if association permitted
               if (mac_pib.flags.bits.macAssociationPermit) {
                 //will keep spinning through here until TX buffer unlocked
                 if (phyTxUnLocked()) {
                   phyGrabTxLock(); //grab the lock
                   macState = MAC_STATE_SEND_ASSOC_RESPONSE;

		    // �������һ��Ǳ�ڵ�bug���ͷŹ����ˣ���Ϊһ�ͷ�MAC����״̬�����п���
		    // �����free���հ��������͹�����Ӧ֡ʱ����Ҫ�ٴν������հ�
		
                   mac_pib.flags.bits.macPending = 0; //release packet
                   goto macFSM_start;
                 }
               }else {
                 //release packet.
                 mac_pib.flags.bits.macPending = 0;
               }
#endif

               break;


             default:
               DEBUG_STRING(1,"MAC: Received MAC CMD that is not currently implemented, discarding.\n");
               mac_pib.flags.bits.macPending = 0;

             }



           }//end if(mac_pib.flags.bits.macPending )

           break;
	 case MAC_STATE_COMMAND_START:
           switch(a_mac_service.cmd) {
	        case LOWSN_SVC_MAC_ERROR:
				//dummy service, just return the status that was passed in
				a_mac_service.status = a_mac_service.args.error.status;
				macState = MAC_STATE_IDLE;
				break;

           case LOWSN_SVC_MAC_GENERIC_TX:
             //send a generic packet with arguments specified by upper level
             macTxData();
             macState = MAC_STATE_GENERIC_TX_WAIT;
             break;
           case LOWSN_SVC_MAC_RETRANSMIT:
             //retransmit the last packet
             //used for frames that are only transmitted once because of no ACK request
             //assumes the TX lock is grabbed, and the TX buffer formatted.
             macSetTxBusy();
             macTxFSM_status = LOWSN_STATUS_MAC_INPROGRESS;
             a_phy_service.cmd = LOWSN_SVC_PHY_TX_DATA;
             phyDoService();
             macState = MAC_STATE_GENERIC_TX_WAIT;
             break;
			
		  case LOWSN_SVC_MAC_ORPHAN_NOTIFY:
              if (phyTxLocked()) break;
              phyGrabTxLock();  //Grab the lock
              //no ack, long SRC, short DST, broadcast PAN
			  a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_MAC;
              a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
			  a_mac_tx_data.DestAddr.saddr = LOWSN_BCAST_PANID;
			  a_mac_tx_data.DestPANID = LOWSN_BCAST_PANID;
			  a_mac_tx_data.SrcPANID = LOWSN_BCAST_PANID;
			  macFormatOrphanNotify();
			  mac_pib.flags.bits.GotOrphanResponse = 0;
			  mac_pib.flags.bits.WaitingForOrphanResponse = 1;
              macTxData();
			  macState = MAC_STATE_ORPHAN_WAIT1;
		      break;

           case LOWSN_SVC_MAC_BEACON_REQ:
             //clear Beacon Response Flag
             mac_pib.flags.bits.GotBeaconResponse =0;        //will be set when get response
             //wait for TX lock to send the beacon request
             if (phyTxLocked()) break;
             phyGrabTxLock();  //Grab the lock
             mac_pib.flags.bits.WaitingForBeaconResponse = 1;  //will be cleared when get response
             //set the channel
             halSetChannel(a_mac_service.args.beacon_req.LogicalChannel);
             //stuff the MAC BEACON REQ command into the TX buffer
             phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
             *phy_pib.currentTxFrm = LOWSN_MACCMD_BCN_REQ;
             phy_pib.currentTxFlen = 1;

             //no MAC ack requested
             a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_MAC|LOWSN_FCF_INTRAPAN_MASK;
             //using no src address, dst address and PAN are both broadcast address
             a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_NOADDR;
             a_mac_tx_data.DestAddr.saddr = LOWSN_BCAST_SADDR;
#ifdef LOWSN_USE_STATIC_PANID
             //we only want to talk to nodes who use this PANID
             //this is not compliant with 802.15.4
             //we do this to reduce the number of responses, only want routers/coord
             //to respond who use this panid
             a_mac_tx_data.DestPANID = LOWSN_DEFAULT_PANID;
#else
             //talk to any nodes willing to send us a beacon
             a_mac_tx_data.DestPANID = LOWSN_BCAST_PANID;
#endif
             macTxData();
             macState = MAC_STATE_GENERIC_TX_WAIT_AND_UNLOCK;
             break;

           case LOWSN_SVC_MAC_ASSOC_REQ:
             //break if the TXBUFFER is locked
             if (phyTxLocked()) break;
             phyGrabTxLock();  //Grab the lock
             //may want to put this in a function
             halSetChannel(a_mac_service.args.assoc_req.LogicalChannel);
             mac_pib.flags.bits.macIsAssociated = 0;  //clear to zero

             macFormatAssocRequest();

             a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_MAC|LOWSN_FCF_ACKREQ_MASK;
#ifdef LOWSN_FORCE_ASSOCIATION_TARGET
             //forced association occurs on DEFAULT PANID
             a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_LADDR|LOWSN_FCF_SRCMODE_LADDR;
             a_mac_tx_data.DestPANID = LOWSN_DEFAULT_PANID;
             a_mac_tx_data.DestAddr.laddr.bytes[0] = parent_addr_B0;
             a_mac_tx_data.DestAddr.laddr.bytes[1] = parent_addr_B1;
             a_mac_tx_data.DestAddr.laddr.bytes[2] = parent_addr_B2;
             a_mac_tx_data.DestAddr.laddr.bytes[3] = parent_addr_B3;
             a_mac_tx_data.DestAddr.laddr.bytes[4] = parent_addr_B4;
             a_mac_tx_data.DestAddr.laddr.bytes[5] = parent_addr_B5;
             a_mac_tx_data.DestAddr.laddr.bytes[6] = parent_addr_B6;
             a_mac_tx_data.DestAddr.laddr.bytes[7] = parent_addr_B7;
#else
             //using short address for DST
             a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_SADDR|LOWSN_FCF_SRCMODE_LADDR;
             //use addressing discovered by beacon request
             a_mac_tx_data.DestAddr.saddr = mac_pib.bcnSADDR;
             a_mac_tx_data.DestPANID = mac_pib.bcnPANID;
#endif

             mac_pib.flags.bits.WaitingForAssocResponse = 1;
             //send to coordinator with Short address = 0;
             a_mac_tx_data.SrcPANID = 0xFFFF;

             //Transmit it
             macTxData();
             macState = MAC_STATE_ASSOC_REQ_WAIT1;
             break;		
           default: break;
           }//end switch cmd
           break;


	 case MAC_STATE_ASSOC_REQ_WAIT1:
           if (!macTXIdle()) break;
           //TX is finished
           phyReleaseTxLock();		
           if (macTxFSM_status != LOWSN_STATUS_SUCCESS) {
             //no sense waiting any longer, nobody responded to MAC-level ACK
             a_mac_service.status = macTxFSM_status;
             mac_pib.flags.bits.WaitingForAssocResponse = 0;
             macState = MAC_STATE_IDLE;			
             break;
           }

           //now need to wait for association response
           //start a timer
           mac_utility_timer = halGetMACTimer();
           macState = MAC_STATE_ASSOC_REQ_WAIT2;
           break;

	 case MAC_STATE_ASSOC_REQ_WAIT2:
           if (mac_pib.flags.bits.macIsAssociated) {
             //association successful, hooray!
             a_mac_service.status = LOWSN_STATUS_SUCCESS;
             mac_pib.flags.bits.WaitingForAssocResponse = 0;
             macState = MAC_STATE_IDLE;	
           }else if ((halMACTimerNowDelta(mac_utility_timer))>MAC_ASSOC_WAIT_TIME  ){
             //timeout on association, give it up
             a_mac_service.status = LOWSN_STATUS_MAC_ASSOCIATION_TIMEOUT;
             DEBUG_STRING(DBG_INFO,"MAC: Association timeout\n");
             mac_pib.flags.bits.WaitingForAssocResponse = 0;
             macState = MAC_STATE_IDLE;	
           }

           break;

		

		 case MAC_STATE_ORPHAN_WAIT1:
		      if (!macTXIdle()) break;
			  //TX is finished
              phyReleaseTxLock();		
			  if (macTxFSM_status != LOWSN_STATUS_SUCCESS){
				  //don't wait, TX failed
				  DEBUG_STRING(DBG_INFO,"Orphan Notify TX failed\n");
				  mac_pib.flags.bits.WaitingForOrphanResponse = 0;
				  a_mac_service.status = macTxFSM_status;
				  macState = MAC_STATE_IDLE;
				  break;
			  }
             //now need to wait for association response
             //start a timer
             mac_utility_timer = halGetMACTimer();
             macState = MAC_STATE_ORPHAN_WAIT2;
             break;

		 case MAC_STATE_ORPHAN_WAIT2:
			 if (mac_pib.flags.bits.GotOrphanResponse) {
                //rejoin successfull
                a_mac_service.status = LOWSN_STATUS_SUCCESS;
                mac_pib.flags.bits.WaitingForOrphanResponse = 0;
                mac_pib.flags.bits.GotOrphanResponse = 0;
                macState = MAC_STATE_IDLE;					
			 } else if ((halMACTimerNowDelta(mac_utility_timer))>MAC_ORPHAN_WAIT_TIME ){
                   //timeout on rejoin, give it up
                  a_mac_service.status = LOWSN_STATUS_MAC_ORPHAN_TIMEOUT;
                  DEBUG_STRING(DBG_INFO,"MAC: Orphan timeout\n");
                  mac_pib.flags.bits.WaitingForOrphanResponse = 0;
                  macState = MAC_STATE_IDLE;	
            }
			break;

	 case MAC_STATE_GENERIC_TX_WAIT:
           if (!macTXIdle()) break;
           //TX is finished, copy status
           a_mac_service.status = macTxFSM_status;
           macState = MAC_STATE_IDLE;	
           break;
           //this is used by MAC CMDs in general which send a packet with no ACK.
	 case MAC_STATE_GENERIC_TX_WAIT_AND_UNLOCK:
           if (!macTXIdle()) break;
           //TX is finished, copy status
           a_mac_service.status = macTxFSM_status;
           macState = MAC_STATE_IDLE;	
           //also unlock TX buffer
           phyReleaseTxLock();
           break;
#ifdef LOWSN_FFD
	 case MAC_STATE_HANDLE_ORPHAN_NOTIFY:
		   //first, check to see if this node is in my neighbor table
		   nt_ptr = ntFindByLADDR(&a_mac_rx_data.SrcAddr.laddr);
		   if (!nt_ptr) {
			   //not my orphan, ignoring
			   DEBUG_STRING(DBG_INFO,"MAC: Received orphan notify, but not my orphan, ignoring.\n");
               macState = MAC_STATE_IDLE;	
               //also unlock TX buffer
               phyReleaseTxLock();
			   break;
		   }
		   DEBUG_STRING(DBG_INFO,"Sending Coord Realign (Orphan response)\n");
		   //at this point, we have an orphan. Send a response.
		   macFormatCoordRealign(mac_addr_tbl[nt_ptr->map_index].saddr);
           goto mac_state_send_assoc_response1;  //shared code
		

	 case MAC_STATE_SEND_BEACON_RESPONSE:
           //got a Beacon Request, send the response
           DEBUG_STRING(DBG_INFO,"Sending BCN Response, PANID: ");
           DEBUG_UINT16(DBG_INFO,mac_pib.macPANID);
           DEBUG_STRING(DBG_INFO,"\n");
	   
           macFormatBeacon();
           a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_BEACON;
           a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_NOADDR|LOWSN_FCF_SRCMODE_SADDR;
           a_mac_tx_data.SrcAddr = macGetShortAddr();
           a_mac_tx_data.SrcPANID = mac_pib.macPANID;
           macTxData();
           macState = MAC_STATE_GENERIC_TX_WAIT_AND_UNLOCK;
           break;

	 case MAC_STATE_SEND_ASSOC_RESPONSE:
		   DEBUG_STRING(DBG_INFO,"Sending Association Response\n");
           macFormatAssociationResponse();
mac_state_send_assoc_response1:

           a_mac_tx_data.fcflsb = LOWSN_FRAME_TYPE_MAC|LOWSN_FCF_ACKREQ_MASK;
           a_mac_tx_data.fcfmsb = LOWSN_FCF_DSTMODE_LADDR|LOWSN_FCF_SRCMODE_LADDR;
           a_mac_tx_data.DestAddr.laddr.bytes[0] = a_mac_rx_data.SrcAddr.laddr.bytes[0];
           a_mac_tx_data.DestAddr.laddr.bytes[1] = a_mac_rx_data.SrcAddr.laddr.bytes[1];
           a_mac_tx_data.DestAddr.laddr.bytes[2] = a_mac_rx_data.SrcAddr.laddr.bytes[2];
           a_mac_tx_data.DestAddr.laddr.bytes[3] = a_mac_rx_data.SrcAddr.laddr.bytes[3];
           a_mac_tx_data.DestAddr.laddr.bytes[4] = a_mac_rx_data.SrcAddr.laddr.bytes[4];
           a_mac_tx_data.DestAddr.laddr.bytes[5] = a_mac_rx_data.SrcAddr.laddr.bytes[5];
           a_mac_tx_data.DestAddr.laddr.bytes[6] = a_mac_rx_data.SrcAddr.laddr.bytes[6];
           a_mac_tx_data.DestAddr.laddr.bytes[7] = a_mac_rx_data.SrcAddr.laddr.bytes[7];
           a_mac_tx_data.DestPANID = mac_pib.macPANID;
           macTxData();
           macState = MAC_STATE_GENERIC_TX_WAIT_AND_UNLOCK;
           break;


#endif


	 default: break;
  }
}


//called by HAL when TX for current packet is finished
void macTxCallback(void) {
  /*if (LOWSN_GET_ACK_REQUEST(*(phy_pib.currentTxFrm))) {
    mac_pib.flags.bits.ackPending = 1;  //we are requesting an ack for this packet
    //record the time of this packet
    mac_pib.tx_start_time = halISRGetMACTimer();
  }*/
}

//we pass in the pointer to the packet
//first byte is packet length
void macRxCallback(BYTE *ptr, BYTE rssi) {

  //if this is an ACK, update the current timeout, else indicate
  // that an ACK is pending
  //   check length                      check frame control field
  if ((*ptr == LOWSN_ACKFRAME_LENGTH ) && LOWSN_IS_ACK(*(ptr+1))) {
    //do not save ACK frames
    //this is an ACK, see if it is our ACK, check DSN
    if (*(ptr+3) == mac_pib.macDSN) {
      //DSN matches, assume this is our ack, clear the ackPending flag
      mac_pib.flags.bits.ackPending = 0;
      //DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_OURACK  );
    }
    //MemFree(ptr);
  } else {
    //save the packet, we assume the Physical/Hal layer has already checked
    //if the MAC buffer has room
    mac_pib.rxHead++;
    if (mac_pib.rxHead == MAC_RXBUFF_SIZE) mac_pib.rxHead = 0;
    //save it.
    mac_pib.rxBuff[mac_pib.rxHead].data = ptr;     //save pointer
    mac_pib.rxBuff[mac_pib.rxHead].rssi = rssi;    //save RSSI value
  }
}

BOOL macRxBuffFull(void){
  BYTE tmp;
  //if next write would go to where Tail is, then buffer is full
  tmp = mac_pib.rxHead+1;
  if (tmp == MAC_RXBUFF_SIZE) tmp = 0;
  return(tmp == mac_pib.rxTail);
}

BOOL macRxBuffEmpty(void){
  return(mac_pib.rxTail == mac_pib.rxHead);
}

//this does NOT remove the packet from the buffer
MACPKT *macGetRxPacket(void) {
  BYTE tmp;	
  if (mac_pib.rxTail == mac_pib.rxHead) return(NULL);
  tmp = mac_pib.rxTail+1;
  if (tmp == MAC_RXBUFF_SIZE) tmp = 0;
  return(&mac_pib.rxBuff[tmp]);
}

//frees the first packet in the buffer.
void macFreeRxPacket(BOOL freemem) {
  mac_pib.rxTail++;
  if (mac_pib.rxTail == MAC_RXBUFF_SIZE) mac_pib.rxTail = 0;
  if (freemem) MemFree(mac_pib.rxBuff[mac_pib.rxTail].data);
}




/************
The TxInProgress bit is set when the macTxData function
is called the first time to actually format the header and
send the packet. After that, each loop through the macTxFSM
checks to see if the TX started and finished correctly, and
if an ACK was received if one was requested.  The FSM becomes
idle if:
a. the PHY TX returns an error
b. the PHY TX returned success and either no ACK was
requested or we received the correct ACK
c. the PHY TX returned success and we exhausted retries.

**************/

static void macTxFSM(void) {
  if(!macTXIdle()) {
    //we are not idle
    if (phyIdle()) {
      //cannot check anything until PHY is idle
      if (a_phy_service.status != LOWSN_STATUS_SUCCESS) {
        //don't bother waiting for ACK, TX did not start correctly
       if (mac_pib.currentAckRetries) mac_pib.currentAckRetries--;
       if (mac_pib.currentAckRetries) {
           //TX did not start correctly, but still try again even if PHY failed
           mac_pib.flags.bits.ackPending = 0;
           macTxData();  //reuse the last packet.
       } else {
          mac_pib.flags.bits.ackPending = 0;
          macSetTxIdle();  //mark TX as idle
          macTxFSM_status = a_phy_service.status; //return status
        }	
      } else if (!mac_pib.flags.bits.ackPending) {
        //either no ACK requested or ACK has been received
        macSetTxIdle();  //finished successfully, mark as idle
        macTxFSM_status = LOWSN_STATUS_SUCCESS;
      }
      //check timeout
      else if (halMACTimerNowDelta(mac_pib.tx_start_time)> mac_pib.macAckWaitDuration)
      {
        // ACK timeout
        conPrintROMString("ACK Timeout! \n");
        if (mac_pib.currentAckRetries) mac_pib.currentAckRetries--;
        if (!mac_pib.currentAckRetries) {
          //retries are zero. We have failed.
          macTxFSM_status = LOWSN_STATUS_MAC_MAX_RETRIES_EXCEEDED;
          macSetTxIdle();
          DEBUG_STRING(1,"MAC TX Retry exceeded\n");
        } else {
          //retry...
          conPrintROMString("MAC Retry!! \n");
          macTxData();  //reuse the last packet.
        }
      }	

    }

  }

}

//format the packet and send it
//this is NOT used for either beacon or ack frames, only for data and MAC frames
//a side effect of this function is that if the source address mode is SHORT
//and our MAC short address is 0xFFFE, then the source address mode is forced to
//long as per the IEEE SPEC.

//this builds the header in reverse order since we adding header bytes to
//headers already added by APS, ADP layers.

//Add the MAC header, then send it to PHY
static void macTxData(void) {
  BYTE c;
  BYTE dstmode, srcmode;

  if (macTXIdle()) {
    //first time we are sending this packet, format the header
    //used static space for header. If need to store, will copy it later
    //format the header
				
    dstmode = LOWSN_GET_DST_ADDR(a_mac_tx_data.fcfmsb);
    srcmode = LOWSN_GET_SRC_ADDR(a_mac_tx_data.fcfmsb);

    if (mac_pib.macPANID == 0xFFFE && srcmode == LOWSN_ADDRMODE_SADDR) {
      //our short address is 0xFFFE, force srcmode to long address
      srcmode = LOWSN_ADDRMODE_LADDR;
      //clear src mode
      a_mac_tx_data.fcfmsb = a_mac_tx_data.fcfmsb & ~LOWSN_FCF_SRCMODE_MASK;
      //set to long address
      LOWSN_SET_SRC_ADDR(a_mac_tx_data.fcfmsb,LOWSN_ADDRMODE_LADDR);
    }

    //format src Address
    switch(srcmode){
    case LOWSN_ADDRMODE_NOADDR:
				  break;
    case LOWSN_ADDRMODE_SADDR:
				  phy_pib.currentTxFrm--;
                                  *phy_pib.currentTxFrm = (BYTE)(a_mac_tx_data.SrcAddr >> 8);
                                  phy_pib.currentTxFrm--;
                                  *phy_pib.currentTxFrm = (BYTE)a_mac_tx_data.SrcAddr;				
                                  phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;
                                  break;
    case LOWSN_ADDRMODE_LADDR:
				  //this has to be our own long address, get it
				  //halGetProcessorIEEEAddress(phy_pib.currentTxFrm-8);
				  	halGetProcessorIEEEAddress_MAC(phy_pib.currentTxFrm-8);
                                  phy_pib.currentTxFlen=phy_pib.currentTxFlen+8;
                                  phy_pib.currentTxFrm = phy_pib.currentTxFrm-8;
                                  break;
    default:
				  break;
    }

    //format src PANID
    if ( !LOWSN_GET_INTRAPAN(a_mac_tx_data.fcflsb) &&
        srcmode != LOWSN_ADDRMODE_NOADDR
          ) {
            phy_pib.currentTxFrm--;
            *phy_pib.currentTxFrm = (BYTE) (a_mac_tx_data.SrcPANID >> 8);
            phy_pib.currentTxFrm--;
            *phy_pib.currentTxFrm = (BYTE)a_mac_tx_data.SrcPANID;
            phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;
          }

    //format dst Address
    switch(dstmode){
    case LOWSN_ADDRMODE_NOADDR:
      break;
		  case LOWSN_ADDRMODE_SADDR:
                    phy_pib.currentTxFrm--;
                    *phy_pib.currentTxFrm = (BYTE)(a_mac_tx_data.DestAddr.saddr >> 8);
                    phy_pib.currentTxFrm--;
                    *phy_pib.currentTxFrm = (BYTE)a_mac_tx_data.DestAddr.saddr;
                    phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;
                    break;
		  case LOWSN_ADDRMODE_LADDR:
                    for(c=0;c<8;c++) {
                      phy_pib.currentTxFrm--;
                      *phy_pib.currentTxFrm = a_mac_tx_data.DestAddr.laddr.bytes[7-c];
                      //*phy_pib.currentTxFrm = a_mac_tx_data.DestAddr.laddr.bytes[c];
                    }
                    phy_pib.currentTxFlen=phy_pib.currentTxFlen+8;
                    break;
		  default:
                    break;
    }

    //format dst PANID, will be present if both dst is nonzero
    if (dstmode != LOWSN_ADDRMODE_NOADDR){
      phy_pib.currentTxFrm--;
      *phy_pib.currentTxFrm = (BYTE) (a_mac_tx_data.DestPANID >> 8);
      phy_pib.currentTxFrm--;
      *phy_pib.currentTxFrm = (BYTE)a_mac_tx_data.DestPANID;					
      phy_pib.currentTxFlen=phy_pib.currentTxFlen+2;
    }

    //format dsn
    mac_pib.macDSN = halGetRandomByte();
    phy_pib.currentTxFrm--;
    *phy_pib.currentTxFrm = mac_pib.macDSN; //set DSN		

    //format MSB Fcontrol
    phy_pib.currentTxFrm--;
    *phy_pib.currentTxFrm = a_mac_tx_data.fcfmsb;

    // for test CC1101, NOT using ACK
    //LOWSN_CLR_ACK_REQUEST(a_mac_tx_data.fcflsb);
    //format LSB Fcontrol
    phy_pib.currentTxFrm--;
    *phy_pib.currentTxFrm = a_mac_tx_data.fcflsb;		

    phy_pib.currentTxFlen = phy_pib.currentTxFlen + 3; //DSN, FCFLSB, FCFMSB


    // at this point, we will attempt a TX
    mac_pib.flags.bits.ackPending = 0;


    //now send the data, ignore the GTS and INDIRECT bits for now
    #if 0
    DEBUG_STRING(DBG_TX,"TX DSN: ");
    DEBUG_UINT8(DBG_TX,mac_pib.macDSN);
    DEBUG_STRING(DBG_TX,"\n");
    #endif

    macSetTxBusy();
    mac_pib.currentAckRetries = mac_pib.macMaxAckRetries;
    macTxFSM_status = LOWSN_STATUS_MAC_INPROGRESS;
  }

  a_phy_service.cmd = LOWSN_SVC_PHY_TX_DATA;

  phyDoService();
}


//might be able to simplify this later.

static void macRxFSM(void){
  MACPKT *pkt;
  BYTE cmd;
  BYTE *pt;
  UINT16 i;

macRxFSM_start:

  switch(macRxState)  {
  case MAC_RXSTATE_IDLE:
    if (macRxBuffEmpty()) break;   //no data, break
    //buffer not empty, start decode
#ifdef LOWSN_MANUAL_MAC_ACK
    if (phy_pib.flags.bits.ackIsSending == 1)  break;
    // ���ֶ�����ACKʱ����֤MAC���������ȴ���ACK����	
#endif
    pkt = macGetRxPacket();

    #ifdef LOWSN_COORDINATOR
    #ifdef LOWSN_SLIP_TO_HOST
    if (slip_pib.forward_kind == SLIP_FORWARD_MAC_PACKET)  {
		
        macRxState = MAC_RXSTATE_FORWARD_HOST;
        goto macRxFSM_start;
    }		
    #endif
    #endif
	
    //dbgPrintPacket(pkt->data+1, *(pkt->data));
    //must be either a DATA, BCN, or MAC packet
    //at this point, we throw away BCN packets if are not waiting
    //for a beacon response
    if ((LOWSN_IS_BCN(*(pkt->data+1))) &&
        !mac_pib.flags.bits.WaitingForBeaconResponse) {
          DEBUG_STRING(DBG_INFO,"MAC: Received BCN pkt, discarding.\n");
          macFreeRxPacket(TRUE);
          break;
        }
    if (LOWSN_IS_ACK(*(pkt->data+1))) {
      //This should not happen. ACK packets should be parsed
      //in the HAL layer that copies ACK packets to temp storage.
      //will keep this for completeness.
      DEBUG_STRING(DBG_INFO,"MAC: Received ACK pkt in macStartRxDecode, discarding, check ack packet parsing..\n");
      macFreeRxPacket(TRUE);
      break;
    }
    //at this point, we have a DATA, MAC CMD, or BCN packet.. do something with it.
    //need to parse the header information get to the payload.
    a_mac_rx_data.orgpkt = pkt;
    macParseHdr();
    if ((LOWSN_IS_BCN(*(pkt->data+1)))){
      DEBUG_STRING(DBG_INFO,"MAC: Parsing BCN pkt.\n");
      //now finished with it.
      macParseBeacon();
      macFreeRxPacket(TRUE);
      break;
    }

	#ifndef LOWSN_NO_CHECK_PHYMAC
		//this is a data packet, check if we should reject it
        if (!macCheckDataDestAddr()) {
            //we need to reject this packet.
            //DEBUG_STRING(DBG_INFO,"MAC: Rejecting Data packet from unassociated node, rejecting.\n");
            macFreeRxPacket(TRUE);
            break;
        }
	#endif

    if (LOWSN_IS_DATA(*(pkt->data+1))){

	#ifndef LOWSN_NO_CHECK_PHYMAC
		//this is a data packet, check if we should reject it
        if (!macCheckDataRejection()) {
            //we need to reject this packet.
            DEBUG_STRING(DBG_INFO,"MAC: Rejecting Data packet from unassociated node, rejecting.\n");
            macFreeRxPacket(TRUE);
            break;
        }
	#endif
	
       mac_pib.last_data_rx_time = halGetMACTimer();  //time of last data or mac command
      //at this point, will accept packet, indicate this to network layer
      //set a flag, and pass the adpStart offset to the ADP layer
      //RX buffer.
      
	// ��ӡ�յ������ݰ�
	// ���һλ��CRC����������ڶ�λ��RSSIֵ������ӡ
	#if 1
	DEBUG_STRING(DBG_INFO, " \n***** RX BUF *********\n");
	pt=a_mac_rx_data.orgpkt->data;
        for (i=0; i<(*(a_mac_rx_data.orgpkt->data) -1); i++) {
		DEBUG_UINT8(DBG_INFO, *pt);
		DEBUG_STRING(DBG_INFO, "   ");
		pt++;
		
    	}	
	DEBUG_STRING(DBG_INFO, " \n***** RX BUF *********\n");
	#endif

      macRxState = MAC_RXSTATE_ADP_HANDOFF;
      goto macRxFSM_start;
    }

    //at this point, we have a MAC command packet, lets do something with it.
    DEBUG_STRING(DBG_INFO,"MAC: Received MAC cmd packet, proceeding.\n");

    //there are some MAC CMDs that we can handle right here.
    //If it is a response, we handle it here. If it is a request,
    //that has to be handled in the main FSM.
    cmd = *(a_mac_rx_data.orgpkt->data + a_mac_rx_data.pload_offset);
    switch(cmd) {
    case LOWSN_MACCMD_ASSOC_RSP:
#ifndef LOWSN_COORDINATOR
      if (mac_pib.flags.bits.WaitingForAssocResponse){
        macParseAssocResponse();
      }					
#endif				
      //free this packet, we are finished with it.
      macFreeRxPacket(TRUE);
      break;

	case LOWSN_MACCMD_COORD_REALIGN:
#ifndef LOWSN_COORDINATOR
      if (mac_pib.flags.bits.WaitingForOrphanResponse){
        macParseOrphanResponse();
      }					
#endif				
      //free this packet, we are finished with it.
      macFreeRxPacket(TRUE);
      break;



#ifdef LOWSN_FFD
      //only FFDs handle this
    case LOWSN_MACCMD_BCN_REQ:
    case LOWSN_MACCMD_ASSOC_REQ:
	case LOWSN_MACCMD_ORPHAN:
      //requests must be handled in the main FSM. We need to signal that a MAC
      //CMD packet is packet is pending, and freeze the RX FSM until the
      //main FSM has taken care of it.
      mac_pib.flags.bits.macPending = 1;
      macRxState = MAC_RXSTATE_CMD_PENDING;
      break;
#endif

    default:
      //unhandled MAC packets
      DEBUG_STRING(DBG_INFO,"MAC: Received MAC CMD that is not implemented or not handled by this node, discarding.\n");
      DEBUG_STRING(DBG_INFO,"Cmd is: ");
      DEBUG_UINT8(DBG_INFO,cmd);
      DEBUG_STRING(DBG_INFO,"\n");
      macFreeRxPacket(TRUE);
      break;

    }			

    break;
  case MAC_RXSTATE_ADP_HANDOFF:
		  if (adpRxBusy()) break;    //adpRX is still busy
                  //handoff the current packet
                  adpRxHandoff();
                  //we are finished with this packet.
                  //free the MAC resource, but not the memory. The ADP layer
                  // or above has to free the memory
                  macFreeRxPacket(FALSE);
                  macRxState = MAC_RXSTATE_IDLE;
                  break;
  case MAC_RXSTATE_CMD_PENDING:
    if (mac_pib.flags.bits.macPending ) break;
			 //when macPending is cleared, this means main FSM is finished with packet.
			 //So free the packet, and start parsing new packets again
			 macFreeRxPacket(TRUE);
                      macRxState = MAC_RXSTATE_IDLE;
                      break;
					  
  #ifdef LOWSN_COORDINATOR
  #ifdef LOWSN_SLIP_TO_HOST
  case MAC_RXSTATE_FORWARD_HOST:
  	if (slipTxLocked())  break;

	DEBUG_STRING(DBG_INFO,"MAC: Forward MAC packet to the host.\n");
	slipGrabTxLock();
	slipSendMacFlags();
	slipSend(pkt->data, *(pkt->data), 1);
	slipReleaseTxLock();

	macFreeRxPacket(TRUE);
	macRxState = MAC_RXSTATE_IDLE;

	break;
  #endif
  #endif
	
  default: break;

  }

}

//parse the header currently in a_mac_rx_data
//return the offset to the network header.

static void macParseHdr() {
  BYTE *ptr;
  BYTE len,i;
  BYTE srcmode, dstmode;

  ptr = a_mac_rx_data.orgpkt->data;

  //skip first byte since the first byte in the a_mac_rx_data.orgpkt is the
  //packet length
  len = 1;ptr++;


  a_mac_rx_data.fcflsb = *ptr; ptr++;
  a_mac_rx_data.fcfmsb = *ptr; ptr++;
  dstmode = LOWSN_GET_DST_ADDR(a_mac_rx_data.fcfmsb);
  srcmode = LOWSN_GET_SRC_ADDR(a_mac_rx_data.fcfmsb);

  //skip DSN
  ptr++;
  len = len +3;

  if (dstmode != LOWSN_ADDRMODE_NOADDR){
    //get the DEST PANDID
    a_mac_rx_data.DestPANID = *ptr;
    ptr++;
    a_mac_rx_data.DestPANID += (((UINT16)*ptr) << 8);
    ptr++;
    len = len + 2;
  }
  //DST address
  if (dstmode == LOWSN_ADDRMODE_SADDR) {
    a_mac_rx_data.DestAddr.saddr = *ptr;
    ptr++;
    a_mac_rx_data.DestAddr.saddr += (((UINT16)*ptr) << 8);
    ptr++;
    len = len + 2;

  }else if (dstmode == LOWSN_ADDRMODE_LADDR) {
    for (i=0;i<8;i++) {
      a_mac_rx_data.DestAddr.laddr.bytes[i] = *ptr;
      ptr++;
    }
    len = len + 8;
  }


  if ( !LOWSN_GET_INTRAPAN(a_mac_rx_data.fcflsb) &&
      srcmode != LOWSN_ADDRMODE_NOADDR
        ) {
          //PANID present if INTRAPAN is zero, and src is nonzero
          a_mac_rx_data.SrcPANID = *ptr;
          ptr++;
          a_mac_rx_data.SrcPANID += (((UINT16)*ptr) << 8);
          ptr++;
          len = len + 2;
        }
  //SRC address
  if (srcmode == LOWSN_ADDRMODE_SADDR) {
    a_mac_rx_data.SrcAddr.saddr = *ptr;
    ptr++;
    a_mac_rx_data.SrcAddr.saddr += (((UINT16)*ptr) << 8);
    ptr++;
    len = len + 2;

  }else if (srcmode == LOWSN_ADDRMODE_LADDR) {
    for (i=0;i<8;i++) {
      a_mac_rx_data.SrcAddr.laddr.bytes[i] = *ptr;
      ptr++;
    }
    len = len + 8;
  }
  //save offset.
  a_mac_rx_data.pload_offset = len;
}

#ifdef LOWSN_FFD
//Beacon payload format
// adp magic number ( 4 bytes) | mac depth
static void macFormatBeacon(void){
  BYTE i;

  //fill in the beacon payload, we have the TX buffer lock
  phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];
#ifndef LOWSN_ZIGBEE_BEACON_COMPLY
  //fill in the magic number
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B3;
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B2;
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B1;
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B0;
#endif

  //next three bytes are zero for timestep difference
  //for multi-hop beaconing networks. This is currently filled in
  //as zero
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm =0;
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm =0;
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm =0;

  //see if I have space for an END device
  --phy_pib.currentTxFrm;
  if (mac_pib.ChildRFDs == LOWSN_MAX_NON_ROUTER_CHILDREN) {
	  *phy_pib.currentTxFrm =0; //no room
  } else {
      *phy_pib.currentTxFrm =1;  //have space.
  }

  //fill in my depth
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = mac_pib.depth;

  //see if I have space for a ROUTER device
  --phy_pib.currentTxFrm;
  if (mac_pib.ChildRouters == LOWSN_MAX_ROUTERS_PER_PARENT) {
	  *phy_pib.currentTxFrm =0; //no room
  } else {
      *phy_pib.currentTxFrm =1;  //have space.
  }

   //network protocol version
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm =LOWSN_ZIGBEE_PROTOCOL_VER;

   //stack protocol
   --phy_pib.currentTxFrm;
   *phy_pib.currentTxFrm =LOWSN_STACK_PROFILE;

   //Zigbee protocol ID
   --phy_pib.currentTxFrm;
   *phy_pib.currentTxFrm =LOWSN_ZIGBEE_PROTOCOL_ID;

  //pending address field
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = 0;  //makes this a NOP

  //GTS directions field
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = 0;  //makes this a NOP

  //2 bytes of superframe
#ifdef LOWSN_COORDINATOR
  i = LOWSN_BEACON_SF_PAN_COORD_MASK;
#else
  i = 0;
#endif
  if (mac_pib.flags.bits.macAssociationPermit) {
    i = i | LOWSN_BEACON_SF_ASSOC_PERMIT_MASK;
  }
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = i;  //MSB of superframe

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = 0xFF; //LSB of superframe

  phy_pib.currentTxFlen = LOWSN_ADP_BEACON_SIZE;

}

#endif

//parse the beacon
static void macParseBeacon(void){
  BYTE *ptr;
  BYTE depth;

  if ( (*(a_mac_rx_data.orgpkt->data)-a_mac_rx_data.pload_offset-PACKET_FOOTER_SIZE +1)
      != LOWSN_ADP_BEACON_SIZE  ) {
        return; //wrong length
      }

  ptr = a_mac_rx_data.orgpkt->data + a_mac_rx_data.pload_offset;
  //check association bit in MSB of superframe
  if (!LOWSN_GET_BEACON_SF_ASSOC_PERMIT(*(ptr+1))) {
    //cannot associate with this node. reject
    return;
  }
  // point at payload (skip four bytes of header)
  ptr = ptr + 4;
  //skip if any mismatches on protocol ID/Ver, stack profile, etc.
  //check protocol ID
  if (*ptr != LOWSN_ZIGBEE_PROTOCOL_ID) return;
  ptr++;
  //check stack profile
  if (*ptr != LOWSN_STACK_PROFILE) return;
  ptr++;
  //check protocol version
  if (*ptr != LOWSN_ZIGBEE_PROTOCOL_VER) return;
  ptr++;

  //check router capacity
  //for right now, if I am a router, I have to join as a router.
  //no option as of now for a router joining as an end-device
#ifdef LOWSN_FFD
  //only routers have to check this
  if (*ptr == 0) return;  //no room to join as router
#endif

  ptr++;

  //get the depth
  depth = *ptr;
  ptr++;

  //check end device capacity
#ifdef LOWSN_RFD
  //only end devices have to check this.
  if (*ptr == 0) return;  //no room to join as end device
#endif
  ptr++;

  //skip the next three bytes, only for beaconing.
  ptr = ptr + 3;

#ifndef LOWSN_ZIGBEE_BEACON_COMPLY
  //check the magic number
  if (*ptr != LOWSN_ADP_MAGICNUM_B0) return;
  ptr++;
  if (*ptr != LOWSN_ADP_MAGICNUM_B1) return;
  ptr++;
  if (*ptr != LOWSN_ADP_MAGICNUM_B2) return;
  ptr++;
  if (*ptr != LOWSN_ADP_MAGICNUM_B3) return;
  ptr++;
#endif

  //at this point, we could accept this node as a parent
  if ((mac_pib.bcnDepth == 0xFF) ||
      (a_mac_rx_data.orgpkt->rssi > mac_pib.bcnRSSI)) {
        //either our first response, or from a closer node
        //save this information.
        //use value with higher RSSI.
        //the RSSI byte is assumed to be formatted as HIGHER IS BETTER
        //the HAL layer converts any native signed RSSI to an unsigned value

        mac_pib.bcnDepth = depth;  //get depth
        mac_pib.bcnRSSI = a_mac_rx_data.orgpkt->rssi;
        mac_pib.bcnSADDR = a_mac_rx_data.SrcAddr.saddr;
        mac_pib.bcnPANID = a_mac_rx_data.SrcPANID;
        DEBUG_STRING(DBG_INFO,"MAC: Bcn rsp is Panid: ");
        DEBUG_UINT16(DBG_INFO, mac_pib.bcnPANID);
         DEBUG_STRING(DBG_INFO,", Saddr: ");
       DEBUG_UINT16(DBG_INFO, mac_pib.bcnSADDR);
        DEBUG_STRING(DBG_INFO,", LQI: ");
       DEBUG_UINT8(DBG_INFO, mac_pib.bcnRSSI);
          DEBUG_STRING(DBG_INFO,", Dpth: ");
       DEBUG_UINT8(DBG_INFO, mac_pib.bcnDepth);
       DEBUG_STRING(DBG_INFO,"\n");
      }
  mac_pib.flags.bits.GotBeaconResponse = 1;
}


static void macFormatAssocRequest(void){
  //fill in payload of request
  phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];

#ifndef IEEE_802_COMPLY
  //put the magic number in the association request
  phy_pib.currentTxFrm--;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B3;

  phy_pib.currentTxFrm--;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B2;

  phy_pib.currentTxFrm--;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B1;

  phy_pib.currentTxFrm--;
  *phy_pib.currentTxFrm = LOWSN_ADP_MAGICNUM_B0;
#endif

  phy_pib.currentTxFrm--;
  *phy_pib.currentTxFrm = mac_pib.macCapInfo;

  phy_pib.currentTxFrm--;
  *phy_pib.currentTxFrm = LOWSN_MACCMD_ASSOC_REQ;

  phy_pib.currentTxFlen = LOWSN_MACCMD_ASSOC_REQ_PAYLOAD_LEN;

}

#ifndef LOWSN_COORDINATOR
//parse the association response
static void macParseAssocResponse(void){
  BYTE *ptr;
  SADDR saddr;

  //first, ensure that the payload length is correct
  //the +1 is because the offset takes into account the lenght byte at the start of the packet
  if ( (*(a_mac_rx_data.orgpkt->data)-a_mac_rx_data.pload_offset-PACKET_FOOTER_SIZE +1)
      != LOWSN_MACCMD_ASSOC_RSP_PAYLOAD_LEN ) {
        DEBUG_STRING( DBG_INFO, "MAC: Failed to join, illegal assoc response\n");
        return; //wrong length
      }
  ptr = a_mac_rx_data.orgpkt->data + a_mac_rx_data.pload_offset;
  //check the status first which is last byte of the payload
  if (LOWSN_GET_ASSOC_STATUS(*(ptr+LOWSN_MACCMD_ASSOC_RSP_PAYLOAD_LEN-1)) != LOWSN_ASSOC_STATUS_SUCCESS) {
    //failed to join
    DEBUG_STRING( DBG_INFO, "MAC: Failed to join, remote node rejected us.\n");
    return;
  }
  ptr++; //skip command byte

  //successful join, get my short SADDR
  saddr = (BYTE) *ptr;
  ptr++;
  saddr += (((UINT16) *ptr) << 8);
  macSetShortAddr(saddr);
  ptr++;

  //our PANID is our parent's panid.
  mac_pib.macPANID = a_mac_rx_data.SrcPANID;
  halSetRadioPANID(a_mac_rx_data.SrcPANID);

#ifndef IEEE_802_COMPLY
  //the short address of the parent are extra bytes in this payload
  mac_pib.macCoordShortAddress = (BYTE) *ptr;
  ptr++;
  mac_pib.macCoordShortAddress += (((UINT16) *ptr) << 8);
  ptr++;
  //get the depth of parent, our depth is 1+ that of our parent
  mac_pib.depth = *ptr + 1;
#else

#ifndef LOWSN_FORCE_ASSOCIATION_TARGET
  //if we are not using forced association, then the beacon response
  //we got had the short address that we used for the association request,
  //so the short address of the beacon responder is our parent
  mac_pib.macCoordShortAddress = mac_pib.bcnSADDR;
  //beacon response also has the depth of our parent, so our depth is 1+ this
  mac_pib.depth = mac_pib.bcnDepth+1;
#endif


#endif

  //copy the SRC long address as my coordinator long address
  mac_pib.macCoordExtendedAddress.bytes[0] = a_mac_rx_data.SrcAddr.laddr.bytes[0];
  mac_pib.macCoordExtendedAddress.bytes[1] = a_mac_rx_data.SrcAddr.laddr.bytes[1];
  mac_pib.macCoordExtendedAddress.bytes[2] = a_mac_rx_data.SrcAddr.laddr.bytes[2];
  mac_pib.macCoordExtendedAddress.bytes[3] = a_mac_rx_data.SrcAddr.laddr.bytes[3];
  mac_pib.macCoordExtendedAddress.bytes[4] = a_mac_rx_data.SrcAddr.laddr.bytes[4];
  mac_pib.macCoordExtendedAddress.bytes[5] = a_mac_rx_data.SrcAddr.laddr.bytes[5];
  mac_pib.macCoordExtendedAddress.bytes[6] = a_mac_rx_data.SrcAddr.laddr.bytes[6];
  mac_pib.macCoordExtendedAddress.bytes[7] = a_mac_rx_data.SrcAddr.laddr.bytes[7];



  //indicate that the association was successful
  mac_pib.flags.bits.macIsAssociated = 1;
  mac_pib.flags.bits.ackPending = 0;  //only one outstanding association req, clear the ack bit
  DEBUG_STRING(DBG_INFO,"MAC:Received good association response!\n");

}
#endif


#ifndef  LOWSN_COORDINATOR
//Parse the coordinator realignment (Orphan response)
static void macParseOrphanResponse(void){
  BYTE *ptr;
  UINT16 tmp;


  //first, ensure that the payload length is correct
  //the +1 is because the offset takes into account the lenght byte at the start of the packet
  if ( (*(a_mac_rx_data.orgpkt->data)-a_mac_rx_data.pload_offset-PACKET_FOOTER_SIZE +1)
      != LOWSN_MACCMD_COORD_REALIGN_PAYLOAD_LEN ) {
        DEBUG_STRING( DBG_INFO, "MAC: illegal Coord Alignment packet\n");
        return; //wrong length
      }
  ptr = a_mac_rx_data.orgpkt->data + a_mac_rx_data.pload_offset;

  DEBUG_STRING(DBG_INFO, "Received Coord Realign (Orphan response)\n");
  mac_pib.flags.bits.GotOrphanResponse = 1;
  mac_pib.flags.bits.macIsAssociated = 1;  //we are associated with somebody!
   mac_pib.flags.bits.ackPending = 0;

  ptr++; //skip command byte

  //get the PANID
  tmp = (BYTE) *ptr;
  ptr++;
  tmp += (((UINT16) *ptr) << 8);
  ptr++;
  macSetPANID(tmp);

  //get the coordinator short address
  mac_pib.macCoordShortAddress = (BYTE) *ptr;
  ptr++;
  mac_pib.macCoordShortAddress += (((UINT16) *ptr) << 8);
  ptr++;

  tmp =(BYTE) *ptr; //get the channel
  ptr++;

  macSetChannel(tmp);  //set the channel

#ifndef LOWSN_COORDINATOR

  //copy the SRC long address as my coordinator long address
  mac_pib.macCoordExtendedAddress.bytes[0] = a_mac_rx_data.SrcAddr.laddr.bytes[0];
  mac_pib.macCoordExtendedAddress.bytes[1] = a_mac_rx_data.SrcAddr.laddr.bytes[1];
  mac_pib.macCoordExtendedAddress.bytes[2] = a_mac_rx_data.SrcAddr.laddr.bytes[2];
  mac_pib.macCoordExtendedAddress.bytes[3] = a_mac_rx_data.SrcAddr.laddr.bytes[3];
  mac_pib.macCoordExtendedAddress.bytes[4] = a_mac_rx_data.SrcAddr.laddr.bytes[4];
  mac_pib.macCoordExtendedAddress.bytes[5] = a_mac_rx_data.SrcAddr.laddr.bytes[5];
  mac_pib.macCoordExtendedAddress.bytes[6] = a_mac_rx_data.SrcAddr.laddr.bytes[6];
  mac_pib.macCoordExtendedAddress.bytes[7] = a_mac_rx_data.SrcAddr.laddr.bytes[7];


#endif

#ifdef LOWSN_RFD
  //get our short address
  tmp = (BYTE) *ptr;
  ptr++;
  tmp += (((UINT16) *ptr) << 8);
  ptr++;
  macSetShortAddr(tmp);
#else
  //this is a router
   //get our short ADDR

   tmp = (BYTE) *ptr;
   ptr++;
   tmp += (((UINT16) *ptr) << 8);
   ptr++;

   if (tmp != macGetShortAddr()) {
	   //our short address has changed!
	   //everything may have changed,
	   //clear neighbor table, and address map
	   ntInitAddressMap();
	   ntInitTable();
  }
  macSetShortAddr(tmp);
#endif

}

#endif



static void macFormatOrphanNotify(void){
phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
  *phy_pib.currentTxFrm = LOWSN_MACCMD_ORPHAN;
   phy_pib.currentTxFlen = 1;
}



#ifdef LOWSN_FFD

static void macFormatCoordRealign(SADDR orphan_saddr){
  //format and send the realignment packet
  //first is the orphans short address
  phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
  *phy_pib.currentTxFrm = (BYTE) (orphan_saddr >>8);

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (orphan_saddr);

  //logical channel
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = phy_pib.phyCurrentChannel;

   //our short addresss
 --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (macGetShortAddr()>>8);

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (macGetShortAddr());

  //our PANID

--phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (mac_pib.macPANID>>8);

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (mac_pib.macPANID);


  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = LOWSN_MACCMD_COORD_REALIGN;


  phy_pib.currentTxFlen = LOWSN_MACCMD_COORD_REALIGN_PAYLOAD_LEN;
}




static void macFormatAssociationResponse(void){
  NAYBORENTRY *ntptr;
  UINT16 new_saddr;
  BYTE tmp, capinfo;

  BYTE TTT;
  new_saddr = 0xFFFF;
  tmp = LOWSN_ASSOC_STATUS_DENIED;  //default status

  //check reasons to reject first
  //check payload length
  TTT = *(a_mac_rx_data.orgpkt->data)-a_mac_rx_data.pload_offset-PACKET_FOOTER_SIZE+1 ;
  if ( (*(a_mac_rx_data.orgpkt->data)-a_mac_rx_data.pload_offset-PACKET_FOOTER_SIZE+1 )
      != LOWSN_MACCMD_ASSOC_REQ_PAYLOAD_LEN) {
        //invalid payload length
        DEBUG_STRING(DBG_INFO,"MAC:Invalid association request, rejecting node!\n");
        goto macFormatAssociationResponse_dopkt;
      }

#ifndef IEEE_802_COMPLY
  {
    BYTE *ptr;
    //Check Magic Number
    ptr = a_mac_rx_data.orgpkt->data + a_mac_rx_data.pload_offset;
    if (*(ptr+2) != LOWSN_ADP_MAGICNUM_B0){
      goto macFormatAssociationResponse_dopkt;
    }
    if (*(ptr+3) != LOWSN_ADP_MAGICNUM_B1){
      goto macFormatAssociationResponse_dopkt;
    }
    if (*(ptr+4) != LOWSN_ADP_MAGICNUM_B2){
      goto macFormatAssociationResponse_dopkt;
    }
    if (*(ptr+5) != LOWSN_ADP_MAGICNUM_B3){
      goto macFormatAssociationResponse_dopkt;
    }
  }
#endif

  //now, see if this node is in the table
  ntptr = ntFindByLADDR(&a_mac_rx_data.SrcAddr.laddr);
  if (ntptr) {
	   new_saddr = mac_addr_tbl[ntptr->map_index].saddr;
           tmp = LOWSN_ASSOC_STATUS_SUCCESS;
           goto macFormatAssociationResponse_dopkt;
  }
  //node is not in table. Look at capability info byte and see if we
  //have room for this node type
  capinfo = *(a_mac_rx_data.orgpkt->data + a_mac_rx_data.pload_offset + 1);

  //node is not in table. Do final check with user
  if (!usrJoinVerifyCallback(&a_mac_rx_data.SrcAddr.laddr, capinfo)) {
    tmp = LOWSN_ASSOC_STATUS_DENIED;
    goto macFormatAssociationResponse_dopkt;
  }


  if ( ((LOWSN_GET_CAPINFO_DEVTYPE(capinfo)) && (mac_pib.ChildRouters == LOWSN_MAX_ROUTERS_PER_PARENT))
      ||
        (!(LOWSN_GET_CAPINFO_DEVTYPE(capinfo)) && (mac_pib.ChildRFDs == LOWSN_MAX_NON_ROUTER_CHILDREN)))
    //no room left
  {
    //no room
    tmp = LOWSN_ASSOC_STATUS_NOROOM;
    goto macFormatAssociationResponse_dopkt;

  }

  //not in table, Add this node
  new_saddr = ntAddNeighbor(&a_mac_rx_data.SrcAddr.laddr.bytes[0],capinfo);
  if (new_saddr == LOWSN_BCAST_SADDR) {
	  //this is an error indication, adding neighbor failed
      tmp = LOWSN_ASSOC_STATUS_NOROOM;
	  goto macFormatAssociationResponse_dopkt;
  }
  DEBUG_STRING(DBG_INFO,"MAC:Sending good association response!\n");
  tmp = LOWSN_ASSOC_STATUS_SUCCESS;
  usrJoinNotifyCallback(&a_mac_rx_data.SrcAddr.laddr);

macFormatAssociationResponse_dopkt:

   if (tmp ==  LOWSN_ASSOC_STATUS_SUCCESS) {
        DEBUG_STRING(DBG_INFO,"MAC:Sending good association response!\n");
   } else {
     DEBUG_STRING(DBG_INFO,"MAC:Rejecting assoc request: ");
     DEBUG_UINT8(DBG_INFO,tmp);
     DEBUG_STRING(DBG_INFO,"\n");
    }

  //format and send the packet
  //status byte
  phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
  *phy_pib.currentTxFrm = tmp;

#ifndef IEEE_802_COMPLY
   //put our depth, short address so that the RFD will know both
  //the radius, short and long addresses even if Beacon request has not been done
   --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = mac_pib.depth;

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (macGetShortAddr()>>8);

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (macGetShortAddr());
#endif
  //new short address for the RFD
  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (new_saddr>>8);

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = (BYTE) (new_saddr);

  //CMD

  --phy_pib.currentTxFrm;
  *phy_pib.currentTxFrm = LOWSN_MACCMD_ASSOC_RSP;

  phy_pib.currentTxFlen = LOWSN_MACCMD_ASSOC_RSP_PAYLOAD_LEN;

}

#endif


//check for DATA packets
//For RFD, just check if this packet came from parent
//For Routers, if this uses SHORT addressing, then check
//to see if this is associated with us

static BOOL macCheckDataRejection(void){

	BYTE AddrMode,i;
	
	//if not associated, reject
#ifndef LOWSN_COORDINATOR
	if (!mac_pib.flags.bits.macIsAssociated) {
		DEBUG_STRING(DBG_INFO, "MAC: Rejecting data pkt as we are not associated\n");
		return(FALSE);
	}
#endif
    AddrMode = LOWSN_GET_DST_ADDR(a_mac_rx_data.fcfmsb);

	if (AddrMode == LOWSN_ADDRMODE_LADDR) {
		//this packet send directly to our long address. accept it.
		return(TRUE);
	}

	/*if (AddrMode == LOWSN_ADDRMODE_LADDR) {
		//���Ŀ�ĳ���ַ�Ƿ�Ϊ�Լ��ĳ���ַ
		//check parent long address.
		if ((a_mac_rx_data.DestAddr.laddr.bytes[0] != aExtendedAddress_B7)
		||(a_mac_rx_data.DestAddr.laddr.bytes[1] != aExtendedAddress_B6)
		||(a_mac_rx_data.DestAddr.laddr.bytes[2] != aExtendedAddress_B5)
		||(a_mac_rx_data.DestAddr.laddr.bytes[3] != aExtendedAddress_B4)
		||(a_mac_rx_data.DestAddr.laddr.bytes[4] != aExtendedAddress_B3)
		||(a_mac_rx_data.DestAddr.laddr.bytes[5] != aExtendedAddress_B2)
		||(a_mac_rx_data.DestAddr.laddr.bytes[6] != aExtendedAddress_B1)
		||(a_mac_rx_data.DestAddr.laddr.bytes[7] != aExtendedAddress_B0))
		{
			DEBUG_STRING(DBG_INFO, "MAC: Rejecting data. Dest long addr is not me\n");
			return(FALSE);
		}
		else 
			return(TRUE);
	}
	else if(AddrMode == LOWSN_ADDRMODE_SADDR)
	{
			//check parent short address
			if ((a_mac_rx_data.DestAddr.saddr != macGetShortAddr())
				&&(a_mac_rx_data.DestAddr.saddr != LOWSN_BCAST_SADDR))	
				{
					DEBUG_STRING(DBG_INFO, "MAC: Rejecting data. Dest short addr is not me\n");
					return(FALSE);
				}
	}*/
	
	//check the parent
	AddrMode = LOWSN_GET_SRC_ADDR(a_mac_rx_data.fcfmsb);
	if (AddrMode == LOWSN_ADDRMODE_SADDR) {
		//check parent short address
		if (a_mac_rx_data.SrcAddr.saddr == mac_pib.macCoordShortAddress)  		
			return(TRUE);

	} else if (AddrMode == LOWSN_ADDRMODE_LADDR){
		//check parent long address.
		for (i=0;i<8;i++) {
			if (a_mac_rx_data.SrcAddr.laddr.bytes[i] !=
				mac_pib.macCoordExtendedAddress.bytes[i])
				break;
		}
		if (i==8) return(TRUE); //have a match
	}
#ifdef LOWSN_RFD
	DEBUG_STRING(DBG_INFO, "MAC: Rejecting data pkt from unassociated node\n");
	return(FALSE);
#else
	//ok, for FFDs, check the neighbor table
	if (AddrMode == LOWSN_ADDRMODE_SADDR){
		if (ntFindBySADDR (a_mac_rx_data.SrcAddr.saddr) !=(NAYBORENTRY *) NULL)
			return(TRUE);
	}else if (AddrMode == LOWSN_ADDRMODE_LADDR){
        if (ntFindByLADDR (&a_mac_rx_data.SrcAddr.laddr))
			return(TRUE);
	}
    DEBUG_STRING(DBG_INFO, "MAC: Rejecting data pkt from unassociated node\n");
	return(FALSE);
#endif
}

static BOOL macCheckDataDestAddr(void){

	BYTE AddrMode,i;
	
    AddrMode = LOWSN_GET_DST_ADDR(a_mac_rx_data.fcfmsb);

	if (AddrMode == LOWSN_ADDRMODE_LADDR) {
		//���Ŀ�ĳ���ַ�Ƿ�Ϊ�Լ��ĳ���ַ
		//check parent long address.
		if ((a_mac_rx_data.DestAddr.laddr.bytes[0] != aExtendedAddress_B7)
		||(a_mac_rx_data.DestAddr.laddr.bytes[1] != aExtendedAddress_B6)
		||(a_mac_rx_data.DestAddr.laddr.bytes[2] != aExtendedAddress_B5)
		||(a_mac_rx_data.DestAddr.laddr.bytes[3] != aExtendedAddress_B4)
		||(a_mac_rx_data.DestAddr.laddr.bytes[4] != aExtendedAddress_B3)
		||(a_mac_rx_data.DestAddr.laddr.bytes[5] != aExtendedAddress_B2)
		||(a_mac_rx_data.DestAddr.laddr.bytes[6] != aExtendedAddress_B1)
		||(a_mac_rx_data.DestAddr.laddr.bytes[7] != aExtendedAddress_B0))
		{
			DEBUG_STRING(DBG_INFO, "MAC: Rejecting data. Dest long addr is not me\n");
			return(FALSE);
		}
		else 
			return(TRUE);
	}
	else if(AddrMode == LOWSN_ADDRMODE_SADDR)
	{
			//check parent short address
			if ((a_mac_rx_data.DestAddr.saddr != macGetShortAddr())
				&&(a_mac_rx_data.DestAddr.saddr != LOWSN_BCAST_SADDR))	
				{
					DEBUG_STRING(DBG_INFO, "MAC: Rejecting data. Dest short addr is not me\n");
					return(FALSE);
				}
			else
				return(TRUE);
	}
}

