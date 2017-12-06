


#include "compiler.h"
#include "6lowsn_config.h"         //user configurations
#include "6lowsn_common_types.h"   //types common acrosss most files
#include "ieee_lrwpan_defs.h"
#include "hal.h"
#include "halStack.h"

#include "console.h"
#include "debug.h"
#include "memalloc.h"
#include "phy.h"


#include "evboard.h"



PHY_PIB phy_pib;
PHY_SERVICE a_phy_service;
PHY_STATE_ENUM phyState;

//static tmp space for that is used by NET, APS, MAC layers
//since only one TX can be in progress at a time, there will be
//not contention for this.
//The current frame is built up in this space, in reverse transmit order.
BYTE tmpTxBuff[LOWSN_MAX_FRAME_SIZE];

void phyInit(void ) {
  MemInit();  //initialize memory
  phyState = PHY_STATE_IDLE;
  phy_pib.flags.val = 0;
}


//call back from HAL to here, can be empty functions
//not needed in this stack
void phyRxCallback(void) {
}

void phyTxStartCallBack(void) {
phy_pib.txStartTime = halGetMACTimer();

}

void phyTxEndCallBack(void) {
phy_pib.flags.bits.txFinished = 1;   //TX is finished.
}



void phyFSM(void) {

	UINT16 i;	
	BYTE *pt;

	#ifdef LOWSN_MANUAL_MAC_ACK
	BYTE send_ack_bytes[3];
	#endif
	
  //do evbpolling here
  //evbPoll();

  //check background tasks here
	
phyFSM_start:
  
  switch (phyState) {
  case PHY_STATE_IDLE:
	  halIdle();  //Hal Layer might want to do something in idle state
        #ifdef LOWSN_MANUAL_MAC_ACK
	  if (phy_pib.flags.bits.needACK == 1) {
	  	
		phy_pib.flags.bits.needACK = 0;
		phyState = PHY_STATE_SEND_ACK;
		goto phyFSM_start;
	    }
	   #endif
    break;
#ifdef LOWSN_MANUAL_MAC_ACK
	
     case PHY_STATE_SEND_ACK:
	 	
       phy_pib.flags.bits.txFinished = 0;

	// form ACK frame
	send_ack_bytes[0] = LOWSN_FRAME_TYPE_ACK;
	send_ack_bytes[1] = 0;
	send_ack_bytes[2] = phy_pib.rcvSeqforACK;
	
	//conPrintROMString("EA\n");
        a_phy_service.status = halSendPacket(3, &send_ack_bytes[0]);
        if (a_phy_service.status == LOWSN_STATUS_SUCCESS) {
          //TX started, wait for it to end.
          phyState = PHY_STATE_TX_ACK_WAIT;
		  
        }else {
        
          DEBUG_STRING(1,"PHY: ACK TX did not start\n");
           phy_pib.flags.bits.ackIsSending = 0;
          phyState = PHY_STATE_IDLE;
        }
       break;

      case PHY_STATE_TX_ACK_WAIT: 
        if (phy_pib.flags.bits.txFinished){
            DEBUG_STRING(1,"PHY: ACK SEND OK! \n");
            phy_pib.flags.bits.ackIsSending = 0; 
            phyState = PHY_STATE_IDLE;
	
        }
        else if  (halMACTimerNowDelta(phy_pib.txStartTime) > MAX_ACK_TX_TRANSMIT_TIME){
          //should not happen, indicate an error to console
          DEBUG_STRING(1,"PHY: MAX_ACK_TX_TRANSMIT_TIME timeout\n");
	    a_phy_service.status = LOWSN_STATUS_PHY_TX_FINISH_FAILED;
          //no action for now, will see if this happens
          phy_pib.flags.bits.ackIsSending = 0; 
          phyState = PHY_STATE_IDLE;
        }
      break;

#endif
  case PHY_STATE_COMMAND_START:
    switch(a_phy_service.cmd) {
      case LOWSN_SVC_PHY_INIT_RADIO: //not split phase
       a_phy_service.status = halInitRadio(phy_pib.phyCurrentFrequency,
                                                  phy_pib.phyCurrentChannel,
                                                  a_phy_service.args.phy_init_radio_args.radio_flags
                                                    );
	   phyState = PHY_STATE_IDLE;
       break;
      case LOWSN_SVC_PHY_TX_DATA:
        phy_pib.flags.bits.txFinished = 0;

	// 打印发出的数据包
	#if 1
	DEBUG_STRING(DBG_INFO, " \n***** TX BUF *********\n");
	pt=phy_pib.currentTxFrm;
	for (i=0; i<phy_pib.currentTxFlen;i++) {
		DEBUG_UINT8(DBG_INFO, *pt);
		DEBUG_STRING(DBG_INFO, "  ");
		pt++;
    	}	
	DEBUG_STRING(DBG_INFO, " \n***** TX BUF *********\n");
	#endif
		
        a_phy_service.status =
           halSendPacket(phy_pib.currentTxFlen,
                         phy_pib.currentTxFrm);
        if (a_phy_service.status == LOWSN_STATUS_SUCCESS) {
          //TX started, wait for it to end.
          phyState = PHY_STATE_TX_WAIT;
        }else {
          //something failed, will give up on this, MAC can take action if it wants
          //should not happen, indicate an error to console
          DEBUG_STRING(1,"PHY: TX did not start\n");
          phyState = PHY_STATE_IDLE;
        }
       break;
     default: break;
    }//end switch cmd
    break;
  case PHY_STATE_TX_WAIT:  //wait for TX out of radio to complete or timeout
    if (phy_pib.flags.bits.txFinished){
        phyState = PHY_STATE_IDLE;
     }
    else if  (halMACTimerNowDelta(phy_pib.txStartTime) > MAX_TX_TRANSMIT_TIME){
      //should not happen, indicate an error to console
      DEBUG_STRING(1,"PHY: MAX_TX_TRANSMIT_TIME timeout\n");
	  a_phy_service.status = LOWSN_STATUS_PHY_TX_FINISH_FAILED;
      //no action for now, will see if this happens

			
      
      phyState = PHY_STATE_IDLE;
    }
    break;
  default: break;
  }//end switch phyState
}

