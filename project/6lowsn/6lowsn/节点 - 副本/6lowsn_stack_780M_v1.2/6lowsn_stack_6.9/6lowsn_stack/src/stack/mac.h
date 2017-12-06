

#ifndef MAC_H
#define MAC_H

#define aBaseSlotDuration 60
#define aNumSuperFrameSlots 16
#define aBaseSuperFrameDuration (aBaseSlotDuration*aNumSuperFrameSlots)
#define aMaxBE 5
#define aMinBE 0
#define aUnitBackoffPeriod 20        //in symbols
#define macMaxCSMABackoffs 4
#define aMaxBeaconOverhead 75
#define aMaxBeaconPayloadLength (aMaxPHYPacketSize-aMaxBeaconOverhead)
#define aMaxFrameOverhead 25
#define aMaxFrameResponseTime 1220
#define aMaxFrameRetries LOWSN_MAC_MAX_FRAME_RETRIES
#define aMaxLostBeacons 4
#define aMaxMACFrameSize (aMaxPHYPacketSize-aMaxFrameOverhead)
#define aResponseWaitTime (32*aBaseSuperFrameDuration)

//default timeout on network responses
#ifdef LOWSN_DEBUG
//give longer due to debugging output
#define MAC_GENERIC_WAIT_TIME      MSECS_TO_MACTICKS(200)
#define MAC_ASSOC_WAIT_TIME        MAC_GENERIC_WAIT_TIME
#define MAC_ORPHAN_WAIT_TIME       MAC_GENERIC_WAIT_TIME
#else
#define MAC_GENERIC_WAIT_TIME      MSECS_TO_MACTICKS(200)
#define MAC_ASSOC_WAIT_TIME        MAC_GENERIC_WAIT_TIME
#define MAC_ORPHAN_WAIT_TIME       MAC_GENERIC_WAIT_TIME
#endif



#define MAC_RXBUFF_SIZE LOWSN_MAX_MAC_RX_PKTS+1

typedef struct _MAC_PIB {
	UINT32 macAckWaitDuration;
	union _MAC_PIB_flags {
		UINT32 val;
		struct {
			unsigned macAssociationPermit:1;
			unsigned macAutoRequest:1;
			unsigned macBattLifeExt:1;
			unsigned macGTSPermit:1;
			unsigned macPromiscousMode:1;
			unsigned macPanCoordinator:1;
			unsigned ackPending:1;
			unsigned TxInProgress:1;   //MAC TX FSM state
			unsigned GotBeaconResponse:1;      //set to a '1' when get Beacon Response
			unsigned WaitingForBeaconResponse:1; //set to '1' when waiting for Response
			unsigned macPending:1;       //mac CMD pending in the RX buffer
			unsigned macIsAssociated:1;
			unsigned WaitingForAssocResponse:1;
			unsigned GotOrphanResponse:1;
			unsigned WaitingForOrphanResponse:1;
			

		}bits;
	}flags;
	LADDR macCoordExtendedAddress;
	SADDR macCoordShortAddress;
	UINT16 macPANID;
	BYTE macDSN;
	BYTE depth;            //depth in the network
	BYTE macCapInfo;
    BYTE macMaxAckRetries;
	struct  {
		unsigned maxMaxCSMABackoffs:3;
		unsigned macMinBE:2;
	}misc;
	UINT32 tx_start_time;    //time that packet was sent
      UINT32 last_data_rx_time;    //time that last data rx packet was received that was accepted by this node

    BYTE bcnDepth;
	SADDR bcnSADDR;
	UINT16 bcnPANID;
	BYTE bcnRSSI;

	BYTE currentAckRetries;
	BYTE rxTail;             //tail pointer of rxBuff
    BYTE rxHead;             //head pointer of rxBuff
	//fifo for RX pkts, holds LOWSN_MAX_MAC_RX_PKTS
	MACPKT  rxBuff[MAC_RXBUFF_SIZE];  //buffer for packets not yet processed

#ifdef LOWSN_FFD
	//neighbor info
	UINT16 nextChildRFD;
	UINT16 nextChildRouter;
	BYTE   ChildRFDs;         //number of neighbor RFDs
	BYTE   ChildRouters;      //number of neighbor Routers
#endif


}MAC_PIB;


//used for parsing of RX data
typedef struct _MAC_RX_DATA {
	MACPKT *orgpkt;       //original packet
	BYTE fcflsb;
	BYTE fcfmsb;
	UINT16 DestPANID;
	LADDR_UNION DestAddr; //dst address, either short or long
	UINT16 SrcPANID;
	LADDR_UNION SrcAddr;  //src address, either short or long
	BYTE pload_offset;    //start of payload
}MAC_RX_DATA;

typedef struct _MAX_TX_DATA {
	UINT16 DestPANID;
	LADDR_UNION DestAddr; //dst address, either short or long
	UINT16 SrcPANID;
	SADDR SrcAddr;         //src address, either short or long, this holds short address version
	                       //if long is needed, then get this from HAL layer
	BYTE fcflsb;          //frame control bits specify header bits
	BYTE fcfmsb;
	union  {
		BYTE val;
		struct _MAC_TX_DATA_OPTIONS_bits {
			unsigned gts:1;
			unsigned indirect:1;
		}bits;
	}options;		
}MAC_TX_DATA;

typedef union _MAC_ARGS {
	struct {
		BYTE   LogicalChannel;
       //addressing, capinfo, security comes from mac_pib data.
	}assoc_req;
	struct {
		BYTE LogicalChannel;
	}beacon_req;
	struct {
       LOWSN_STATUS_ENUM status;
	}error;
	struct {
		SADDR saddr;
	}ping_node;
}MAC_ARGS;

typedef enum _MAC_STATE_ENUM {
  MAC_STATE_IDLE,
  MAC_STATE_COMMAND_START,
  MAC_STATE_GENERIC_TX_WAIT,
  MAC_STATE_GENERIC_TX_WAIT_AND_UNLOCK,
  MAC_STATE_HANDLE_ORPHAN_NOTIFY,
  MAC_STATE_ORPHAN_WAIT1,
  MAC_STATE_ORPHAN_WAIT2,
  MAC_STATE_ASSOC_REQ_WAIT1,
  MAC_STATE_ASSOC_REQ_WAIT2,
  MAC_STATE_SEND_BEACON_RESPONSE,
  MAC_STATE_SEND_ASSOC_RESPONSE
 } MAC_STATE_ENUM;

typedef struct _MAC_SERVICE {
  LOWSN_SVC_ENUM cmd;
  MAC_ARGS args;
  LOWSN_STATUS_ENUM status;
}MAC_SERVICE;


extern MAC_PIB mac_pib;
extern MAC_SERVICE a_mac_service;
extern MAC_STATE_ENUM macState;
extern MAC_TX_DATA a_mac_tx_data;
extern MAC_RX_DATA a_mac_rx_data;


void macInit(void);
void macFSM(void);

LOWSN_STATUS_ENUM macInitRadio(void);
LOWSN_STATUS_ENUM macWarmStartRadio(void);
void macSetPANID(UINT16 panid);
UINT16 macGetPANID(void);
void macSetChannel(BYTE channel);
void macSetShortAddr(UINT16 saddr);
//SADDR macGetShortAddr();
#define macGetShortAddr()   (mac_addr_tbl[0].saddr)
#define macGetPanID()   (mac_pib.macPANID)

BOOL macRxBuffEmpty(void);
BOOL macRxBuffFull(void);
MACPKT *macGetRxPacket(void);
void macFreeRxPacket(BOOL freemem);

#ifdef LOWSN_FFD
BOOL usrJoinVerifyCallback(LADDR *ptr, BYTE capinfo);
BOOL usrJoinNotifyCallback(LADDR *ptr);
#endif


#define macIdle() (macState == MAC_STATE_IDLE)
#define macBusy() (macState != MAC_STATE_IDLE)

#define macTXIdle() (!mac_pib.flags.bits.TxInProgress)
#define macTXBusy() (mac_pib.flags.bits.TxInProgress)
#define macSetTxBusy() mac_pib.flags.bits.TxInProgress = 1
#define macSetTxIdle() mac_pib.flags.bits.TxInProgress = 0

#define macDoService() \
	a_mac_service.status = LOWSN_STATUS_MAC_INPROGRESS;\
	macState = MAC_STATE_COMMAND_START;\
	macFSM();\


#endif

