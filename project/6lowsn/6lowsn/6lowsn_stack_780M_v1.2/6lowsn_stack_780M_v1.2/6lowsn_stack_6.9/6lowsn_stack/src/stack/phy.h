
#ifndef PHY_H
#define PHY_H

#include "compiler.h"
#include "halStack.h"


#define aMaxPHYPacketSize 127
#define aTurnaroundTime 12

#define MAX_TX_TRANSMIT_TIME (SYMBOLS_TO_MACTICKS(300))  //a little long..

#ifdef LOWSN_MANUAL_MAC_ACK
#define MAX_ACK_TX_TRANSMIT_TIME (SYMBOLS_TO_MACTICKS(300))  //a little long..
#endif
typedef enum _RADIO_STATUS_ENUM {
  RADIO_STATUS_OFF,				
  RADIO_STATUS_RX_ON,				
  RADIO_STATUS_TX_ON,
  RADIO_STATUS_RXTX_ON
}RADIO_STATUS_ENUM;

typedef struct _PHY_PIB {
  PHY_FREQ_ENUM phyCurrentFrequency;        //current frequency in KHz (2405000 = 2.405 GHz)
  BYTE phyCurrentChannel;
  UINT32 phyChannelsSupported;
  BYTE phyTransmitPower;
  BYTE phyCCAMode;
  union _PHY_DATA_flags {
    BYTE val;
    struct {
     unsigned txFinished:1;    // indicates if TX at PHY level is finished...
	 unsigned txBuffLock:1;    // lock the TX buffer.
     #ifdef LOWSN_MANUAL_MAC_ACK
	unsigned needACK:1;  // 是否需要发送ACK
	unsigned ackIsSending:1;  // 是否需要发送NACK
	#endif
    }bits;
  }flags;
  UINT32 txStartTime;
  #ifdef LOWSN_MANUAL_MAC_ACK
  BYTE rcvSeqforACK;  //收到的帧的序列号，用于手动构造ACK帧
  #endif
  BYTE *currentTxFrm;   //current frame
  BYTE currentTxFlen;   //current TX frame length
}PHY_PIB;


typedef union _PHY_ARGS {
  struct _PHY_INIT_RADIO {
    RADIO_FLAGS radio_flags;
  }phy_init_radio_args;
}PHY_ARGS;



typedef struct _PHY_SERVICE {
  LOWSN_SVC_ENUM cmd;
  PHY_ARGS args;
  LOWSN_STATUS_ENUM status;
}PHY_SERVICE;

typedef enum _PHY_STATE_ENUM {
  PHY_STATE_IDLE,
  PHY_STATE_COMMAND_START,
  #ifdef LOWSN_MANUAL_MAC_ACK
  PHY_STATE_SEND_ACK, 
  PHY_STATE_TX_ACK_WAIT, 
  #endif
  PHY_STATE_TX_WAIT
 } PHY_STATE_ENUM;

extern PHY_STATE_ENUM phyState;
extern PHY_PIB phy_pib;
extern PHY_SERVICE a_phy_service;
extern BYTE tmpTxBuff[LOWSN_MAX_FRAME_SIZE];

//prototypes
void phyFSM(void);
//void phyDoService(PHY_SERVICE *ps);
void phyInit(void );

#define phyIdle() (phyState == PHY_STATE_IDLE)
#define phyBusy() (phyState != PHY_STATE_IDLE)

#define phyTxLocked()   (phy_pib.flags.bits.txBuffLock == 1)
#define phyTxUnLocked()   (phy_pib.flags.bits.txBuffLock == 0)

#define phyGrabTxLock()	phy_pib.flags.bits.txBuffLock = 1
#define phyReleaseTxLock() phy_pib.flags.bits.txBuffLock = 0







//cannot overlap services
//make this a macro to reduce stack depth
#define phyDoService() \
  a_phy_service.status = LOWSN_STATUS_PHY_INPROGRESS;\
  phyState = PHY_STATE_COMMAND_START;\
  phyFSM();


#endif


