

#ifndef SLIP_H
#define SLIP_H

#include "nwk.h"

#define MAX_SLIP_BUF_SIZE   100

#define END             192    /* indicates end of packet */
#define ESC             219    /* indicates byte stuffing */
#define ESC_END         220    /* ESC ESC_END means END data byte */
#define ESC_ESC         221    /* ESC ESC_ESC means ESC data byte */

#define SLIP_CMD_QUERY_FIRST  '?'
#define SLIP_CMD_INFO_FIRST    '!'
#define SLIP_CMD_PREFIX_REQ    'P'
#define SLIP_CMD_MAC_REQ    'M'


#define SLIP_APS_FIRST_FLAG        0xA1
#define SLIP_APS_SECOND_FLAG     0xA2

#define SLIP_MAC_FIRST_FLAG       0xB7
#define SLIP_MAC_SECOND_FLAG    0xB8


//#define SLIP_IP_FIRST_FLAG           0x3
//#define SLIP_IP_SECOND_FLAG        0x4

#define SLIP_APS_COORD_TO_HOST  0
#define SLIP_APS_HOST_TO_COORD  0x1

#define SLIP_APS_MP_FRAME     0x1
#define SLIP_APS_DATA_FRAME  0x2

#define SLIP_APS_TO_HOST_HEADER_LEN  33  




typedef enum _SLIP_FORWARD_ENUM {
	SLIP_FORWARD_NONE, 
	SLIP_FORWARD_MAC_PACKET,
	SLIP_FORWARD_IP_PACKET
} SLIP_FORWARD_ENUM;

typedef enum _SLIP_STATE_ENUM {
  SLIP_STATE_IDLE,
  SLIP_STATE_INJECT_NWK_TX_WAIT,
  SLIP_STATE_INJECT_NWK_RX_WAIT, 
  SLIP_STATE_INJECT_APS_DATA_WAIT, 
  SLIP_STATE_INJECT_APS_TX
} SLIP_STATE_ENUM;

typedef struct _SLIP_PIB {
  BYTE speed;
  union _SLIP_DATA_flags {
    BYTE val;
    struct {
     	unsigned txFinished:1;   
	unsigned txLock:1; 
    }bits;
  }flags;
  SLIP_FORWARD_ENUM forward_kind; 
  BYTE aps_forward;
}SLIP_PIB;

typedef struct _SLIP_SERVICE {
  LOWSN_SVC_ENUM cmd;
  MAC_ARGS args;
  LOWSN_STATUS_ENUM status;
}SLIP_SERVICE;


extern BYTE slipbuf[MAX_SLIP_BUF_SIZE];
extern SLIP_PIB slip_pib;
extern SLIP_STATE_ENUM slipState;
extern SLIP_SERVICE a_slip_service;

#define slipTxLocked()   (slip_pib.flags.bits.txLock == 1)
#define slipTxUnLocked()   (slip_pib.flags.bits.txLock == 0)

#define slipGrabTxLock()	slip_pib.flags.bits.txLock = 1
#define slipReleaseTxLock() slip_pib.flags.bits.txLock = 0


#define slipReady()  halGetchRdy()
#define slipIsCmdPkt()  (slipbuf[0] == SLIP_CMD_QUERY_FIRST || slipbuf[0] == SLIP_CMD_INFO_FIRST )
#define slipIsIpPkt6()  (NWK_IS_IP6(slipbuf[0]))
#define slipIsApsPkt()  ((slipbuf[0] == SLIP_APS_FIRST_FLAG) && (slipbuf[1] == SLIP_APS_SECOND_FLAG))
#define slipIsMacPkt()  ((slipbuf[0] == SLIP_MAC_FIRST_FLAG) && (slipbuf[1] == SLIP_MAC_SECOND_FLAG))

#define slipSendMacFlags()  do{ halRawPut(END); halRawPut(SLIP_MAC_FIRST_FLAG); halRawPut(SLIP_MAC_SECOND_FLAG);} while (0)
//#define slipSendIPFlags()  do{ halRawPut(END); halRawPut(SLIP_IP_FIRST_FLAG); halRawPut(SLIP_IP_SECOND_FLAG);} while (0)
#define slipSendIPFlags()  do{ halRawPut(END); } while (0)
#define slipSendApsFlags()  do{ halRawPut(END); halRawPut(SLIP_APS_FIRST_FLAG); halRawPut(SLIP_APS_SECOND_FLAG); } while (0)

void slipInit(void);
void slipSend(BYTE *pbuf, UINT16 len, UINT8 flag);
UINT16 slipRcv(void);
void slipRequestPrefix(void);
BOOL slipParsePrefix(IPADDR *prefix, UINT8 *p_len);
void slipParseCmd(UINT16 len);
void slipFlushBuf(UINT16 len);
void slipFSM(void);
	

#endif




