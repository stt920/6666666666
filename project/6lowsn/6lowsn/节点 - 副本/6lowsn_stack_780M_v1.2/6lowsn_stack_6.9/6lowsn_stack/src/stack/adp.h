

#ifndef ADP_H
#define ADP_H

#include "compiler.h"
#include "ds.h"

// 待以后增加MESH 路由命令帧后再启用
//#define ADP_FRM_TYPE_DATA  0
//#define ADP_FRM_TYPE_CMD   1
//#define ADP_FRM_TYPE_MASK  3
//#define ADP_IS_DATA(x) ((x & ADP_FRM_TYPE_MASK)==ADP_FRM_TYPE_DATA)
//#define ADP_IS_CMD(x) ((x & ADP_FRM_TYPE_MASK)==ADP_FRM_TYPE_CMD)

#define ADP_IS_MESH_TYPE(x) ((x & 0xC0)==ADP_MESH_TYPE_BASE)



#define ADP_SUPPRESS_ROUTE_DISCOVER  (0<<6)
#define ADP_ENABLE_ROUTE_DISCOVER    (1<<6)
#define ADP_FORCE_ROUTE_DISCOVER     (2<<6)

#define ADP_ROUTE_MASK         (3 << 6)
#define ADP_GET_ROUTE(x)       (x & ADP_ROUTE_MASK)


//this value of zero means that our packets will not
//be recognized as Zibee SPEC packets which is what we
//want.  Do not want these packets confused with packets
//from Zigbee compliant devices.
#define ADP_PROTOCOL           (0<<2)
#define ADP_PROTOCOL_MASK      (15 << 2)
#define ADP_GET_PROTOCOL(x)    ((x & ADP_PROTOCOL_MASK) >> 2)



#define ADP_SECURITY_MASK      (1 << 1)


#define ADP_GENERIC_RETRIES  3   //number of retries for ADP operations


#define ADP_RXBUFF_SIZE LOWSN_MAX_ADP_RX_PKTS+1



#define ADP_RX_IP_COMPRESS      1
#define ADP_RX_IP_UNCOMPRESS  0

#define ADP_TX_ALWAYS_UNCOMPRESS      1
#define ADP_RX_AUTO_COMPRESS              0




#define ADP_DISPATCH_IPV6 0x41
#define ADP_DISPATCH_HC1  0x42
#define ADP_DISPATCH_BC0  0x50
#define ADP_DISPATCH_ESC  0x7F
#define ADP_MESH_TYPE_BASE  0x80

#define ADP_PREFIX_PI  0
#define ADP_PREFIX_PC  1

#define ADP_IID_II  0
#define ADP_IID_IC  1

#define ADP_TRAFFIC_FLOW_INLINE  0
#define ADP_TRAFFIC_FLOW_ZERO     1

#define ADP_NEXT_HEADER_INLINE    0x00
#define ADP_NEXT_HEADER_UDP         0x01
#define ADP_NEXT_HEADER_ICMP        0x02
#define ADP_NEXT_HEADER_TCP          0x03

#define ADP_HC2_ENCODING_NONE          0
#define ADP_HC2_ENCODING_ENABLE       1


#define ADP_GET_SRC_PREFIX_TYPE(x)       ((x>>7)&0x1)
#define ADP_GET_SRC_IID_TYPE(x)             ((x>>6)&0x1)
#define ADP_GET_DST_PREFIX_TYPE(x)       ((x>>5)&0x1)
#define ADP_GET_DST_IID_TYPE(x)              ((x>>4)&0x1)
#define ADP_GET_TRAFFIC_FLOW_TYPE(x)  ((x>>3)&0x1)
#define ADP_GET_NEXT_HEADER(x)              ((x>>1)&0x3)
#define ADP_GET_HC2_ENCODING(x)             ((x)&0x1)


#define ADP_SET_SRC_PC(x)  BITSET(x,7)
#define ADP_SET_SRC_IC(x)  BITSET(x,6)
#define ADP_SET_DST_PC(x)  BITSET(x,5)
#define ADP_SET_DST_IC(x)  BITSET(x,4)
#define ADP_SET_TRAFFIC_FLOW_COMP(x)  BITSET(x,3)
#define ADP_SET_NEXT_HEADER(x,f) (x=x|(f<<1))
#define ADP_SET_HC2_ENCODING(x)  BITSET(x,0)

#define ADP_SET_MESH_ORIGINAL_SADDR(x)  BITSET(x,5)
#define ADP_SET_MESH_ORIGINAL_LADDR(x)  BITCLR(x,5)
#define ADP_SET_MESH_FINAL_SADDR(x)  BITSET(x,4)
#define ADP_SET_MESH_FINAL_LADDR(x)  BITCLR(x,4)
#define ADP_SET_MESH_HOPLFT(x, f)   (x=x|(f&0x0F))

#define ADP_GET_MESH_ORIGINAL_ADDRTYPE(x)  ((x>>5)&0x1)
#define ADP_SET_MESH_FINAL_ADDRTYPE(x)  ((x>>4)&0x1)
#define ADP_GET_MESH_HOPLFT(x)          (x&0x0F)

#define ADP_MESH_SADDR        1
#define ADP_MESH_LADDR        0


#ifdef  LOWSN_NO_MESH_HEADER
#define ADP_MESH_ALLSADDR_HEADER_LEN  0
#else
//Mesh header中源节点和目标节点全部使用短地址时, 长度固定为5个字节
#define ADP_MESH_ALLSADDR_HEADER_LEN  5
#endif


typedef struct _ADP_FWD_PKT {
   BYTE *data;  //points to top of original pkt as it sits in the heap
   BYTE adpOffset;  //start of the adpheader in this packet
}ADP_FWD_PKT;


typedef struct _ADP_PIB{
	union _ADP_PIB_FLAGS{
		BYTE val;
		struct {
			unsigned adpFormed:1;
		}bits;
	}flags;

	BYTE adpHCMethod;  //发送时对IPv6 Header的压缩策略:  1: 始终不压缩;0或其他值: 自动根据需要压缩
	
#ifdef LOWSN_FFD
	BYTE rxTail;             //tail pointer of rxBuff
    BYTE rxHead;             //head pointer of rxBuff
	//fifo for RX pkts, holds LOWSN_MAX_ADP_RX_PKTS
	ADP_FWD_PKT  rxBuff[ADP_RXBUFF_SIZE];  //buffer for packets to be forwarded
#endif
}ADP_PIB;



typedef struct _ADP_RX_DATA {
	MACPKT orgpkt;
	BYTE adpOffset;
	BYTE pload_offset;
	
	//路由头部内容
	SADDR dstSADDR;
   	SADDR srcSADDR;
	UINT16 srcPANID;

	//6LOWPAN头部内容

	BYTE Dispatch;

	BYTE HC1Encoding;
	
       BYTE hcflag; //0: uncompressed; 1: compressed
       
	// 当IP包是压缩格式时，才填充以下IP头解析的内容并传递给NWK
	
	BYTE TrafficClass;
	UINT32 FlowLabel;
	UINT16 PayloadLength;
	BYTE NextHeader;
	BYTE HopLimit;
	IPADDR SrcAddress;
   	IPADDR DstAddress;

}ADP_RX_DATA;


typedef struct _ADP_TX_DATA {
	SADDR dstSADDR;
	BYTE  *dstLADDR;
	LADDR dstLADDRFull;
   	SADDR srcSADDR;
	BYTE radius;
	BYTE fcflsb;
	BYTE fcfmsb;	
	
	BYTE MeshType;
	
	BYTE Dispatch;
	BYTE HC1Encoding;
	
}ADP_TX_DATA;

typedef union _ADP_ARGS {
   struct {
		UINT32 ScanChannels;
		BYTE ScanDuration;
		//UINT16 PANid;
	}form_network;
   struct {
		UINT32 ScanChannels;
		BYTE ScanDuration;
		//UINT16 PANid;
		BOOL  RejoinNetwork;
   }join_network;

   // 为了保持层之间的独立性和结构性，牺牲这些传递字节作为代价.
   // 另外一种方法是重新解析IP Header，获得这些属性值. 目前选择直接传递方式.
   struct {
	BYTE TrafficClass;
	UINT32 FlowLabel;
	BYTE NextHeader;
	BYTE HopLimit;
	IPADDR SrcAddress;
   	IPADDR DstAddress;
   }hc_info;   // this information is used for compresson decision
}ADP_ARGS;

typedef enum _ADP_STATE_ENUM {
  ADP_STATE_IDLE,
  ADP_STATE_COMMAND_START,
  ADP_STATE_GENERIC_TX_WAIT,
  ADP_STATE_FORM_NETWORK_START,
  ADP_STATE_JOIN_NETWORK_START,
  ADP_STATE_JOIN_NETWORK_START_WAIT,
  ADP_STATE_REJOIN_NETWORK_START,
  ADP_STATE_REJOIN_WAIT,
  ADP_STATE_JOIN_SEND_BEACON_REQ,
  ADP_STATE_JOIN_ADP_WAIT1_BREQ,
  ADP_STATE_JOIN_ADP_WAIT2_BREQ,
  ADP_STATE_JOIN_MAC_ASSOC_CHANSELECT,
  ADP_STATE_JOIN_MAC_ASSOC,
  ADP_STATE_JOIN_MAC_ASSOC_WAIT,
  ADP_STATE_FWD_WAIT
} ADP_STATE_ENUM;


typedef struct _ADP_SERVICE {
  LOWSN_SVC_ENUM cmd;
  ADP_ARGS args;
  LOWSN_STATUS_ENUM status;
}ADP_SERVICE;

extern ADP_SERVICE a_adp_service;
extern ADP_TX_DATA a_adp_tx_data;
extern ADP_STATE_ENUM adpState;

extern ADP_RX_DATA a_adp_rx_data;

UINT16 adpGetHopsToDest(SADDR dstSADDR);

BOOL adpRxBusy(void);
void adpRxHandoff(void);
void adpFmtCompHeader(void);
void adpTxCompData(void);
void adpParseCompHdr(void);

extern ADP_PIB adp_pib;

void adpFSM(void);
void adpInit(void);

#ifdef LOWSN_FFD
void adpFormNetworkDirectly(void);
#endif

void adpJoinNetworkDirectly(UINT16 my_saddr, UINT16 parent_saddr, UINT8 *parent_laddr, UINT8 my_depth);

#define adpIdle() (adpState == ADP_STATE_IDLE)
#define adpBusy() (adpState != ADP_STATE_IDLE)

#define adpDoService() \
   a_adp_service.status = LOWSN_STATUS_ADP_INPROGRESS;\
   adpState = ADP_STATE_COMMAND_START;\
   adpFSM();


#endif


