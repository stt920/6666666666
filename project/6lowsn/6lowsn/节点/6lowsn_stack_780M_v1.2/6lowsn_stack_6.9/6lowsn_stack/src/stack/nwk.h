/*********************************************************************
    文件名：nwk.h
    作  者：ji shanyang
    版  本：v1.0.0
    日  期：2012.12
    描  述：ipv6 protocol
*********************************************************************/

#ifndef _NWK_H
#define _NWK_H

#include "ds.h"
//#include "compiler.h"

/**
 * The size of the 6lowsn packet buffer, and the type of lowsn buffer.
 *
 * The lowsn packet buffer should not be smaller than 60 bytes, and does
 * not need to be larger than 1514 bytes. Lower size results in lower
 * TCP throughput, larger size results in higher TCP throughput.
 *
 * \hideinitializer
 */
#ifndef LOWSN_CONF_BUFFER_SIZE
#define LOWSN_BUFSIZE (LOWSN_LINK_MTU + LOWSN_LLH_LEN)
#else /* LOWSN_CONF_BUFFER_SIZE */
#define LOWSN_BUFSIZE (LOWSN_CONF_BUFFER_SIZE)
#endif /* LOWSN_CONF_BUFFER_SIZE */



/**
 * The lowsn packet buffer.
 *
 * the lowsn_buf array is used to hold outgoing packets. When sending
 * data, the device driver should read the link level headers and the TCP/IP
 * headers from this buffer.
 * the outgoing packet length is hold in lowsn_len;
 *
 * the lowsn_rx array is used to hold the incoming packets. the device driver
 * should place teh incoming data into this buffer.
 * the incoming packet length is hold in lowsn_rxlen;
 */







//------------------------------------------------------------------------

#ifdef LOWSN_CONF_UDP_CONNS
#define LOWSN_UDP_CONNS (LOWSN_CONF_UDP_CONNS)
#else /* LOWSN_CONF_UDP_CONNS */
#define LOWSN_UDP_CONNS    6
#endif /* LOWSN_CONF_UDP_CONNS */
/**
 * Representation of a LOWSN UDP connection.
 */
typedef struct lowsn_udp_connection {
  lowsn_ipaddr_t ripaddr;   /**< The IP address of the remote peer. */
  uint16_t lport;        /**< The local port number in network byte order. */
  uint16_t rport;        /**< The remote port number in network byte order. */
  uint8_t  ttl;          /**< Default time-to-live. */

  /** The application state. */
  //lowsn_udp_appstate_t appstate;
}lowsn_udp_conn_t;




//-------------------------------------------------------------------------

typedef union {
  uint32_t u32[(LOWSN_BUFSIZE + 3) / 4];
  uint8_t u8[LOWSN_BUFSIZE];
} lowsn_buf_t;

extern lowsn_buf_t lowsn_aligned_buf;
#define lowsn_buf (lowsn_aligned_buf.u8)
//extern lowsn_buf_t lowsn_aligned_rxbuf;
//#define lowsn_rxbuf (lowsn_aligned_rxbuf.u8)

extern lowsn_ds6_netif_t lowsn_ds6_if;
extern uint8_t lowsn_ext_len;
extern uint16_t lowsn_len;
extern lowsn_lladdr_t lowsn_lladdr;




//-------------------------------------------------------------------------
/*the buffer is used when the protocol stack recieve only one packet and the node shuold
  *send out tow packets. the node store the second sending packet in the buffer*/
typedef struct lowsn_swap_buffer{
  uint8_t buf[LOWSN_LINK_MTU];
  uint16_t buflen;	//the packet lenght in the swap buffer
}lowsn_swap_buf_t;




//-------------------------------------------------------------------------
/* The IPv6 header */
typedef struct lowsn_ip_hdr {
  uint8_t vtc;		//version 4-bits; traffic class4-bits(LSB);
  uint8_t tcflow;	//traffic class 4-bits(MSB); flow label 4-bits(LSB)
  uint16_t flow;	//flow label 16-bits(MSB)
  uint8_t len[2];	//payload lenght
  uint8_t proto, ttl;	//next header ; hop limit
  lowsn_ipaddr_t srcipaddr, destipaddr;		//source ipv6 address and destination ipv6 address
}lowsn_ip_hdr_t;

typedef struct lowsn_echo_hdr{
	uint16_t identifier;
	uint16_t sequence;
}lowsn_echo_hdr_t;


/*
 * IPv6 extension option headers: we are able to process
 * the 4 extension headers defined in RFC2460 (IPv6):
 * - Hop by hop option header, destination option header:
 *   These two are not used by any core IPv6 protocol, hence
 *   we just read them and go to the next. They convey options,
 *   the options defined in RFC2460 are Pad1 and PadN, which do
 *   some padding, and that we do not need to read (the length
 *   field in the header is enough)
 * - Routing header: this one is most notably used by MIPv6,
 *   which we do not implement, hence we just read it and go
 *   to the next
 * - Fragmentation header: we read this header and are able to
 *   reassemble packets
 *
 * We do not offer any means to send packets with extension headers
 *
 * We do not implement Authentication and ESP headers, which are
 * used in IPSec and defined in RFC4302,4303,4305,4385
 */
/* common header part */
typedef struct lowsn_ext_hdr {
  uint8_t next;
  uint8_t len;
} lowsn_ext_hdr_t;

/* Hop by Hop option header */
typedef struct lowsn_hbho_hdr {
  uint8_t next;
  uint8_t len;
} lowsn_hbho_hdr_t;

/* destination option header */
typedef struct lowsn_desto_hdr {
  uint8_t next;
  uint8_t len;
} lowsn_desto_hdr_t;

/* We do not define structures for PAD1 and PADN options */

/*
 * routing header
 * the routing header as 4 common bytes, then routing header type
 * specific data there are several types of routing header. Type 0 was
 * deprecated as per RFC5095 most notable other type is 2, used in
 * RFC3775 (MIPv6) here we do not implement MIPv6, so we just need to
 * parse the 4 first bytes
 */
typedef struct lowsn_routing_hdr {
  uint8_t next;
  uint8_t len;
  uint8_t routing_type;
  uint8_t seg_left;
} lowsn_routing_hdr_t;

/* fragmentation header */
typedef struct lowsn_frag_hdr {
  uint8_t next;
  uint8_t res;
  uint16_t offsetresmore;
  uint32_t id;
} lowsn_frag_hdr_t;
#define LOWSN_FRAGH_OFFSET_MASK 0xfff8
#define LOWSN_FRAGH_RES_MASK 0x0006
#define LOWSN_FRAGH_M_MASK 0x0001

#define LOWSN_REASS_TIMEROUT 60 //in uints of second



/*
 * an option within the destination or hop by hop option headers
 * it contains type an length, which is true for all options but PAD1
 */
typedef struct lowsn_ext_hdr_opt {
  uint8_t type;
  uint8_t len;
} lowsn_ext_hdr_opt_t;

/* PADN option */
typedef struct lowsn_ext_hdr_opt_padn {
  uint8_t opt_type;
  uint8_t opt_len;
} lowsn_ext_hdr_opt_padn_t;


/* Header sizes. */
#define LOWSN_IPH_LEN    40
#define LOWSN_FRAGH_LEN  8
#define LOWSN_UDPH_LEN    8    /* Size of UDP header */
#define LOWSN_TCPH_LEN   20    /* Size of TCP header */
#define LOWSN_ICMPH_LEN   4    /* Size of ICMP header */
#define LOWSN_IPUDPH_LEN (LOWSN_UDPH_LEN + LOWSN_IPH_LEN)    /* Size of IP +UDP header */
#define LOWSN_IPTCPH_LEN (LOWSN_TCPH_LEN + LOWSN_IPH_LEN)    /* Size of IP + TCP header */
#define LOWSN_TCPIP_HLEN LOWSN_IPTCPH_LEN
#define LOWSN_IPICMPH_LEN (LOWSN_IPH_LEN + LOWSN_ICMPH_LEN) /* size of ICMP + IP header */
#define LOWSN_LLIPH_LEN (LOWSN_LLH_LEN + LOWSN_IPH_LEN)    /* size of L2 + IP header */


/**
 * The sums below are quite used in ND. When used for lowsn_buf, we
 * include link layer length when used for lowsn_len, we do not, hence
 * we need values with and without LLH_LEN we do not use capital
 * letters as these values are variable
 */
#define lowsn_l2_l3_hdr_len (LOWSN_LLH_LEN + LOWSN_IPH_LEN + lowsn_ext_len)
#define lowsn_l2_l3_icmp_hdr_len (LOWSN_LLH_LEN + LOWSN_IPH_LEN + lowsn_ext_len + LOWSN_ICMPH_LEN)
#define lowsn_l3_hdr_len (LOWSN_IPH_LEN + lowsn_ext_len)
#define lowsn_l3_icmp_hdr_len (LOWSN_IPH_LEN + lowsn_ext_len + LOWSN_ICMPH_LEN)


#define LOWSN_IP6_VER        6
#define LOWSN_PROTO_TCP   6
#define LOWSN_PROTO_UDP   17
#define LOWSN_PROTO_ICMP6 58

/** @{ */
/** \brief  extension headers types */
#define LOWSN_PROTO_HBHO        0
#define LOWSN_PROTO_DESTO       60
#define LOWSN_PROTO_ROUTING     43
#define LOWSN_PROTO_FRAG        44
#define LOWSN_PROTO_NONE        59
/** @} */
/** @{ */
/**
 * \brief Bitmaps for extension header processing
 */
#define LOWSN_EXT_HDR_BITMAP_HBHO 0x01
#define LOWSN_EXT_HDR_BITMAP_DESTO1 0x02
#define LOWSN_EXT_HDR_BITMAP_ROUTING 0x04
#define LOWSN_EXT_HDR_BITMAP_FRAG 0x08
#define LOWSN_EXT_HDR_BITMAP_AH 0x10
#define LOWSN_EXT_HDR_BITMAP_ESP 0x20
#define LOWSN_EXT_HDR_BITMAP_DESTO2 0x40
/** @} */
/** @{ */
/** \brief  Destination and Hop By Hop extension headers option types */
#define LOWSN_EXT_HDR_OPT_PAD1  0
#define LOWSN_EXT_HDR_OPT_PADN  1
/** @} */


/*----------------------------------------------------------------------*/
/**define the states of network main FSM*/
typedef enum lowsn_ipv6_state{
	NWK_STATE_IDLE,
	NWK_STATE_CMD_START,
	NWK_STATE_NEXT_HOP_START,			//next hop determination start
//	NWK_STATE_CHECK_NEIGHBOR_CACHE,
	NWK_STATE_ADDR_RESOL_START,		//address resolution start
	NWK_STATE_FRAG_TX_WAIT,
	NWK_STATE_GENERIC_TX_WAIT,
	NWK_STATE_PING_REQ_WAIT1,
	NWK_STATE_PING_REQ_WAIT2,
	NWK_STATE_RS_WAIT1,
	NWK_STATE_RS_WAIT2,
	NWK_STATE_ADP_PASSTHRU_WAIT,
	NWK_STATE_SEND_PING_REPLY,
	#ifdef LOWSN_FFD
	NWK_STATE_SEND_RA,
	#endif
	NWK_STATE_SLIP_TX_WAIT, 
	NWK_STATE_SLIP_FORWARD, 
	NWK_STATE_GENERIC_TX_WAIT_AND_UNLOCK,
}lowsn_ipv6_state_t;

/**define the states of network recieving FSM*/
typedef enum lowsn_ipv6_rxstate {
	NWK_RXSTATE_IDLE,
	NWK_RXSTATE_START,
	NWK_RXSTATE_APS_HANDOFF,
	#ifdef LOWSN_COORDINATOR
	#ifdef LOWSN_SLIP_TO_HOST
  	NWK_RXSTATE_FORWARD_HOST, 
	#endif
	#endif
	NWK_RXSTATE_CMD_PENDING
} lowsn_ipv6_rxstate_t;



/**
*the defination of the network PIB attributes
*
*/
typedef struct lowsn_ipv6_pib{
	/*Flags*/
	union lowsn_ipv6_pib_flags{
		UINT16 val;
		struct {
			unsigned nwkPending:1;		/*indicate the network layer need to send a paket generated by the network layer,
															and the packet is stored in lowsn_buf buffer.*/
			//unsigned sendicmperr:1;
			//unsigned sendresponse:1;
			//unsigned sendqueue:1;
			unsigned nwkIsGetEchoReply:1;
			unsigned WaitingForEchoRelay:1;
			unsigned nwkIsGetRA:1;
			unsigned WaitingForRA:1;
			unsigned nwktxfragment:1;	//indicate the network layer is sending a fragment packet
			unsigned slipforwardPending:1; //是否存在SLIP转发数据
			unsigned slipRxPending:1; //是否存在SLIP转发数据
		}bits;
	}flags;

	struct  {
		UINT16 Identifier;
		UINT16 SequenceNumber;
		UINT16 Plen;
	}lastPing; //记录发出的ping request的信息
		
}lowsn_ipv6_pib_t;



typedef struct _NWK_TX_DATA {
	BYTE Version;
	BYTE TrafficClass;
	UINT32 FlowLabel;
	UINT16 PayloadLength;
	BYTE NextHeader;
	BYTE HopLimit;
	IPADDR SrcAddress;
   	IPADDR DstAddress;
	//BYTE IPStartPtr*;  //在tmpTxBuff 中指向标准IPv6头起始位置的指针


	//为了兼容Zigbee应用层而加的 原NWK定义，无意义

	SADDR dstSADDR;
	BYTE  *dstLADDR;
	LADDR dstLADDRFull;
   	SADDR srcSADDR;
	BYTE radius;
	BYTE fcflsb;
	BYTE fcfmsb;	
	
}NWK_TX_DATA;


typedef struct _NWK_RX_DATA {
	MACPKT orgpkt;
	BYTE nwkOffset; // nwk帧头起始位置, 仅用于IP非压缩时使用,IP压缩时该指针不可用
	BYTE pload_offset; // 指向IP头部之后的负载起始位置，IP扩展头部也算负载, IP压缩时由适配层传递上来
	BYTE hcflag;      //指示IP头是否是压缩的，由ADP传给NWK，若是压缩的，则IP头的解析由ADP负责，否则由NWK负责
	BYTE TrafficClass;
	UINT32 FlowLabel;
	UINT16 PayloadLength;
	BYTE NextHeader;
	BYTE HopLimit;
	IPADDR SrcAddress;
   	IPADDR DstAddress;

	// 仅为了兼容Zigbee应用层，无意义

	SADDR dstSADDR;
   	SADDR srcSADDR;
}NWK_RX_DATA;



typedef union lowsn_ipv6_args {
/* define the argument a servie need :for example */
   struct {
		uint8_t example;
   }join_network;//end example

	struct{
		lowsn_ipaddr_t dstaddr;
		UINT16 plen;
	}tx_ping6;

	struct {
		UINT16 WaitDuration;
 	}get_ra;//end example

}lowsn_ipv6_args_t;


typedef struct lowsn_ipv6_service {
  LOWSN_SVC_ENUM cmd;
  lowsn_ipv6_args_t args;
  LOWSN_STATUS_ENUM status;
}lowsn_ipv6_service_t;

extern lowsn_ipv6_state_t nwkState;
extern lowsn_ipv6_rxstate_t nwkRxState;
extern lowsn_ipv6_service_t a_nwk_service;
extern lowsn_ipv6_pib_t nwk_pib;
extern NWK_TX_DATA a_nwk_tx_data;
extern NWK_RX_DATA a_nwk_rx_data;


#define nwkIdle() (nwkState == NWK_STATE_IDLE)
#define nwkBusy() (nwkState != NWK_STATE_IDLE)


#define nwkDoService() \
   a_nwk_service.status = LOWSN_STATUS_NWK_INPROGRESS;\
   nwkState = NWK_STATE_CMD_START;\
   nwkFSM();










#define NWK_IP_VER_MASK  0xF0

#define NWK_IS_IP6(x) ((BYTE)((x & NWK_IP_VER_MASK) >> 4) == LOWSN_IP6_VER)





/*---------------------------------------------------------------------------*/
uint16_t
lowsn_chksum(uint16_t *data, uint16_t len);

uint16_t
lowsn_ipchksum(BYTE *iphead_start);

static uint16_t
upper_layer_chksum(uint8_t proto, BYTE *iphead_start, UINT16 ipplen);

uint16_t
lowsn_icmp6chksum(BYTE *iphead_start, UINT16 ipplen);

uint16_t
lowsn_tcpchksum(BYTE *iphead_start, UINT16 ipplen);

uint16_t
lowsn_udpchksum(BYTE *iphead_start, UINT16 ipplen);

/*---------------------------------------------------------------------------*/

void nwkFSM(void);

void nwkRxFSM(void);

void nwkInit(void);

//void nwkTxData(lowsn_lladdr_t* lladdr);

//void nwk_rx_handoff(uint8_t* data, uint16_t len);

#if 0
uint8_t lowsn_ext_hdr_process(void);

uint8_t lowsn_reassembly(void);

void lowsn_reass_timeout(void);

uint8_t lowsn_udp_parse(void);

void lowsn_tx_first_frag(void);
void lowsn_tx_other_frag(void);
#endif






#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
void nwkRestoreCompHeader(void); 
void nwkParseSlipHdr(BYTE *ptr);
void nwkInjectSlipTxPacket(void);
BOOL nwkInjectSlipRxPacket(void);
#endif
#endif

#if 1
int 
Nwk_Forward_Outside(UINT8 Flag_G,
						IPADDR SrcIpAddr_G,
						IPADDR DstIpAddr_G,
						UINT16 RemotePort_G,
						UINT16 LocalPort_G,
						BYTE * Pload_G,
						BYTE Length_G,
						BYTE NextHeader_G);
#endif





BOOL nwk_buf_free(void);

BOOL nwkRxBusy(void);

void nwkTxData(BOOL forward_flag);

void nwkParseHdr(BYTE *ptr);

void nwkFindDstDLLAddr(void);

#endif






























