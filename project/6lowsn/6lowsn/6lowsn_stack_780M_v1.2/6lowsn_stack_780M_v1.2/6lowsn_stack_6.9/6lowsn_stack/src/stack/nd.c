/*********************************************************************
    文件名：nd.c
    作  者：ji shanyang
    版  本：v1.0.0
    日  期：2012.12
    描  述：Neighbor Discover Protocol
*********************************************************************/

#include <stdlib.h>
#include <string.h>
#include "6lowsn_common_types.h"
#include "6lowsn_config.h"
#include "halstack.h"
#include "console.h"
#include "debug.h"
#include "ds.h"
#include "nwk.h"
#include "icmpv6.h"
#include "mac.h"
#include "phy.h"
#include "nd.h"
#include "neighbor.h"





/*------------------------------------------------------------------*/
/** @{ */
/** \name Pointers to the header structures.
 *  All pointers except LOWSN_IP_BUF depend on lowsn_ext_len, which at
 *  packet reception, is the total length of the extension headers.
 *
 *  The pointer to ND6 options header also depends on nd6_opt_offset,
 *  which we set in each function.
 *
 *  Care should be taken when manipulating these buffers about the
 *  value of these length variables
 */
extern lowsn_ds6_pmtu_t pmtu;
extern lowsn_swap_buf_t lowsn_swap_buf;
extern lowsn_ds6_prefix_t lowsn_ds6_prefix_list[LOWSN_DS6_PREFIX_NB];	//brief prefix list


#define LOWSN_IP_BUF                ((lowsn_ip_hdr_t *)&lowsn_buf[LOWSN_LLH_LEN])  /**< Pointer to IP header */
#define LOWSN_ICMP_BUF            ((lowsn_icmp6_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])  /**< Pointer to ICMP header*/
/**@{  Pointers to messages just after icmp header */
#define LOWSN_ND6_RS_BUF            ((lowsn_nd6_rs_t  *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len])
#define LOWSN_ND6_RA_BUF            ((lowsn_nd6_ra_t  *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len])
#define LOWSN_ND6_NS_BUF            ((lowsn_nd6_ns_t  *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len])
#define LOWSN_ND6_NA_BUF            ((lowsn_nd6_na_t  *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len])
#define LOWSN_ND6_RED_BUF			((lowsn_nd6_red_t  *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len])
/** @} */
/** Pointer to ND option */
#define LOWSN_ND6_OPT_HDR_BUF  ((lowsn_nd6_opt_hdr_t *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define LOWSN_ND6_OPT_PREFIX_BUF ((lowsn_nd6_opt_prefix_info_t *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define LOWSN_ND6_OPT_MTU_BUF ((lowsn_nd6_opt_mtu_t *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len + nd6_opt_offset])
/** @} */



static lowsn_ipaddr_t ipaddr;
static lowsn_ds6_prefix_t *prefix; /**  Pointer to a prefix list entry */
static lowsn_ds6_uaddr_t *addr; /**  Pointer to an interface uincast address */

extern lowsn_ipv6_pib_t nwk_pib;

//static uint16_t nd6_opt_offset; /** Offset from the end of the icmpv6 header to the option in lowsn_buf*/
//static uint8_t *nd6_opt_llao;   /**  Pointer to llao option in lowsn_buf */
//static lowsn_nd6_opt_prefix_info_t *nd6_opt_prefix_info; /**  Pointer to prefix information option in lowsn_buf */
//static lowsn_ds6_nbr_t *nbr; /**  Pointer to a nbr cache entry*/
//static lowsn_ds6_defrt_t *defrt; /**  Pointer to a router list entry */
//static lowsn_ds6_dest_t* dest; /**  Pointer to an destination cache entry */
//extern lowsn_ds6_dest_t lowsn_ds6_dest_cache[LOWSN_DS6_DEST_NB];



/*---------------------------------------------------------------------------*/
// 构造Router Solicitation Message
// RFC4861 pp.18

/*---------------------------------------------------------------------------*/

void ndFmtRS(void)
{
	DEBUG_STRING(DBG_INFO,"ND: Format RS.\n");

	phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];

	// 填写options
	// Valid Options: Source link-layer address The link-layer address of the sender, if known. MUST NOT be included if the Source Address
       // is the unspecified address. Otherwise, it SHOULD be included on link layers that have addresses.
	// 暂时不填写，在mesh under下，无论是否是未指定地址，该域实际上可精简掉，mesh头
	// 会含有源节点的链路层地址

	
	// Reserved
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;

	//checksum is added when all packet is ready, use 0 here.
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;	

	//code: 0
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	
	//type: 133
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = ICMP6_RS;


	phy_pib.currentTxFlen = 4 + LOWSN_ICMPH_LEN;


	// set IP header
	a_nwk_tx_data.Version = LOWSN_IP6_VER;
	a_nwk_tx_data.TrafficClass=0x00;
	a_nwk_tx_data.FlowLabel = 0x00;
	a_nwk_tx_data.PayloadLength = phy_pib.currentTxFlen;
	a_nwk_tx_data.NextHeader = LOWSN_PROTO_ICMP6;
	a_nwk_tx_data.HopLimit = LOWSN_ND6_HOP_LIMIT;

	lowsn_create_linklocal_allrouters_mcast(&a_nwk_tx_data.DstAddress);

	ds6FindSrcIP(&a_nwk_tx_data.SrcAddress, &a_nwk_tx_data.DstAddress);
	//lowsn_create_unspecified(&a_nwk_tx_data.SrcAddress);

	//a_nwk_tx_data.IPStartPtr = phy_pib.currentTxFrm - LOWSN_IPH_LEN;

	return;

}



/*---------------------------------------------------------------------------*/
/**
 * 构造 router advertisment
 active_flag: 1: 路由器主动发起的RA，目标地址为组播地址；
                 0或其他值:路由器收到RS后发出的RA，会额外解析收到的RS帧

 *
 --------------------------------------------------------------------------*/
 #ifdef LOWSN_FFD

void ndFmtRA(BYTE active_flag)
{

	BYTE i;

	DEBUG_STRING(DBG_INFO,"ND: Format RA.\n");

	phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];
	phy_pib.currentTxFlen = 0;

	// options域目前只支持Prefix Information
	for(prefix = lowsn_ds6_prefix_list; prefix < lowsn_ds6_prefix_list + LOWSN_DS6_PREFIX_NB; prefix++) {
    		//if((prefix->isused) && (prefix->advertise)) {
    		if((prefix->isused) && (prefix->advertise)) {
			// Prefix	
			for(i=0; i<16; i++) {
       			phy_pib.currentTxFrm--;
              		*phy_pib.currentTxFrm = (prefix->ipaddr).u8[15-i];
       		}

			#ifndef LOWSN_RA_COMPRESS
			// Reserved2
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0x0;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0x0;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0x0;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0x0;

			//Preferred Lifetime，暂时设定为无限长
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;

			//Valid Lifetime，暂时设定为无限长
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 0xFF;				
			#endif
				
			// L, A, Reserved1
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = prefix->l_a_reserved;

			// Prefix Length
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = prefix->length;

			//Length
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = 4;

			// Type
			phy_pib.currentTxFrm--;
			*phy_pib.currentTxFrm = LOWSN_ND6_OPT_PREFIX_INFO;	

			#ifndef LOWSN_RA_COMPRESS
			phy_pib.currentTxFlen += LOWSN_ND6_OPT_PREFIX_INFO_LEN;
			#else
			phy_pib.currentTxFlen += LOWSN_ND6_OPT_PREFIX_INFO_LEN - 12;
			#endif
    		}
	}		

	#ifndef  LOWSN_RA_COMPRESS
	// Retrans Timer: 0
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;

	// Reachable Time: 0
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;

	// Router Lifetime: 0
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	
	// Reserved, M, O: 0
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	#endif

	// Cur Hop Limit
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = lowsn_ds6_if.cur_hop_limit;


	//checksum is added when all packet is ready, use 0 here.
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;	

	//code: 0
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	
	//type: 134
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = ICMP6_RA;

	#ifndef  LOWSN_RA_COMPRESS
	phy_pib.currentTxFlen += (LOWSN_ND6_RA_LEN + LOWSN_ICMPH_LEN);
	#else
	phy_pib.currentTxFlen += (LOWSN_ND6_RA_LEN -11 + LOWSN_ICMPH_LEN);
	#endif

	// set IP header
	a_nwk_tx_data.Version = LOWSN_IP6_VER;
	a_nwk_tx_data.TrafficClass=0x00;
	a_nwk_tx_data.FlowLabel = 0x00;
	a_nwk_tx_data.PayloadLength = phy_pib.currentTxFlen;
	a_nwk_tx_data.NextHeader = LOWSN_PROTO_ICMP6;
	a_nwk_tx_data.HopLimit = LOWSN_ND6_HOP_LIMIT;

	// 目标节点地址
	if (active_flag == 1)  {
		lowsn_create_linklocal_allnodes_mcast(&a_nwk_tx_data.DstAddress);
	}
	else {
		// 使用收到的RS帧中的地址
		lowsn_ipaddr_copy(&a_nwk_tx_data.DstAddress, &a_nwk_rx_data.SrcAddress);
		//memcpy(&a_nwk_tx_data.DstAddress, &a_nwk_rx_data.SrcAddress, 16);
	}	

	//  源节点地址
       // MUST be the link-local address assigned to the interface from which this message is sent.
      //ds6FindSrcIP(&a_nwk_tx_data.SrcAddress, &a_nwk_tx_data.DstAddress);
	lowsn_ip6addr(&a_nwk_tx_data.SrcAddress,0xfe80,0,0,0,0,0,0,0);

	//a_nwk_tx_data.IPStartPtr = phy_pib.currentTxFrm - LOWSN_IPH_LEN;


	return;

}

#endif


/*---------------------------------------------------------------------------

 *
 * 解析Router Advertisement Message
*
*  暂时只支持更新prefix
 *

---------------------------------------------------------------------------*/


void ndParseRA(void)
{

	BYTE *ptr;
	BYTE options_len;
	BYTE *prefix_postion;

	
	
	DEBUG_STRING(DBG_INFO,"ND: Received RA.\n");

	ptr = a_nwk_rx_data.orgpkt.data + a_nwk_rx_data.pload_offset;


	// 对IPv6头部、ICMP头部、options长度等进行必要的检查，暂忽略。

	// 只解析前缀，对其他域的解析暂忽略

	// 指向options起始位置
	
	#ifndef  LOWSN_RA_COMPRESS
	options_len = LOWSN_ICMPH_LEN+ LOWSN_ND6_RA_LEN;
	#else
	options_len = LOWSN_ICMPH_LEN+ LOWSN_ND6_RA_LEN - 11;
	#endif

	while(options_len < a_nwk_rx_data.PayloadLength) {

		ptr = ptr + options_len;
		options_len = options_len + (*(ptr+1)) << 3;

		if (*ptr == LOWSN_ND6_OPT_PREFIX_INFO)  {
			
				DEBUG_STRING(DBG_INFO,"ND: Find RA Prefix.\n");

				#ifndef LOWSN_RA_COMPRESS
				 prefix_postion = ptr + 16;
				#else
				prefix_postion = ptr + 16 - 12;
				#endif 

				if(!lowsn_is_addr_link_local((IPADDR *)(prefix_postion))) {
					
					if((*(ptr+3)) & LOWSN_ND6_RA_FLAG_ONLINK) {
						
						prefix = lowsn_ds6_prefix_lookup((IPADDR *)(prefix_postion), *(ptr+2));
						if(prefix == NULL) {
							// 暂时发的总是时间无限长的前缀
                                                 #ifdef LOWSN_FFD
							prefix =  lowsn_ds6_prefix_add((IPADDR *)(prefix_postion), *(ptr+2), 1, 0xC0, 0, 0);
							#else
							prefix = lowsn_ds6_prefix_add((IPADDR *)(prefix_postion), *(ptr+2), 0);
							#endif																			
                                                    
							 // 收到前缀后的回调函数
							  usrGetPrefixCallback((IPADDR *)(prefix_postion), *(ptr+2));
							  
						}
					}
					
					if(((*(ptr+3)) & LOWSN_ND6_RA_FLAG_AUTONOMOUS) && (*(ptr+2) == LOWSN_DEFAULT_PREFIX_LEN)) {
	
						lowsn_ipaddr_copy(&ipaddr, (IPADDR *)(prefix_postion));		//copy the ipv6 address prefix
						if (lowsn_ds6_if.iid_gen_method == LOWSN_IID_GLOBAL_SHORTADDR) {
							ds6GenInterfaceID16(&ipaddr,  macGetPanID(), macGetShortAddr());
						}
						else  {
							ds6GenInterfaceID64(&ipaddr);
						}
							
						addr = lowsn_ds6_uaddr_lookup(&ipaddr);
						if(addr == NULL) {
							if (lowsn_ds6_if.iid_gen_method == LOWSN_IID_GLOBAL_SHORTADDR) {
								lowsn_ds6_uaddr_add(&ipaddr, 0, ADDR_AUTOCONF_SHORTADDR);
							}	
							else  {
								lowsn_ds6_uaddr_add(&ipaddr, 0, ADDR_AUTOCONF_EUI64);
							}
						}		
							
					}
					
				  }
		}

		else  {
			
				lowsn_len = 0;
				DEBUG_STRING(DBG_INFO,"ND: Received RA option is not supported.\n");
		}
	}	

 	 return;

}



#if 0


/*------------------------------------------------------------------*/
/* create a link layer address option */
static void
create_llao(uint8_t *llao, uint8_t type) {
  llao[LOWSN_ND6_OPT_TYPE_OFFSET] = type;
  llao[LOWSN_ND6_OPT_LEN_OFFSET] = LOWSN_ND6_OPT_LLAO_LEN >> 3;
  memcpy(&llao[LOWSN_ND6_OPT_DATA_OFFSET], &lowsn_lladdr, LOWSN_LLADDR_LEN);
  /* padding on some */
  memset(&llao[LOWSN_ND6_OPT_DATA_OFFSET + LOWSN_LLADDR_LEN], 0,
         LOWSN_ND6_OPT_LLAO_LEN - 2 - LOWSN_LLADDR_LEN);
}


/*------------------------------------------------------------------*/

 /**
 * \brief Process a neighbor solicitation
 *
 * The NS can be received in 3 cases (procedures):
 * - sender is performing DAD (ip src = unspecified, no SLLAO option)
 * - sender is performing NUD (ip dst = unicast)
 * - sender is performing address resolution (ip dest = solicited node mcast
 * address)
 *
 * We do:
 * - if the tgt belongs to me, reply, otherwise ignore
 * - if i was performing DAD for the same address, two cases:
 * -- I already sent a NS, hence I win
 * -- I did not send a NS yet, hence I lose
 *
 * If we need to send a NA in response (i.e. the NS was done for NUD, or
 * address resolution, or DAD and there is a conflict), we do it in this
 * function: set src, dst, tgt address in the three cases, then for all cases
 * set the rest, including  SLLAO
 *
 */
void
lowsn_nd_ns_input(void)
{
	uint8_t flags;
	uint8_t llaoflag = 0;
	
	
	DEBUG_STRING(DBG_INFO, "Received NS from ");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, " to ");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, " with target address");
	PRINT6ADDR((lowsn_ipaddr_t *) (&LOWSN_ND6_NS_BUF->tgtipaddr));
	DEBUG_STRING(DBG_INFO, "\r\n");
	
#if LOWSN_CONF_IPV6_CHECKS
	if((LOWSN_IP_BUF->ttl != LOWSN_ND6_HOP_LIMIT) ||
			(lowsn_is_addr_mcast(&LOWSN_ND6_NS_BUF->tgtipaddr)) ||
			(LOWSN_ICMP_BUF->icode != 0)) {
		DEBUG_STRING(DBG_INFO, "NS received is bad\r\n");
		goto discard;
	}
	if(lowsn_len-40 < 24){
		DEBUG_STRING(DBG_INFO, "Discard RS for ICMP len\r\n");
		goto discard;
	}
#endif /* LOWSN_CONF_IPV6_CHECKS */

	/* Options processing */
	nd6_opt_llao = NULL;
	nbr = NULL;
	nd6_opt_offset = LOWSN_ND6_NS_LEN;
	nbr = lowsn_ds6_nbr_lookup(&LOWSN_IP_BUF->srcipaddr);
	while(lowsn_l3_icmp_hdr_len + nd6_opt_offset < lowsn_len) {
#if LOWSN_CONF_IPV6_CHECKS
		if(LOWSN_ND6_OPT_HDR_BUF->len == 0) {
			DEBUG_STRING(DBG_INFO, "NS received is bad\r\n");
			goto discard;
		}
#endif /* LOWSN_CONF_IPV6_CHECKS */
		switch (LOWSN_ND6_OPT_HDR_BUF->type) {
			case LOWSN_ND6_OPT_SLLAO:		//source link layer address option
				llaoflag = 1;
				nd6_opt_llao = &lowsn_buf[lowsn_l2_l3_icmp_hdr_len + nd6_opt_offset];
#if LOWSN_CONF_IPV6_CHECKS
				/* There must be NO option in a DAD NS */
				if(lowsn_is_addr_unspecified(&LOWSN_IP_BUF->srcipaddr)) {
					DEBUG_STRING(DBG_INFO, "NS received is bad\r\n");
					goto discard;
				}
				else {
#endif /*LOWSN_CONF_IPV6_CHECKS */
//					nbr = lowsn_ds6_nbr_lookup(&LOWSN_IP_BUF->srcipaddr);
					if(nbr == NULL) {
						/*if the neighbor is not in the NBC then add a new entry in the NBC*/
						lowsn_ds6_nbr_add(&LOWSN_IP_BUF->srcipaddr,
								(lowsn_lladdr_t *)&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET],
								0, NBR_STALE);
    	 		   }
					else {
						/*the neighbor is in the NCE but the link layer address in the NS message is different
						  *from that int NBC entry,then the node should update the neighbore's link layer address
						  *and update the neighbor cache entry's state to STALE*/
						if(memcmp(&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET],
								&nbr->lladdr, LOWSN_LLADDR_LEN) != 0) {
							memcpy(&nbr->lladdr, &nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET],LOWSN_LLADDR_LEN);
							nbr->state = NBR_STALE;
						}
						else {
							/*if the NCE's state is INCOMPLETE, the node should send the queuing packet in the
							  *queuing buffer(if any) and then send a NA message to respond to the NS message*/
							if(nbr->state == NBR_INCOMPLETE) {
								nbr->state = NBR_STALE;
							}
						}
					}
#if LOWSN_CONF_IPV6_CHECKS
				}
#endif /*LOWSN_CONF_IPV6_CHECKS */
			break;

			default:
				DEBUG_STRING(DBG_INFO, "ND option not supported in NS");
			break;
		}
		nd6_opt_offset += (LOWSN_ND6_OPT_HDR_BUF->len << 3);
	}
	/*if there is no source linl layer address option in the NS message, the host MUST Do anything*/
	if(nbr != NULL){
		if((nbr->state == NBR_INCOMPLETE)&&(llaoflag == 0)){
			lowsn_len = 0;
			return;
		}
	}


	addr = lowsn_ds6_uaddr_lookup(&LOWSN_ND6_NS_BUF->tgtipaddr);
	if(addr != NULL) {
#if LOWSN_ND6_DEF_MAXDADNS > 0
		if(lowsn_is_addr_unspecified(&LOWSN_IP_BUF->srcipaddr)) {
			/* DAD CASE */
#if LOWSN_CONF_IPV6_CHECKS
			if(!lowsn_is_addr_solicited_node(&LOWSN_IP_BUF->destipaddr)) {
				DEBUG_STRING(DBG_INFO, "NS received is bad\r\n");
				goto discard;
			}
#endif /* LOWSN_CONF_IPV6_CHECKS */
			if(addr->state != ADDR_TENTATIVE) {
				lowsn_create_linklocal_allnodes_mcast(&LOWSN_IP_BUF->destipaddr);
				ds6FindSrcIP(&LOWSN_IP_BUF->srcipaddr, &LOWSN_IP_BUF->destipaddr);
				flags = LOWSN_ND6_NA_FLAG_OVERRIDE;
				goto create_na;
			}
			else {
				/** \todo if I sent a NS before him, I win */
				lowsn_ds6_dad_failed(addr);
				goto discard;
			}
		}
#else /* LOWSN_ND6_DEF_MAXDADNS > 0 */
		if(lowsn_is_addr_unspecified(&LOWSN_IP_BUF->srcipaddr)) {
			/* DAD CASE */
			goto discard;
		}
#endif /* LOWSN_ND6_DEF_MAXDADNS > 0 */


#if LOWSN_CONF_IPV6_CHECKS
		if(lowsn_ds6_is_my_uaddr(&LOWSN_IP_BUF->srcipaddr)) {
        /**
         * \NOTE do we do something here? we both are using the same address.
         * If we are doing dad, we could cancel it, though we should receive a
         * NA in response of DAD NS we sent, hence DAD will fail anyway. If we
         * were not doing DAD, it means there is a duplicate in the network!
         */
			DEBUG_STRING(DBG_INFO, "NS received is bad\r\n");
			goto discard;
		}
#endif /*LOWSN_CONF_IPV6_CHECKS */

		/* Address resolution case */
		if(lowsn_is_addr_solicited_node(&LOWSN_IP_BUF->destipaddr)) {
			lowsn_ipaddr_copy(&LOWSN_IP_BUF->destipaddr, &LOWSN_IP_BUF->srcipaddr);
			lowsn_ipaddr_copy(&LOWSN_IP_BUF->srcipaddr, &LOWSN_ND6_NS_BUF->tgtipaddr);
			flags = LOWSN_ND6_NA_FLAG_SOLICITED | LOWSN_ND6_NA_FLAG_OVERRIDE;
			goto create_na;
		}

    /* NUD CASE */
		if(lowsn_ds6_uaddr_lookup(&LOWSN_IP_BUF->destipaddr) == addr) {
			lowsn_ipaddr_copy(&LOWSN_IP_BUF->destipaddr, &LOWSN_IP_BUF->srcipaddr);
			lowsn_ipaddr_copy(&LOWSN_IP_BUF->srcipaddr, &LOWSN_ND6_NS_BUF->tgtipaddr);
			flags = LOWSN_ND6_NA_FLAG_SOLICITED | LOWSN_ND6_NA_FLAG_OVERRIDE;
			goto create_na;
		}
		else {
#if LOWSN_CONF_IPV6_CHECKS
			DEBUG_STRING(DBG_INFO, "NS received is bad\r\n");
			goto discard;
#endif /* LOWSN_CONF_IPV6_CHECKS */
		}
	}
	else {
		goto discard;
	}


create_na:
	/* If the node is a router it should set R flag in NAs */
#ifdef LOWSN_COORDINATOR
	flags = flags | LOWSN_ND6_NA_FLAG_ROUTER;
#endif
	lowsn_ext_len = 0;
	LOWSN_IP_BUF->vtc = 0x60;
	LOWSN_IP_BUF->tcflow = 0;
	LOWSN_IP_BUF->flow = 0;
	LOWSN_IP_BUF->len[0] = 0;       /* length will not be more than 255 */
	LOWSN_IP_BUF->len[1] = LOWSN_ICMPH_LEN + LOWSN_ND6_NA_LEN + LOWSN_ND6_OPT_LLAO_LEN;
	LOWSN_IP_BUF->proto = LOWSN_PROTO_ICMP6;
	LOWSN_IP_BUF->ttl = LOWSN_ND6_HOP_LIMIT;

	LOWSN_ICMP_BUF->type = ICMP6_NA;
	LOWSN_ICMP_BUF->icode = 0;

	LOWSN_ND6_NA_BUF->flagsreserved = flags;
	memcpy(&LOWSN_ND6_NA_BUF->tgtipaddr, &addr->ipaddr, sizeof(lowsn_ipaddr_t));

	create_llao(&lowsn_buf[lowsn_l2_l3_icmp_hdr_len + LOWSN_ND6_NA_LEN], LOWSN_ND6_OPT_TLLAO);

	LOWSN_ICMP_BUF->icmpchksum = 0;
	LOWSN_ICMP_BUF->icmpchksum = ~ lowsn_icmp6chksum();

	lowsn_len = LOWSN_IPH_LEN + LOWSN_ICMPH_LEN + LOWSN_ND6_NA_LEN + LOWSN_ND6_OPT_LLAO_LEN;
	
	DEBUG_STRING(DBG_INFO, "Sending NA to ");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, " from ");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, " with target address ");
	PRINT6ADDR(&LOWSN_ND6_NA_BUF->tgtipaddr);
	DEBUG_STRING(DBG_INFO, "\r\n");
	
	/* send the queuing paket in the queuing buffer, because the node can't send the
	 * queuing buffer at once, so we should copy the NA message to the swap buffer
	 * and copy the queuing buffer of the neighbor to the lowsn_buf*/
#if LOWSN_CONF_IPV6_QUEUE_PKT
	if((nbr!=NULL)&&(nbr->queue_buflen!=0)){
//		if(tmpnbr->queue_buflen != 0){
			DEBUG_STRING(DBG_INFO, "Copy NA to swap buf\r\n");
			lowsn_swap_buf.buflen = lowsn_len;
			memcpy(lowsn_swap_buf.buf, LOWSN_IP_BUF, lowsn_swap_buf.buflen);
			
			DEBUG_STRING(DBG_INFO, "Copy the queuing pkt to lowsn_buf\r\n");
//			lowsn_len = tmpnbr->queue_buflen;
//			memcpy(LOWSN_IP_BUF, tmpnbr->queue_buf, lowsn_len);
			lowsn_len = nbr->queue_buflen;
			memcpy(LOWSN_IP_BUF, nbr->queue_buf, lowsn_len);
//		}
	}
#endif /*LOWSN_CONF_IPV6_QUEUE_PKT*/

	return;

discard:
	lowsn_len = 0;
	return;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Send a neighbor solicitation,
 * \param src pointer to the src of the NS if known
 * \param dest pointer to ip address to send the NS, for DAD or ADDR Resol,
 * MUST be NULL, for NUD, must be correct unicast dest
 * \param tgt  pointer to ip address to fill the target address field, must
 * not be NULL
 *
 * - RFC 4861, 7.2.2 :
 *   "If the source address of the packet prompting the solicitation is the
 *   same as one of the addresses assigned to the outgoing interface, that
 *   address SHOULD be placed in the IP Source Address of the outgoing
 *   solicitation.  Otherwise, any one of the addresses assigned to the
 *   interface should be used."
 *   This is why we have a src ip address as argument. If NULL, we will do
 *   src address selection, otherwise we use the argument.
 *
 * - we check if it is a NS for Address resolution  or NUD, if yes we include
 *   a SLLAO option, otherwise no.
 */
void
lowsn_nd_ns_output(lowsn_ipaddr_t *src, lowsn_ipaddr_t *dest, lowsn_ipaddr_t *tgt){

	lowsn_ext_len = 0;
	LOWSN_IP_BUF->vtc = 0x60;
	LOWSN_IP_BUF->tcflow = 0;
	LOWSN_IP_BUF->flow = 0;
	LOWSN_IP_BUF->proto = LOWSN_PROTO_ICMP6;
	LOWSN_IP_BUF->ttl = LOWSN_ND6_HOP_LIMIT;
	/*add the destination IPv6 address for NS message*/
	if(dest == NULL) {
		/*for address resolution and duplicate address detctation*/
		lowsn_create_solicited_node(tgt, &LOWSN_IP_BUF->destipaddr);
	}
	else {
		/*for neighbor unreachable detctation*/
		lowsn_ipaddr_copy(&LOWSN_IP_BUF->destipaddr, dest);
	}
	LOWSN_ICMP_BUF->type = ICMP6_NS;
	LOWSN_ICMP_BUF->icode = 0;
	LOWSN_ND6_NS_BUF->reserved = 0;
	lowsn_ipaddr_copy((lowsn_ipaddr_t *) &LOWSN_ND6_NS_BUF->tgtipaddr, tgt);
	LOWSN_IP_BUF->len[0] = 0;       /* length will not be more than 255 */
  /*
   * check if we add a SLLAO option: for DAD, MUST NOT, for NUD, MAY
   * (here yes), for Address resolution , MUST
   */
	if(!(lowsn_ds6_is_my_uaddr(tgt))) {
		if(src != NULL) {
		lowsn_ipaddr_copy(&LOWSN_IP_BUF->srcipaddr, src);
	}
	else {
		ds6FindSrcIP(&LOWSN_IP_BUF->srcipaddr, &LOWSN_IP_BUF->destipaddr);
	}
	if (lowsn_is_addr_unspecified(&LOWSN_IP_BUF->srcipaddr)) {
		DEBUG_STRING(DBG_INFO, "Dropping NS due to no suitable source address\r\n");
		lowsn_len = 0;
		return;
	}
		LOWSN_IP_BUF->len[1] =
		LOWSN_ICMPH_LEN + LOWSN_ND6_NS_LEN + LOWSN_ND6_OPT_LLAO_LEN;

		create_llao(&lowsn_buf[lowsn_l2_l3_icmp_hdr_len + LOWSN_ND6_NS_LEN], LOWSN_ND6_OPT_SLLAO);

		lowsn_len = LOWSN_IPH_LEN + LOWSN_ICMPH_LEN + LOWSN_ND6_NS_LEN + LOWSN_ND6_OPT_LLAO_LEN;
	}
	else {
		lowsn_create_unspecified(&LOWSN_IP_BUF->srcipaddr);
		LOWSN_IP_BUF->len[1] = LOWSN_ICMPH_LEN + LOWSN_ND6_NS_LEN;
		lowsn_len = LOWSN_IPH_LEN + LOWSN_ICMPH_LEN + LOWSN_ND6_NS_LEN;
	}

	LOWSN_ICMP_BUF->icmpchksum = 0;
	LOWSN_ICMP_BUF->icmpchksum = ~ lowsn_icmp6chksum();
	DEBUG_STRING(DBG_INFO, "Sending NS to");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, "from");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, "with target address");
	PRINT6ADDR(tgt);
	DEBUG_STRING(DBG_INFO, "\r\n");
	return;

}
/*---------------------------------------------------------------------------*/
/**
 * \brief Process a Neighbor Advertisement
 *
 * we might have to send a pkt that had been buffered while address
 * resolution was performed (if we support buffering, see LOWSN_CONF_QUEUE_PKT)
 *
 * As per RFC 4861, on link layer that have addresses, TLLAO options MUST be
 * included when responding to multicast solicitations, SHOULD be included in
 * response to unicast (here we assume it is for now)
 *
 * NA can be received after sending NS for DAD, Address resolution or NUD. Can
 * be unsolicited as well.
 * It can trigger update of the state of the neighbor in the neighbor cache,
 * router in the router list.
 * If the NS was for DAD, it means DAD failed
 *
 */
void
lowsn_nd_na_input(void){
	uint8_t is_llchange;	//link layer address change
	uint8_t is_router;		
	uint8_t is_solicited;
	uint8_t is_override;

	DEBUG_STRING(DBG_INFO, "Received NA from");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, "to");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, "with target address");
	PRINT6ADDR((lowsn_ipaddr_t *) (&LOWSN_ND6_NA_BUF->tgtipaddr));
	DEBUG_STRING(DBG_INFO, "\r\n");

	/*
	* booleans. the three last one are not 0 or 1 but 0 or 0x80, 0x40, 0x20
	* but it works. Be careful though, do not use tests such as is_router == 1
	*/
	is_llchange = 0;
	is_router = ((LOWSN_ND6_NA_BUF->flagsreserved & LOWSN_ND6_NA_FLAG_ROUTER));
	is_solicited = ((LOWSN_ND6_NA_BUF->flagsreserved & LOWSN_ND6_NA_FLAG_SOLICITED));
	is_override = ((LOWSN_ND6_NA_BUF->flagsreserved & LOWSN_ND6_NA_FLAG_OVERRIDE));

#if LOWSN_CONF_IPV6_CHECKS
	if((LOWSN_IP_BUF->ttl != LOWSN_ND6_HOP_LIMIT) ||
			(LOWSN_ICMP_BUF->icode != 0) ||
			(lowsn_is_addr_mcast(&LOWSN_ND6_NA_BUF->tgtipaddr)) ||
			(is_solicited && lowsn_is_addr_mcast(&LOWSN_IP_BUF->destipaddr))) {
		DEBUG_STRING(DBG_INFO, "NA received is bad\r\n");
		goto discard;
	}
#endif /*LOWSN_CONF_IPV6_CHECKS */

	/* Options processing: we handle TLLAO, and must ignore others */
	nd6_opt_offset = LOWSN_ND6_NA_LEN;
	nd6_opt_llao = NULL;
	while(lowsn_l3_icmp_hdr_len + nd6_opt_offset < lowsn_len) {
#if LOWSN_CONF_IPV6_CHECKS
		if(LOWSN_ND6_OPT_HDR_BUF->len == 0) {
			DEBUG_STRING(DBG_INFO, "NA received is bad\r\n");
			goto discard;
		}
#endif /*LOWSN_CONF_IPV6_CHECKS */
		switch (LOWSN_ND6_OPT_HDR_BUF->type) {
			case LOWSN_ND6_OPT_TLLAO:
				nd6_opt_llao = (uint8_t *)LOWSN_ND6_OPT_HDR_BUF;
				break;
			default:
				DEBUG_STRING(DBG_INFO, "ND option not supported in NA\r\n");
			break;
		}
		nd6_opt_offset += (LOWSN_ND6_OPT_HDR_BUF->len << 3);
	}
	
	addr = lowsn_ds6_uaddr_lookup(&LOWSN_ND6_NA_BUF->tgtipaddr);
	/* Message processing, including TLLAO if any */
	if(addr != NULL) {
#if LOWSN_ND6_DEF_MAXDADNS > 0
		if(addr->state == ADDR_TENTATIVE) {
			lowsn_ds6_dad_failed(addr);
		}
#endif /*LOWSN_ND6_DEF_MAXDADNS > 0 */
		DEBUG_STRING(DBG_INFO, "NA received is bad\r\n");
		goto discard;
	}
	else {
		nbr = lowsn_ds6_nbr_lookup(&LOWSN_ND6_NA_BUF->tgtipaddr);
		if(nbr == NULL) {
			goto discard;
		}
		if(nd6_opt_llao != 0) {
			is_llchange = memcmp(&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET], (void *)(&nbr->lladdr), LOWSN_LLADDR_LEN);
		}
		
		if(nbr->state == NBR_INCOMPLETE) {
			if(nd6_opt_llao == NULL) {
				goto discard;
			}
			memcpy(&nbr->lladdr, &nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET], LOWSN_LLADDR_LEN);
			if(is_solicited) {
				/*cancel the send ns schedule*/
				nbr->state = NBR_REACHABLE;
				nbr->sendns = 0;
				nbr->nscount = 0;

				/* update reachable time is stored in ms */
				mstimer_set(&(nbr->reachable), lowsn_ds6_if.reachable_tminterval);

			}
			else {
				nbr->state = NBR_STALE;
			}
			nbr->isrouter = is_router;
		}
		else {
			if(!is_override && is_llchange) {
				if(nbr->state == NBR_REACHABLE) {
					nbr->state = NBR_STALE;
				}
				goto discard;
			}
			else {
				if(is_override || (!is_override && nd6_opt_llao != 0 && !is_llchange) || nd6_opt_llao == 0) {
					if(nd6_opt_llao != 0) {
						memcpy(&nbr->lladdr, &nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET], LOWSN_LLADDR_LEN);
					}
		  	    	if(is_solicited) {
						DEBUG_STRING(DBG_INFO, "End NURD Success\r\n");
						nbr->state = NBR_REACHABLE;
						nbr->sendns = 0;
						nbr->nscount = 0;
			
						/* reachable time is stored in ms */
						mstimer_set(&(nbr->reachable), lowsn_ds6_if.reachable_tminterval);
					}
					else {
						if(nd6_opt_llao != 0 && is_llchange) {
							nbr->state = NBR_STALE;
						}
					}
		  		}
	    	}
			
			if(nbr->isrouter && !is_router) {
				defrt = lowsn_ds6_defrt_lookup(&LOWSN_IP_BUF->srcipaddr);
				if(defrt != NULL) {
					lowsn_ds6_defrt_rm(defrt);
				}
			}
			nbr->isrouter = is_router;
		}
  	}
#if LOWSN_CONF_IPV6_QUEUE_PKT
	/**
	*The nbr is now reachable, check if we had queuing pakcet in it's queuing buffer.
	*--if YES		send the queuing packet out
	*--if NO		do nothing
	**/
	if(nbr->queue_buflen != 0) {
		DEBUG_STRING(DBG_INFO, "Sendint the queuing paket out\r\n");
		lowsn_len = nbr->queue_buflen;
		memcpy(LOWSN_IP_BUF, nbr->queue_buf, lowsn_len);
		nbr->queue_buflen = 0;
		nwk_pib.flags.bits.nwkpending = 1;
		return;
    }
#endif /*LOWSN_CONF_IPV6_QUEUE_PKT */
	/**
	  * here the processing of the valid NA message has been finished.
	  * Cancel the Send NS message schedule event(Address ressolution, neighbor unreachable detectation,
	  * and duplicate address detectaion), by cleaning the variable of "lowsn_ds6_if.sendrs".
	**/
//	nbr->sendns = 0;
//	nbr->nscount = 0;

discard:
  lowsn_len = 0;
  return;






}

/*---------------------------------------------------------------------------*/
/**
 * \brief Construct an Router Solicitation message
 *
 * src is chosen through the uip_netif_select_src function. If src is
 * unspecified  (i.e. we do not have a preferred address yet), then we do not
 * put a SLLAO option (MUST NOT in RFC 4861). Otherwise we do.
 *
 * RS message format,
 * possible option is SLLAO, MUST NOT be included if source = unspecified
 * SHOULD be included otherwise
 */
void lowsn_nd_rs_output(void){
	/*construct the ipv6 header (without any extention option header) and icmpv6 header*/
	LOWSN_IP_BUF->vtc = 0x60;
	LOWSN_IP_BUF->tcflow = 0;
	LOWSN_IP_BUF->flow = 0;
	LOWSN_IP_BUF->proto = LOWSN_PROTO_ICMP6;
	LOWSN_IP_BUF->ttl = LOWSN_ND6_HOP_LIMIT;
	lowsn_create_linklocal_allrouters_mcast(&LOWSN_IP_BUF->destipaddr);
	/*select the source ipv6 address*/
	ds6FindSrcIP(&LOWSN_IP_BUF->srcipaddr, &LOWSN_IP_BUF->destipaddr);
	LOWSN_ICMP_BUF->type = ICMP6_RS;
	LOWSN_ICMP_BUF->icode = 0;
	LOWSN_ND6_RS_BUF->reserved = 0;
	LOWSN_IP_BUF->len[0] = 0;       /* length will not be more than 255 */
	
	if(lowsn_is_addr_unspecified(&LOWSN_IP_BUF->srcipaddr)) {
		LOWSN_IP_BUF->len[1] = LOWSN_ICMPH_LEN + LOWSN_ND6_RS_LEN;
		lowsn_len = lowsn_l3_icmp_hdr_len + LOWSN_ND6_RS_LEN;
	}
	else {
		lowsn_len = lowsn_l3_icmp_hdr_len + LOWSN_ND6_RS_LEN + LOWSN_ND6_OPT_LLAO_LEN;
		LOWSN_IP_BUF->len[1] = LOWSN_ICMPH_LEN + LOWSN_ND6_RS_LEN + LOWSN_ND6_OPT_LLAO_LEN;
		/*creat the Source link-layer address option*/
		create_llao(&lowsn_buf[lowsn_l2_l3_icmp_hdr_len + LOWSN_ND6_RS_LEN], LOWSN_ND6_OPT_SLLAO);
	}
	/*calculate the checksum*/
	LOWSN_ICMP_BUF->icmpchksum = 0;
	LOWSN_ICMP_BUF->icmpchksum = ~ lowsn_icmp6chksum();

	DEBUG_STRING(DBG_INFO, "Sending RS to");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, "from");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, "\r\n");
	return;

}











/*---------------------------------------------------------------------------*/
/**
 *
 * \brief process a Router Advertisement
 *
 * - Possible actions when receiving a RA: add router to router list,
 *   recalculate reachable time, update link hop limit, update retrans timer.
 * - If MTU option: update MTU.
 * - If SLLAO option: update entry in neighbor cache
 * - If prefix option: start autoconf, add prefix to prefix list
 */
void
lowsn_nd_ra_input(void){

	uint16_t tmplen = 0;
	uint8_t* tmpp = NULL;
	
	DEBUG_STRING(DBG_INFO, "Received RA from");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, "to");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, "\r\n");

#if LOWSN_CONF_IPV6_CHECKS
	if(LOWSN_IP_BUF->ttl != LOWSN_ND6_HOP_LIMIT){
		DEBUG_STRING(DBG_INFO, "Discard RA for TTL\r\n");
		goto discard;
	}
	if(!lowsn_is_addr_link_local(&LOWSN_IP_BUF->srcipaddr)){
		DEBUG_STRING(DBG_INFO, "Discard RA for srcipaddr\r\n");
		goto discard;
	}
	if(LOWSN_ICMP_BUF->icode != 0){
		DEBUG_STRING(DBG_INFO, "Discard RA for ICMP Code\r\n");
		goto discard;
	}
	if(lowsn_len-40 < 16){
		DEBUG_STRING(DBG_INFO, "Discard RA for ICMP len\r\n");
		goto discard;
	}
	
	//check all the option len in the RA, MUST be greater than 0
	tmplen = LOWSN_IPH_LEN + lowsn_ext_len + LOWSN_ICMPH_LEN + LOWSN_ND6_RA_LEN;
	//tmpp = &lowsn_buf[LOWSN_LLH_LEN + LOWSN_IPH_LEN + lowsn_ext_len + LOWSN_ICMPH_LEN + LOWSN_ND6_RA_LEN];
	tmpp = &lowsn_buf[LOWSN_LLH_LEN + tmplen];
	while(tmplen < lowsn_len){
		if(*(tmpp+1) == 0){
			DEBUG_STRING(DBG_INFO, "Discard RA for Option len=0\r\n");
			goto discard;
		}
		else{
			tmplen = tmplen + (*(tmpp+1)<<3);
			tmpp = tmpp + (*(tmpp+1)<<3);
		}
	}
#endif /*LOWSN_CONF_IPV6_CHECKS */

	/*process the current hop limit feild of the RA message*/
	if(LOWSN_ND6_RA_BUF->cur_ttl != 0) {
		lowsn_ds6_if.cur_hop_limit = LOWSN_ND6_RA_BUF->cur_ttl;
		DEBUG_STRING(DBG_INFO, "lowsn_ds6_if.cur_hop_limit %u\r\n", lowsn_ds6_if.cur_hop_limit);
	}
	/*process the current Reachable Time feild of the RA message*/
	if(LOWSN_ND6_RA_BUF->reachable_time != 0) {
		if(lowsn_ds6_if.base_reachable_time !=lowsn_ntohl(LOWSN_ND6_RA_BUF->reachable_time)) {
			lowsn_ds6_if.base_reachable_time = lowsn_ntohl(LOWSN_ND6_RA_BUF->reachable_time);
			lowsn_ds6_if.reachable_tminterval = lowsn_ds6_compute_reachable_time();
		}
	}
	/*process the Retrans Timer feild of the RA message*/
	if(LOWSN_ND6_RA_BUF->retrans_timer != 0) {
		lowsn_ds6_if.retrans_tminterval= lowsn_ntohl(LOWSN_ND6_RA_BUF->retrans_timer);
	}

	/* Options processing */
	nd6_opt_offset = LOWSN_ND6_RA_LEN;
	while(lowsn_l3_icmp_hdr_len + nd6_opt_offset < lowsn_len) {
//		if(LOWSN_ND6_OPT_HDR_BUF->len == 0) {
//			DEBUG_STRING(DBG_INFO, "RA received is bad2\r\n");
//			goto discard;
//		}
		switch (LOWSN_ND6_OPT_HDR_BUF->type) {
			/*process the Source link-layer address option*/
			case LOWSN_ND6_OPT_SLLAO:
				DEBUG_STRING(DBG_INFO, "Processing SLLAO option in RA\r\n");
				nd6_opt_llao = (uint8_t *) LOWSN_ND6_OPT_HDR_BUF;
				nbr = lowsn_ds6_nbr_lookup(&LOWSN_IP_BUF->srcipaddr);
				if(nbr == NULL) {
					nbr = lowsn_ds6_nbr_add(&LOWSN_IP_BUF->srcipaddr,
						(lowsn_lladdr_t *)&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET],
						1, NBR_STALE);
				}
				else {
					if(nbr->state == NBR_INCOMPLETE) {
						memcpy(&nbr->lladdr, &nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET], LOWSN_LLADDR_LEN);
						nbr->state = NBR_STALE;
#if LOWSN_CONF_IPV6_QUEUE_PKT
						/**
						*The nbr has lladdr, check if we had queuing pakcet in it's queuing buffer.
						*--if YES		send the queuing packet out
						*--if NO		do nothing
						**/
						if(nbr->queue_buflen != 0) {
							DEBUG_STRING(DBG_INFO, "Sendint the queuing paket out\r\n");
							lowsn_len = nbr->queue_buflen;
							memcpy(LOWSN_IP_BUF, nbr->queue_buf, lowsn_len);
							nbr->queue_buflen = 0;
							nwk_pib.flags.bits.nwkpending = 1;
							return;
					    }
#endif /*LOWSN_CONF_IPV6_QUEUE_PKT */		
					}
					if(memcmp(&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET], &nbr->lladdr, LOWSN_LLADDR_LEN) != 0) {
						memcpy(&nbr->lladdr, &nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET], LOWSN_LLADDR_LEN);
						nbr->state = NBR_STALE;
					}
					nbr->isrouter = 1;
				}
			break;

			/*process the MTU option*/
			case LOWSN_ND6_OPT_MTU:
				DEBUG_STRING(DBG_INFO, "Processing MTU option in RA\r\n");
				lowsn_ds6_if.link_mtu = lowsn_ntohl(((lowsn_nd6_opt_mtu_t *) LOWSN_ND6_OPT_HDR_BUF)->mtu);
				if(lowsn_ds6_if.link_mtu < pmtu.size){
					pmtu.size = lowsn_ds6_if.link_mtu;
					stimer_set(&pmtu.age, PMTU_UPDATE_INTERVAL);
				}
			break;

			/*process the Prefix Information option*/
			case LOWSN_ND6_OPT_PREFIX_INFO:
				DEBUG_STRING(DBG_INFO, "Processing PREFIX option:");
				PRINT6ADDR(&((lowsn_nd6_opt_prefix_info_t *)LOWSN_ND6_OPT_HDR_BUF)->prefix);
				DEBUG_STRING(DBG_INFO, "\r\n");
				nd6_opt_prefix_info = (lowsn_nd6_opt_prefix_info_t *)LOWSN_ND6_OPT_HDR_BUF;
				if((lowsn_ntohl(nd6_opt_prefix_info->validlt) >= lowsn_ntohl(nd6_opt_prefix_info->preferredlt))
					    && (!lowsn_is_addr_link_local(&nd6_opt_prefix_info->prefix))) {
					/* on-link flag related processing */
					if(nd6_opt_prefix_info->flagsreserved1 & LOWSN_ND6_RA_FLAG_ONLINK) {
						prefix = lowsn_ds6_prefix_lookup(&nd6_opt_prefix_info->prefix, nd6_opt_prefix_info->preflen);
						if(prefix == NULL) {
							if(nd6_opt_prefix_info->validlt != 0) {
								if(nd6_opt_prefix_info->validlt != LOWSN_ND6_INFINITE_LIFETIME) {
									prefix = lowsn_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
													nd6_opt_prefix_info->preflen, lowsn_ntohl(nd6_opt_prefix_info->validlt));
								}
								else {
									prefix = lowsn_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
													nd6_opt_prefix_info->preflen, 0);
								}
							}
						}
						else {
							switch (nd6_opt_prefix_info->validlt) {
								case 0:
									lowsn_ds6_prefix_rm(prefix);
								break;
								
								case LOWSN_ND6_INFINITE_LIFETIME:
									prefix->isinfinite = 1;
								break;
								
								default:
									DEBUG_STRING(DBG_INFO, "Updating timer of prefix");
									PRINT6ADDR(&prefix->ipaddr);
									DEBUG_STRING(DBG_INFO, "new value %lu\r\n", lowsn_ntohl(nd6_opt_prefix_info->validlt));
									/*change the invalidtime and update the schedule timer and event*/
									stimer_set(&prefix->invalidtime, lowsn_ntohl(nd6_opt_prefix_info->validlt));
									prefix->isinfinite = 0;
								break;
							}
						}
					} /* End of on-link flag related processing */
					
					/* autonomous flag related processing */
					if((nd6_opt_prefix_info->flagsreserved1 & LOWSN_ND6_RA_FLAG_AUTONOMOUS)
							&& (nd6_opt_prefix_info->validlt != 0)
							&& (nd6_opt_prefix_info->preflen == LOWSN_DEFAULT_PREFIX_LEN)) {
	
						lowsn_ipaddr_copy(&ipaddr, &nd6_opt_prefix_info->prefix);		//copy the ipv6 address prefix
						lowsn_ds6_set_addr_iid(&ipaddr, &lowsn_lladdr);		//addr ipv6 address interface ID
						addr = lowsn_ds6_uaddr_lookup(&ipaddr);
						if((addr != NULL) && (addr->type == ADDR_AUTOCONF)) {
							if(nd6_opt_prefix_info->validlt != LOWSN_ND6_INFINITE_LIFETIME) {
              				/* The processing below is defined in RFC4862 section 5.5.3 e */
              					if((lowsn_ntohl(nd6_opt_prefix_info->validlt) > 2 * 60 * 60) ||
              					        (lowsn_ntohl(nd6_opt_prefix_info->validlt) > stimer_remaining(&addr->invalidtime))) {
              					    DEBUG_STRING(DBG_INFO, "Updating timer of address");
              					    PRINT6ADDR(&addr->ipaddr);
              					    DEBUG_STRING(DBG_INFO, "new value %lu\r\n",
              					    lowsn_ntohl(nd6_opt_prefix_info->validlt));
              					    stimer_set(&addr->invalidtime, lowsn_ntohl(nd6_opt_prefix_info->validlt));
              					}
								else {
              					    stimer_set(&addr->invalidtime, 2 * 60 * 60);
              					    DEBUG_STRING(DBG_INFO, "Updating timer of address ");
              					    PRINT6ADDR(&addr->ipaddr);
              					    DEBUG_STRING(DBG_INFO, "new value %lu\r\n", (unsigned long)(2 * 60 * 60));
              					}
              					addr->isinfinite = 0;
							}
							else {
								addr->isinfinite = 1;
							}
						}
						else {
							if(lowsn_ntohl(nd6_opt_prefix_info->validlt) == LOWSN_ND6_INFINITE_LIFETIME) {
								lowsn_ds6_uaddr_add(&ipaddr, 0, ADDR_AUTOCONF);
							}
							else {
								lowsn_ds6_uaddr_add(&ipaddr, lowsn_ntohl(nd6_opt_prefix_info->validlt),  ADDR_AUTOCONF);
								//lowsn_ds6_uaddr_add(&ipaddr, lowsn_ntohl(nd6_opt_prefix_info->preferredlt),  ADDR_AUTOCONF);
							}
						}
					} /* End of autonomous flag related processing */		
					
					/*delete the unicast address autouconfiged from the prefix*/
					if((nd6_opt_prefix_info->flagsreserved1 & LOWSN_ND6_RA_FLAG_AUTONOMOUS)
							&& (nd6_opt_prefix_info->preferredlt== 0)){
						lowsn_ipaddr_copy(&ipaddr, &nd6_opt_prefix_info->prefix);		//copy the ipv6 address prefix
						lowsn_ds6_set_addr_iid(&ipaddr, &lowsn_lladdr);		//addr ipv6 address interface ID
						addr = lowsn_ds6_uaddr_lookup(&ipaddr);
						if(addr!=NULL){
							lowsn_ds6_uaddr_rm(addr);
						}	
					}
				}
			break;
			
			default:
				DEBUG_STRING(DBG_INFO, "ND option not supported in RA");
			break;
		}
		nd6_opt_offset += (LOWSN_ND6_OPT_HDR_BUF->len << 3);
	}
	/*update the router lifetime*/
	defrt = lowsn_ds6_defrt_lookup(&LOWSN_IP_BUF->srcipaddr);
	if(LOWSN_ND6_RA_BUF->router_lifetime != 0) {
		if(nbr != NULL) {
			nbr->isrouter = 1;
		}
		if(defrt == NULL) {
			lowsn_ds6_defrt_add(&LOWSN_IP_BUF->srcipaddr, (unsigned long)(lowsn_ntohs(LOWSN_ND6_RA_BUF->router_lifetime)));
	    }
		else {
			stimer_set(&(defrt->invalidtime),  (unsigned long)(lowsn_ntohs(LOWSN_ND6_RA_BUF->router_lifetime)));
	    }
	}
	else {		//the route lifetime = 0
		if(defrt != NULL) {
//			int j;
			/*update the destination cache table*/	
			lowsn_ds6_dest_update(&defrt->ipaddr);
//			for(j=0; j<LOWSN_DS6_DEST_NB; j++){
//				if(lowsn_ds6_dest_cache[j].isused &&
//							(lowsn_ds6_ipcmp(&lowsn_ds6_dest_cache[j].nexthop, &defrt->ipaddr))){
//					lowsn_ds6_dest_rm(&lowsn_ds6_dest_cache[j]);
//				}
//			}			
			/*discard the default router*/
			lowsn_ds6_defrt_rm(defrt);
		}
	}
	
	/**
	  * here the processing of the valid RA message has been finished.
	  * Cancel the Send RS message schedule event by cleaning the variable of "lowsn_ds6_if.sendrs".
	**/
	//lowsn_ds6_if.sendrs = 0;	
	//lowsn_da6_schedule_clean();
	//DEBUG_STRING(DBG_INFO, "Stop send RS\r\n");
	lowsn_ds6_if.sendrs = 0;
	lowsn_ds6_if.rscount = 0;

#if LOWSN_CONF_IPV6_QUEUE_PKT
	  /* If the nbr just became reachable (e.g. it was in NBR_INCOMPLETE state
	   * and we got a SLLAO), check if we had buffered a pkt for it */
	  /*  if((nbr != NULL) && (nbr->queue_buf_len != 0)) {
	    uip_len = nbr->queue_buf_len;
	    memcpy(LOWSN_IP_BUF, nbr->queue_buf, uip_len);
	    nbr->queue_buf_len = 0;
	    return;
	    }*/


#endif /*LOWSN_CONF_IPV6_QUEUE_PKT */

discard:
  lowsn_len = 0;
  return;

}




/*---------------------------------------------------------------------------*/
/**
 * \brief Process a router solicitaion and send a router advertisment
 *
 */
void
lowsn_nd_rs_input(void){



}



/*----------------------------------------------------------------------------*/
/**
  * process the redirect message and update the destination cache table
**/
void lowsn_nd_red_input(void){
	uint16_t tmplen = 0;
	uint8_t* tmpp = NULL;
	lowsn_ds6_dest_t* tmpdest=NULL;
	uint8_t tgteqdest=0;
	
	DEBUG_STRING(DBG_INFO, "Received RED from");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, "to");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, "\r\n");

#if LOWSN_CONF_IPV6_CHECKS
	if(LOWSN_IP_BUF->ttl != LOWSN_ND6_HOP_LIMIT){
		DEBUG_STRING(DBG_INFO, "Discard RED for TTL\r\n");
		goto discard;
	}
	if(!lowsn_is_addr_link_local(&LOWSN_IP_BUF->srcipaddr)){
		DEBUG_STRING(DBG_INFO, "Discard RED for srcipaddr\r\n");
		goto discard;
	}
	if(LOWSN_ICMP_BUF->icode != 0){
		DEBUG_STRING(DBG_INFO, "Discard RED for ICMP Code\r\n");
		goto discard;
	}
	if(lowsn_len-40 < 40){
		DEBUG_STRING(DBG_INFO, "Discard RED for ICMP len\r\n");
		goto discard;
	}
	if(lowsn_is_addr_mcast(&LOWSN_ND6_RED_BUF->destipaddr)){
		DEBUG_STRING(DBG_INFO, "Discard RED for ICMP Destination Address\r\n");
		goto discard;
	}
	if(!lowsn_is_addr_link_local(&LOWSN_ND6_RED_BUF->tgtipaddr)&&
			!lowsn_ds6_ipcmp(&LOWSN_ND6_RED_BUF->tgtipaddr, &LOWSN_ND6_RED_BUF->destipaddr)){
		DEBUG_STRING(DBG_INFO, "Discard RED for Target Address\r\n");
		goto discard;
	}
	tmpdest = lowsn_ds6_dest_lookup(&LOWSN_ND6_RED_BUF->destipaddr);
	if((tmpdest!=NULL) && !lowsn_ds6_ipcmp(&LOWSN_IP_BUF->srcipaddr, &tmpdest->nexthop)){
		DEBUG_STRING(DBG_INFO, "Discard RED for ip srcaddr\r\n");
		goto discard;
	}
	
	//check all the option len in the RA, MUST be greater than 0
	tmplen = LOWSN_IPH_LEN + lowsn_ext_len + LOWSN_ICMPH_LEN + LOWSN_ND6_RED_LEN;
	tmpp = &lowsn_buf[LOWSN_LLH_LEN + tmplen];
	while(tmplen < lowsn_len){
		if(*(tmpp+1) == 0){
			DEBUG_STRING(DBG_INFO, "Discard RED for Option len=0\r\n");
			goto discard;
		}
		else{
			tmplen = tmplen + (*(tmpp+1)<<3);
			tmpp = tmpp + (*(tmpp+1)<<3);
		}
	}
#endif /*LOWSN_CONF_IPV6_CHECKS */
	//update the destinaton cache table
	dest = lowsn_ds6_dest_lookup(&LOWSN_ND6_RED_BUF->destipaddr);
	if(dest == NULL){
		lowsn_ds6_dest_add(&LOWSN_ND6_RED_BUF->destipaddr, &LOWSN_ND6_RED_BUF->tgtipaddr);
	}
	else{
		lowsn_ipaddr_copy(&dest->nexthop, &LOWSN_ND6_RED_BUF->tgtipaddr);
	}
	
	//Process the options include the redirect message
	nd6_opt_offset = LOWSN_ND6_RED_LEN;
	nd6_opt_llao = NULL;
	while(lowsn_l3_icmp_hdr_len + nd6_opt_offset < lowsn_len){
		switch (LOWSN_ND6_OPT_HDR_BUF->type) {
			//process the target link-layer address
			case LOWSN_ND6_OPT_TLLAO:
				DEBUG_STRING(DBG_INFO, "process the target link-layer address\r\n");
				tgteqdest = lowsn_ds6_ipcmp(&LOWSN_ND6_RED_BUF->destipaddr, &LOWSN_ND6_RED_BUF->tgtipaddr);
				nd6_opt_llao = (uint8_t *) LOWSN_ND6_OPT_HDR_BUF;
				nbr = lowsn_ds6_nbr_lookup(&LOWSN_ND6_RED_BUF->tgtipaddr);
				if(nbr!=NULL){
					if(lowsn_ds6_lladdr_cmp(&nbr->lladdr, (lowsn_lladdr_t *)&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET])){
						if(tgteqdest != 1){
							nbr->isrouter = 1;
						}
					}
					else{
						lowsn_lladdr_copy(&nbr->lladdr, (lowsn_lladdr_t *)&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET]);
						nbr->state = NBR_STALE;
						if(tgteqdest != 1){
							nbr->isrouter = 1;
						}					
					}
				}
				else{
					nbr = lowsn_ds6_nbr_add(&LOWSN_ND6_RED_BUF->tgtipaddr,
								(lowsn_lladdr_t *)&nd6_opt_llao[LOWSN_ND6_OPT_DATA_OFFSET], 0, NBR_STALE);
					if(tgteqdest != 1){
						nbr->isrouter = 1;
					}
				}
				break;
			//process the redirect header option	
			case LOWSN_ND6_OPT_REDIRECTED_HDR:
				DEBUG_STRING(DBG_INFO, "process the redirect header option\r\n");
				break;
				
			default:
				DEBUG_STRING(DBG_INFO, "ND option not supported in RED\r\n");
				break;

		}
		nd6_opt_offset += (LOWSN_ND6_OPT_HDR_BUF->len << 3);
	}
#if LOWSN_CONF_IPV6_QUEUE_PKT
	/**
	*The nbr is now reachable, check if we had queuing pakcet in it's queuing buffer.
	*--if YES		send the queuing packet out
	*--if NO		do nothing
	**/
	if(nbr->queue_buflen != 0) {
		DEBUG_STRING(DBG_INFO, "Sendint the queuing paket out\r\n");
		lowsn_len = nbr->queue_buflen;
		memcpy(LOWSN_IP_BUF, nbr->queue_buf, lowsn_len);
		nbr->queue_buflen = 0;
		nwk_pib.flags.bits.nwkpending = 1;
		return;
	 }
#endif /*LOWSN_CONF_IPV6_QUEUE_PKT */	
discard:
	lowsn_len = 0;
	return;
}



#endif



