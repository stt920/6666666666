/*********************************************************************
    文件名：icmpv6.c
    作  者：ji shanyang
    版  本：v1.0.0
    日  期：2012.12
    描  述：ICMPv6 protocol
*********************************************************************/
#include <string.h>

#include "compiler.h"
#include "6lowsn_config.h"         //user configurations
#include "6lowsn_common_types.h"   //types common acrosss most files
#include "ieee_lrwpan_defs.h"
#include "ds.h"
#include "console.h"
#include "debug.h"
#include "memalloc.h"
#include "neighbor.h"
#include "icmpv6.h"
#include "nd.h"
#include "hal.h"
#include "halStack.h"
#include "phy.h"
#include "mac.h"
#include "adp.h"
#include "nwk.h"
#include "mp.h"
#include "aps.h"
#include "evboard.h"



#define LOWSN_IP_BUF                ((lowsn_ip_hdr_t *)&lowsn_buf[LOWSN_LLH_LEN])
#define LOWSN_ICMP_BUF            ((lowsn_icmp6_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_ICMP6_ERROR_BUF  ((lowsn_icmp6_error_t*)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len])
#define LOWSN_EXT_BUF              ((lowsn_ext_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_FIRST_EXT_BUF        ((lowsn_ext_hdr_t *)&lowsn_buf[LOWSN_LLIPH_LEN])

/** \brief temporary IP address */

extern lowsn_ds6_pmtu_t pmtu;

#if 0

/*---------------------------------------------------------------- */
/** \
 * brief Process an echo request
 *
 * Perform a few checks, then send an Echo reply. The reply is
 * built here (with full ipv6 header).
  */
void
lowsn_icmp6_echo_request_input(void)
{
  /*
   * we send an echo reply. It is trivial if there was no extension
   * headers in the request otherwise we need to remove the extension
   * headers and change a few fields
   */
  DEBUG_STRING(DBG_INFO, "Received Echo Request from");
  PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
  DEBUG_STRING(DBG_INFO, "to");
  PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
  DEBUG_STRING(DBG_INFO, "\r\n");
  /* IP header */
  LOWSN_IP_BUF->ttl = lowsn_ds6_if.cur_hop_limit;

  if(lowsn_is_addr_mcast(&LOWSN_IP_BUF->destipaddr)){
    lowsn_ipaddr_copy(&LOWSN_IP_BUF->destipaddr, &LOWSN_IP_BUF->srcipaddr);
    ds6FindSrcIP(&LOWSN_IP_BUF->srcipaddr, &LOWSN_IP_BUF->destipaddr);
  } else {
    lowsn_ipaddr_copy(&tmp_ipaddr, &LOWSN_IP_BUF->srcipaddr);
    lowsn_ipaddr_copy(&LOWSN_IP_BUF->srcipaddr, &LOWSN_IP_BUF->destipaddr);
    lowsn_ipaddr_copy(&LOWSN_IP_BUF->destipaddr, &tmp_ipaddr);
  }

  if(lowsn_ext_len > 0) {
      /* If there were extension headers*/
      LOWSN_IP_BUF->proto = LOWSN_PROTO_ICMP6;
      lowsn_len -= lowsn_ext_len;
      LOWSN_IP_BUF->len[0] = ((lowsn_len - LOWSN_IPH_LEN) >> 8);
      LOWSN_IP_BUF->len[1] = ((lowsn_len - LOWSN_IPH_LEN) & 0xff);
      /* move the echo request payload (starting after the icmp header)
       * to the new location in the reply.
       * The shift is equal to the length of the extension headers present
       * Note: LOWSN_ICMP_BUF still points to the echo request at this stage
       */
      memmove((uint8_t *)LOWSN_ICMP_BUF + LOWSN_ICMPH_LEN - lowsn_ext_len,
              (uint8_t *)LOWSN_ICMP_BUF + LOWSN_ICMPH_LEN,
              (lowsn_len - LOWSN_IPH_LEN - LOWSN_ICMPH_LEN));
      lowsn_ext_len = 0;
  }

  /* Below is important for the correctness of LOWSN_ICMP_BUF and the
   * checksum
   */
  /* Note: now LOWSN_ICMP_BUF points to the beginning of the echo reply */
  LOWSN_ICMP_BUF->type = ICMP6_ECHO_REPLY;
  LOWSN_ICMP_BUF->icode = 0;
  LOWSN_ICMP_BUF->icmpchksum = 0;
  LOWSN_ICMP_BUF->icmpchksum = ~ lowsn_icmp6chksum();
  DEBUG_STRING(DBG_INFO, "Sending Echo Reply to");
  PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
  DEBUG_STRING(DBG_INFO, "from");
  PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
  DEBUG_STRING(DBG_INFO, "\r\n");
  return;

}


/*---------------------------------------------------------------- */
/**
 * \brief recieve an icmpv6 packet too big error message
 */
void
lowsn_icmp6_pktbigerr_input(void){
	DEBUG_STRING(DBG_INFO, "Received packet too big error message from");
	PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
	DEBUG_STRING(DBG_INFO, "to");
	PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
	DEBUG_STRING(DBG_INFO, "\r\n");
	if(pmtu.size > lowsn_ntohl(LOWSN_ICMP6_ERROR_BUF->param)){
		pmtu.size = lowsn_ntohl(LOWSN_ICMP6_ERROR_BUF->param);
		if(pmtu.size < LOWSN_IPV6_MIN_LINK_MTU){
			pmtu.size = LOWSN_IPV6_MIN_LINK_MTU;
		}
	}
	DEBUG_STRING(DBG_INFO, "PMTU:%d\r\n",pmtu.size);
	stimer_set(&pmtu.age, PMTU_UPDATE_INTERVAL);
	lowsn_len = 0;
}

/*---------------------------------------------------------------- */
/**
 * \brief construct an icmpv6 error message
 * \param type type of the error message
 * \param code of the error message
 * \param type 32 bit parameter of the error message, semantic depends on error
 */
void
lowsn_icmp6_error_output(uint8_t type, uint8_t code, uint32_t param)
{
 /* check if originating packet is not an ICMP error*/
  if (lowsn_ext_len) {
    if((LOWSN_EXT_BUF->next == LOWSN_PROTO_ICMP6) ){
		lowsn_ext_len += (LOWSN_EXT_BUF->len << 3) + 8;
		if(LOWSN_ICMP_BUF->type < 128){
	      lowsn_len = 0;
	      return;
		}
    }
  } else {
    if((LOWSN_IP_BUF->proto == LOWSN_PROTO_ICMP6) &&
			(lowsn_len > 44) && (LOWSN_ICMP_BUF->type < 128)){
      lowsn_len = 0;
      return;
    }
  }

  lowsn_ext_len = 0;

  /* remember data of original packet before shifting */
  lowsn_ipaddr_copy(&tmp_ipaddr, &LOWSN_IP_BUF->destipaddr);

  lowsn_len += LOWSN_IPICMPH_LEN + LOWSN_ICMP6_ERROR_LEN;

  //if(lowsn_len > LOWSN_LINK_MTU)
  		//lowsn_len = LOWSN_LINK_MTU;
  if(lowsn_len > LOWSN_IPV6_MIN_LINK_MTU)
    lowsn_len = LOWSN_IPV6_MIN_LINK_MTU;

  memmove((uint8_t *)LOWSN_ICMP6_ERROR_BUF + lowsn_ext_len + LOWSN_ICMP6_ERROR_LEN,
          (void *)LOWSN_IP_BUF, lowsn_len - LOWSN_IPICMPH_LEN - lowsn_ext_len - LOWSN_ICMP6_ERROR_LEN);

  LOWSN_IP_BUF->vtc = 0x60;
  LOWSN_IP_BUF->tcflow = 0;
  LOWSN_IP_BUF->flow = 0;
  if (lowsn_ext_len) {
    LOWSN_FIRST_EXT_BUF->next = LOWSN_PROTO_ICMP6;
  } else {
    LOWSN_IP_BUF->proto = LOWSN_PROTO_ICMP6;
  }
  LOWSN_IP_BUF->ttl = lowsn_ds6_if.cur_hop_limit;

  /* the source should not be unspecified nor multicast, the check for
     multicast is done in lowsn_process */
  if(lowsn_is_addr_unspecified(&LOWSN_IP_BUF->srcipaddr)){
    lowsn_len = 0;
    return;
  }

  lowsn_ipaddr_copy(&LOWSN_IP_BUF->destipaddr, &LOWSN_IP_BUF->srcipaddr);

  if(lowsn_is_addr_mcast(&tmp_ipaddr)){
    if(type == ICMP6_PARAM_PROB && code == ICMP6_PARAMPROB_OPTION){
      ds6FindSrcIP(&LOWSN_IP_BUF->srcipaddr, &tmp_ipaddr);
    } else {
      lowsn_len = 0;
      return;
    }
  } else {
#ifdef LOWSN_COORDINATOR
    /* need to pick a source that corresponds to this node */
    ds6FindSrcIP(&LOWSN_IP_BUF->srcipaddr, &tmp_ipaddr);
#else
    lowsn_ipaddr_copy(&LOWSN_IP_BUF->srcipaddr, &tmp_ipaddr);
#endif
  }

  LOWSN_ICMP_BUF->type = type;
  LOWSN_ICMP_BUF->icode = code;
  LOWSN_ICMP6_ERROR_BUF->param = lowsn_htonl(param);
  LOWSN_IP_BUF->len[0] = ((lowsn_len - LOWSN_IPH_LEN) >> 8);
  LOWSN_IP_BUF->len[1] = ((lowsn_len - LOWSN_IPH_LEN) & 0xff);
  LOWSN_ICMP_BUF->icmpchksum = 0;
  LOWSN_ICMP_BUF->icmpchksum = ~ lowsn_icmp6chksum();
  DEBUG_STRING(DBG_INFO, "Sending ICMPv6 ERROR message to");
  PRINT6ADDR(&LOWSN_IP_BUF->destipaddr);
  DEBUG_STRING(DBG_INFO, "from");
  PRINT6ADDR(&LOWSN_IP_BUF->srcipaddr);
  DEBUG_STRING(DBG_INFO, "\r\n");
  return;

}

#endif

void icmpCommonFmt(IPADDR *dest)
{

	a_nwk_tx_data.Version = LOWSN_IP6_VER;
	a_nwk_tx_data.TrafficClass=0x00;
	a_nwk_tx_data.FlowLabel = 0x00;
	a_nwk_tx_data.PayloadLength = phy_pib.currentTxFlen;
	a_nwk_tx_data.NextHeader = LOWSN_PROTO_ICMP6;
	a_nwk_tx_data.HopLimit = lowsn_ds6_if.cur_hop_limit;
	memcpy(&a_nwk_tx_data.DstAddress, dest, sizeof(*dest));
	ds6FindSrcIP(&a_nwk_tx_data.SrcAddress, &a_nwk_tx_data.DstAddress);
}



void icmpFmtEchoRequest(UINT16 plen)
{
	UINT16 i;
	BYTE j;
	
	phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];

	// ping命令负载中的每个字节依次填充0,1,2,3,...,最大填充到255，然后再返回0继续填充。
	for (i=0, j=0; i<plen; ++i, ++j) {
		phy_pib.currentTxFrm--;
		*phy_pib.currentTxFrm = j;
		if (j >= 0xFF) {
			j = 0;
		}	
	}	
		
	// Sequence Number
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;

	// Identifier
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE)halGetRandomByte();
	nwk_pib.lastPing.Identifier = (UINT16)(*phy_pib.currentTxFrm);
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x66;   // special MSB of 6lowsn stack
	nwk_pib.lastPing.Identifier += 0x6600;

	//checksum is added by icmpCommonFmt, use 0 here.
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;	

	// code
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0;

	//type
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = ICMP6_ECHO_REQUEST;

	phy_pib.currentTxFlen = plen + 4 + LOWSN_ICMPH_LEN;

	// 记录本次echo request的信息，以便收到reply时核对
	nwk_pib.lastPing.SequenceNumber=0x0;
	nwk_pib.lastPing.Plen= a_nwk_service.args.tx_ping6.plen;
	
  	lowsn_len = LOWSN_IPH_LEN + LOWSN_ICMPH_LEN + 4 + plen;

}



void icmpParseEchoReply(void){
	
	BYTE *ptr;
	UINT16 i;

	// 应首先检查REPLY的长度是否跟发出去的一致，但需要MAC头部长度域跨越适配层传上来，
	// 以后再添加

	ptr = a_nwk_rx_data.orgpkt.data + a_nwk_rx_data.pload_offset;

	// check Identifier
	ptr = ptr + 4;
	i = (((UINT16)*ptr) << 8);
	ptr++;
	i += *ptr;
	ptr++;

	if (i != nwk_pib.lastPing.Identifier) {
		DEBUG_STRING( DBG_INFO, "ICMP: Ping Reply, not our identifier.\n");
		return;
	}	
		
	// check Sequence Number
	i = (((UINT16)*ptr) << 8);
	ptr++;
	i += *ptr;
	ptr++;

	if (i != nwk_pib.lastPing.SequenceNumber) {
		DEBUG_STRING( DBG_INFO, "ICMP: Ping Reply, not our sequence number.\n");
		return;
	}	

	// check typical payload
	// 检查负载时须先确定负载长度是否至少有2个字节，因为负载长度未检查，暂不检查
	//ptr = ptr + 2;
	//if (((*ptr) != 0) && ((*(ptr+1)) != 0) ) {
	//	DEBUG_STRING( DBG_INFO, "ICMP: Ping Reply, not our payload.\n");
	//	return;
	//}	

	//indicate that the ping was successful
  	nwk_pib.flags.bits.nwkIsGetEchoReply = 1;
  	DEBUG_STRING(DBG_INFO,"ICMP:Received good echo reply!\n");

}




// 对于其它节点发来的请求，reply函数首先完成对于收到ICMP包格式的审查，然后从收到的包
// 中提取有用信息，接着用这些信息构造发送的relay的内容
// 一个函数内实际上完成了接收包, 填充发送两个任务

void icmpFmtEchoReply(void){

	BYTE *ptr;
	UINT16 identifier, seqnum, plen, i;

	// 假定状态机已经保证收到的帧是对的，不再检查。如果还需检查，在这里添加

	ptr = a_nwk_rx_data.orgpkt.data + a_nwk_rx_data.pload_offset;

	// get Identifier
	ptr = ptr + 4;
	identifier = ((UINT16)(*ptr)) << 8 | (*(ptr+1));

	// get Sequence Number
	ptr = ptr + 2;
	seqnum = ((UINT16)(*ptr)) << 8 | (*(ptr+1));

	// payload
	ptr = ptr + 2;
	plen = a_nwk_rx_data.PayloadLength - LOWSN_ICMPH_LEN - 4;

	// form echo reply
	phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];

	// copy payload from echo request
	for (i=0; i<plen; i++) {
		phy_pib.currentTxFrm--;
		*phy_pib.currentTxFrm = *(ptr+plen-i-1);
	}	
	
	// add Sequence Number
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (seqnum);
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (seqnum >> 8);	

	// add Identifier
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (identifier);
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (identifier >> 8);	

	//checksum is added by icmpCommonFmt, use 0 here.
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0x0;	

	// add code
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = 0;

	//type
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = ICMP6_ECHO_REPLY;

	phy_pib.currentTxFlen = plen + 4 + LOWSN_ICMPH_LEN;

  	lowsn_len = LOWSN_IPH_LEN + LOWSN_ICMPH_LEN + 4 + plen;


}














