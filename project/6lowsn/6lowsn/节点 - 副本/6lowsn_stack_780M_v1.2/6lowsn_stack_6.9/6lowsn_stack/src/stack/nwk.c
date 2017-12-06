
/******************************************************************************************************
*
* 文 件 名：nwk.c
*
* 文件描述：
*
* 创 建 者：Ji Shanyang
*
* 当前版本： 0.50
*
* 修 改 者：
*
* 修改历史：



********************************************************************************************************/
#include <stdlib.h>
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
#include "nd.h"
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
#include "slip.h"
#endif
#endif



static UINT32 nwk_utility_timer;   //utility timer

/** \name Buffer variables
 ** @{ */
/** Packet buffer for incoming and outgoing packets */
lowsn_buf_t lowsn_aligned_buf;
uint16_t lowsn_len;	//the packet length in the lowsn_aligned_buf
/*define the fragment buffer*/
lowsn_ds6_fragbuf_t lowsn_fragbuf;
/**define the reassemble buffer*/
lowsn_ds6_reassbuf_t lowsn_reassbuf;
/**define the swap buffer*/
lowsn_swap_buf_t lowsn_swap_buf;
/** @} */



/** \name UDP connections and pointer*/
/** @{ */
/**define the UDP connections*/
lowsn_udp_conn_t *lowsn_udp_conn;
lowsn_udp_conn_t lowsn_udp_conns[LOWSN_UDP_CONNS];
/** @} */

/** \network layer  Finite State Machine(FSM) variables*/
/** @{ */
lowsn_ipv6_state_t nwkState;
lowsn_ipv6_rxstate_t nwkRxState;
lowsn_ipv6_service_t a_nwk_service;
lowsn_ipv6_pib_t nwk_pib;
NWK_TX_DATA a_nwk_tx_data;
NWK_RX_DATA a_nwk_rx_data;
/** @} */

#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
// 用于恢复压缩的IP头使用
BYTE ip_header_buf[LOWSN_IPH_LEN]; 
#endif
#endif

extern systime_s_t internalclock;
extern systime_ms_t internalclock_ms;
extern uint8_t lladdr_dadfail;

/**
  * @{\* define the PMUT (used by network layer)
**/
lowsn_ds6_pmtu_t pmtu;
/**
  * @}\*
**/


/*---------------------------------------------------------------------------*/
/* Buffers                                                                   */
/*---------------------------------------------------------------------------*/
/** \name Buffer defines
 *  @{
 */
#define FBUF                            					 ((lowsn_ip_hdr_t *)&lowsn_reassbuf.buf[0])
#define LOWSN_IP_BUF                          ((lowsn_ip_hdr_t *)&lowsn_buf[LOWSN_LLH_LEN])
#define LOWSN_ICMP_BUF                      ((lowsn_icmp6_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_ECHO_BUF				 ((lowsn_echo_hdr_t *)&lowsn_buf[LOWSN_LLH_LEN + LOWSN_IPH_LEN+LOWSN_ICMPH_LEN])
//#define LOWSN_UDP_BUF                        ((lowsn_udp_hdr_t *)&lowsn_buf[LOWSN_LLH_LEN + LOWSN_IPH_LEN])
#define LOWSN_UDP_BUF                        ((lowsn_udp_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_EXT_BUF                        ((lowsn_ext_hdr_t*)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_ROUTING_BUF                ((lowsn_routing_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_FRAG_BUF                      ((lowsn_frag_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_HBHO_BUF                      ((lowsn_hbho_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_DESTO_BUF                    ((lowsn_desto_hdr_t *)&lowsn_buf[lowsn_l2_l3_hdr_len])
#define LOWSN_EXT_HDR_OPT_BUF            ((lowsn_ext_hdr_opt_t *)&lowsn_buf[lowsn_l2_l3_hdr_len + lowsn_ext_opt_offset])
#define LOWSN_EXT_HDR_OPT_PADN_BUF  ((lowsn_ext_hdr_opt_padn_t *)&lowsn_buf[lowsn_l2_l3_hdr_len + lowsn_ext_opt_offset])
#define LOWSN_ICMP6_ERROR_BUF            ((lowsn_icmp6_error_t *)&lowsn_buf[lowsn_l2_l3_icmp_hdr_len])
/** @} */


/*---------------------------------------------------------------------------*/
/** @{ \name Layer 3 variables */
/*---------------------------------------------------------------------------*/
/**
 * \brief Type of the next header in IPv6 header or extension headers
 *
 * Can be the next header field in the IPv6 header or in an extension header.
 * When doing fragment reassembly, we must change the value of the next header
 * field in the header before the fragmentation header, hence we need a pointer
 * to this field.
 */
uint8_t *lowsn_next_hdr;
/** \brief bitmap we use to record which IPv6 headers we have already seen */
uint8_t lowsn_ext_bitmap = 0;
/**
 * \brief length of the extension headers read. updated each time we process
 * a header
 */
uint8_t lowsn_ext_len = 0;
/** \brief length of the header options read */
uint8_t lowsn_ext_opt_offset = 0;
/** @} */



/*******************************2014-06-26***********************************

Flag_G:    最高位为1: 源IPv6地址未被提供，采用默认的；
		最高位为0: 源IPv6地址被作为参数提供

***************************************************************************/

int 
Nwk_Forward_Outside(UINT8 Flag_G,
						IPADDR SrcIpAddr_G,
						IPADDR DstIpAddr_G,
						UINT16 RemotePort_G,
						UINT16 LocalPort_G,
						BYTE * Pload_G,
						BYTE Length_G,
						BYTE NextHeader_G)

{
  BYTE tempTest[128]={0};
  BYTE tempTest1[128]={0};
	UINT16 i,j;
	UINT16 checksum;

	//int len;
	

//int result;
//BYTE *ptr111;
//char buf[50];

//emset(buf,0,sizeof(buf));


	//printf("\n 11111111111111111111111\n");
	
	if( ((Flag_G & 0x7F)== 1) && (DstIpAddr_G.u8[0] == 0x20) && (DstIpAddr_G.u8[1] == 0x02) ) 
	{
		i=0;
		tempTest[i++] = 0x60;                          
		tempTest[i++] = 0;
		tempTest[i++] = 0;
		tempTest[i++] = 0;
		tempTest[i++] = (Length_G+8)>> 8;
		tempTest[i++] = Length_G+8;
		tempTest[i++] = LOWSN_PROTO_UDP;
		tempTest[i++] = 0x40;
		
	 if( (Flag_G >>7) == 1 )
		 {
				tempTest[i++] =  0x2f;
				tempTest[i++] =  0xfe;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0;
				tempTest[i++] =  0x60;
		 }
	else
		{
      for( j=0;j<16;j++)
					tempTest[i++] = SrcIpAddr_G.u8[j];
		}
	  
  for( j=0;j<16;j++ )
		{
			tempTest[i++] = DstIpAddr_G.u8[j];
		}
 

		/* add UDP header*/


		/*Source Port*/

			tempTest[i++] = (LocalPort_G)>> 8;

			tempTest[i++] = (BYTE)(LocalPort_G);

		/*Destination Port*/

			tempTest[i++] = (RemotePort_G)>> 8;

			tempTest[i++] = (BYTE)(RemotePort_G);
		
		/*Length
		// Length is the length in octets of this user datagram including this header and the data. 	*/			

			tempTest[i++] = (BYTE)(((UINT16)(Length_G+8)) >> 8);

			tempTest[i++] = (BYTE)((UINT16)(Length_G+8));

		/* Checksum*/
		 tempTest[i++] = 0;
		 tempTest[i++] = 0;

for( j=0;j<Length_G;j++)
	{
		 tempTest[i++] = *(Pload_G+j);
	}
		
		 tempTest[i] = '\0';
		
		checksum = 0;
if ( NextHeader_G == LOWSN_PROTO_UDP )  
		{

			#if LOWSN_CONF_UDP_CHECK
			checksum = ~(lowsn_udpchksum(tempTest, Length_G+8+40));
			if(checksum == 0) 
			{
				checksum = 0xffff;
			}
	    tempTest[47] = (BYTE)checksum;
			tempTest[46] = (BYTE)(checksum >> 8);
			#endif
		}
		//slipGrabTxLock();
              slipSend(tempTest, Length_G+8+40,0);	
		slipReleaseTxLock();
		//wsn_len = 0;
		return 0;
}





	
	else if( ((Flag_G & 0x7F) == 2) && (DstIpAddr_G.u8[0] == 0xfe) && (DstIpAddr_G.u8[1] == 0x80) )
	{
		i=0;
		tempTest1[i++] = 0x60;
		tempTest1[i++] = 0;
		tempTest1[i++] = 0;
		tempTest1[i++] = 0;
		tempTest1[i++] = (Length_G+8)>> 8;
		tempTest1[i++] = Length_G+8;
		tempTest1[i++] = LOWSN_PROTO_UDP;
		tempTest1[i++] = 0x40;
		
	  if( (Flag_G >>7) == 1 )
		 {
				tempTest1[i++] =  0x2f;
				tempTest1[i++] =  0xfe;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0;
				tempTest1[i++] =  0x60;			
		 }
	 else
		{
			  for( j=0;j<16;j++ )
				  tempTest1[i++] = SrcIpAddr_G.u8[j];
		}		
	 
	for( j=0;j<16;j++)
		{
			  tempTest1[i++] = DstIpAddr_G.u8[j];
		}
		/* add UDP header*/

		/*Source Port*/
		tempTest1[i++] = (LocalPort_G)>> 8;
		tempTest1[i++] = (BYTE)(LocalPort_G);
		/*Destination Port*/
		tempTest1[i++] = (RemotePort_G)>> 8;
		tempTest1[i++] = (BYTE)(RemotePort_G);	
		/*Length
		// Length is the length in octets of this user datagram including this header and the data. 	*/			
		tempTest1[i++] = (BYTE)(((UINT16)(Length_G+8)) >> 8);
		tempTest1[i++] = (BYTE)((UINT16)(Length_G+8));
		/* Checksum*/
		tempTest1[i++] = 0;
		tempTest1[i++] = 0;
		for( j=0;j<Length_G;j++)
		{
			tempTest1[i++] = *(Pload_G+j);
		}
		
		tempTest1[i] = '\0';
		
		checksum = 0;
		if ( NextHeader_G == LOWSN_PROTO_UDP )  
		{

			#if LOWSN_CONF_UDP_CHECK
			checksum = ~(lowsn_udpchksum(tempTest1, Length_G+8+40));
			if(checksum == 0) 
			{
				checksum = 0xffff;
			}
	    tempTest1[47] = (BYTE)checksum;
			tempTest1[46] = (BYTE)(checksum >> 8);
			#endif
		}
		//slipGrabTxLock();
		slipSend(tempTest1, Length_G+8+40,0);               
		slipReleaseTxLock();
		return 0;
		}





	
	else if( (Flag_G & 0x7F) == 3 )
	{
		i=0;
		tempTest1[i++] = 0x60;
		tempTest1[i++] = 0;
		tempTest1[i++] = 0;
		tempTest1[i++] = 0;
		tempTest1[i++] = (Length_G+8)>> 8;
		tempTest1[i++] = Length_G+8;
		tempTest1[i++] = LOWSN_PROTO_UDP;
		tempTest1[i++] = 0x40;

   if( (Flag_G >>7) == 1 )
    {
		  tempTest1[i++] =  0x2f;
		  tempTest1[i++] =  0xfe;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
		  tempTest1[i++] =  0;
			tempTest1[i++] =  0;
			tempTest1[i++] =  0;
			tempTest1[i++] =  0x60;
		}
	else
		{
			for( j=0;j<16;j++)
				 tempTest1[i++] = SrcIpAddr_G.u8[j];
		}
	 
		/**********************************/
		
			tempTest1[i++] = 0x3F;
			tempTest1[i++] = 0xFE;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = 0x00;
			tempTest1[i++] = DstIpAddr_G.u8[0];
			tempTest1[i++] = DstIpAddr_G.u8[1];
			tempTest1[i++] = DstIpAddr_G.u8[2];
			tempTest1[i++] = DstIpAddr_G.u8[3];
		
		/**********************************/

		/* add UDP header*/

		/*Source Port*/
		tempTest1[i++] = (LocalPort_G)>> 8;
		tempTest1[i++] = (BYTE)(LocalPort_G);
		/*Destination Port*/
		tempTest1[i++] = (RemotePort_G)>> 8;
		tempTest1[i++] = (BYTE)(RemotePort_G);	
		/*Length
		// Length is the length in octets of this user datagram including this header and the data. 	*/			
		tempTest1[i++] = (BYTE)(((UINT16)(Length_G+8)) >> 8);
		tempTest1[i++] = (BYTE)((UINT16)(Length_G+8));
		/* Checksum*/
		tempTest1[i++] = 0;
		tempTest1[i++] = 0;
		for( j=0;j<Length_G;j++)
		{
			tempTest1[i++] = *(Pload_G+j);
		}
		
		tempTest1[i] = '\0';
		
		checksum = 0;
		if ( NextHeader_G == LOWSN_PROTO_UDP )  
		{
			#if LOWSN_CONF_UDP_CHECK
			checksum = ~(lowsn_udpchksum(tempTest1, Length_G+8+40));
			if(checksum == 0) 
			{
				checksum = 0xffff;
			}
	    tempTest1[47] = (BYTE)checksum;
			tempTest1[46] = (BYTE)(checksum >> 8);
			#endif
		}
		//slipGrabTxLock();
		slipSend(tempTest1, Length_G+8+40,0);            
		slipReleaseTxLock();
		return 0;
	}
	
else
	{
		slipReleaseTxLock();
		return 0;
	}
}
	
/*****************************************************************************/


/*---------------------------------------------------------------------------*/
/*Calculate the checksum of the packet in lowsn_buf. */
static uint16_t
chksum(uint16_t sum, const uint8_t *data, uint16_t len)
{
  uint16_t t;
  const uint8_t *dataptr;
  const uint8_t *last_byte;

  dataptr = data;
  last_byte = data + len - 1;

  while(dataptr < last_byte) {   /* At least two more bytes */
    t = (dataptr[0] << 8) + dataptr[1];
    sum += t;
    if(sum < t) {
      sum++;      /* carry */
    }
    dataptr += 2;
  }

  if(dataptr == last_byte) {
    t = (dataptr[0] << 8) + 0;
    sum += t;
    if(sum < t) {
      sum++;      /* carry */
    }
  }

  /* Return sum in host byte order. */
  return sum;
}
/*---------------------------------------------------------------------------*/
uint16_t
lowsn_chksum(uint16_t *data, uint16_t len)
{
  return lowsn_htons(chksum(0, (uint8_t *)data, len));
}
/*---------------------------------------------------------------------------*/

// for send: lowsn_ipchksum(a_nwk_tx_data.IPStartPtr)
// for receive: lowsn_ipchksum(a_nwk_rx_data.orgpkt.data+a_nwk_rx_data.nwkOffset)
uint16_t
lowsn_ipchksum(BYTE *iphead_start)
{
  uint16_t sum;

  //sum = chksum(0, &lowsn_buf[LOWSN_LLH_LEN], LOWSN_IPH_LEN);
  sum = chksum(0, iphead_start, LOWSN_IPH_LEN);
  return (sum == 0) ? 0xffff : lowsn_htons(sum);
}
/*---------------------------------------------------------------------------*/

// 传递来的参数iphead_start是IP头部起始位置在发送或接收缓冲中的位置, 对应标准IPv6头部
// send: upper_layer_chksum(proto, a_nwk_tx_data.IPStartPtr, a_nwk_tx_data.PayloadLength)
// receive: upper_layer_chksum(proto, a_nwk_rx_data.orgpkt.data+a_nwk_rx_data.nwkOffset, a_nwk_rx_data.PayloadLength)
static uint16_t
upper_layer_chksum(uint8_t proto, BYTE *iphead_start, UINT16 ipplen)
{

  uint16_t upper_layer_len;
  uint16_t sum;

  upper_layer_len = (ipplen - lowsn_ext_len);
// upper_layer_len = (a_nwk_tx_data.PayloadLength - lowsn_ext_len);
 //upper_layer_len = (((uint16_t)(LOWSN_IP_BUF->len[0]) << 8) + LOWSN_IP_BUF->len[1] - lowsn_ext_len);
//  upper_layer_len = lowsn_ntohs(LOWSN_IP_BUF->len);

  /* First sum pseudoheader. */
  /* IP protocol and length fields. This addition cannot carry. */
  sum = upper_layer_len + proto;
  /* Sum IP source and destination addresses. */
 // sum = chksum(sum, (uint8_t *)&LOWSN_IP_BUF->srcipaddr, 2 * sizeof(lowsn_ipaddr_t));
 //sum = chksum(sum, (uint8_t *)&(a_nwk_tx_data.SrcAddress), 2 * sizeof(lowsn_ipaddr_t));
  sum = chksum(sum, iphead_start+8, 2 * sizeof(lowsn_ipaddr_t));

  /* Sum TCP header and data. */
  //sum = chksum(sum, &lowsn_buf[LOWSN_IPH_LEN + LOWSN_LLH_LEN + lowsn_ext_len],
  //            upper_layer_len);
  //sum = chksum(sum, a_nwk_tx_data.IPStartPtr + LOWSN_IPH_LEN + lowsn_ext_len],
  //             upper_layer_len);
  sum = chksum(sum, iphead_start + LOWSN_IPH_LEN + lowsn_ext_len,upper_layer_len);
	
  //return (sum == 0) ? 0xffff : lowsn_htons(sum);
  return (sum == 0) ? 0xffff : sum;
}
/*---------------------------------------------------------------------------*/
uint16_t
lowsn_icmp6chksum(BYTE *iphead_start, UINT16 ipplen)
{
  return upper_layer_chksum(LOWSN_PROTO_ICMP6, iphead_start, ipplen);

}
/*---------------------------------------------------------------------------*/
uint16_t
lowsn_tcpchksum(BYTE *iphead_start, UINT16 ipplen)
{
  return upper_layer_chksum(LOWSN_PROTO_TCP, iphead_start, ipplen);
}
/*---------------------------------------------------------------------------*/
uint16_t
lowsn_udpchksum(BYTE *iphead_start, UINT16 ipplen)
{
  return upper_layer_chksum(LOWSN_PROTO_UDP, iphead_start, ipplen);
}
/*---------------------------------------------------------------------------*/



void nwkInit(void)
{

	ds6Init();
	nwk_pib.flags.val = 0;
	nwkState = NWK_STATE_IDLE;
	nwkRxState = NWK_RXSTATE_IDLE;
}



/*
*brief: the network layer main FSM.
*arg: none
*return: none
*/
void nwkFSM(void)
{

	/**used by next hop determitation and address resolution**/
	//lowsn_ds6_nbr_t* nbr = NULL;
	lowsn_ipaddr_t* nexthop = NULL;
	//lowsn_ds6_dest_t* locdest=NULL;
	BYTE icmp_type;

	adpFSM();	//the adp layer main FSM
	nwkRxFSM();	//the network layer recieving FSM

nwkFSM_start:
	
	switch(nwkState){
		
		case NWK_STATE_IDLE:
			if(nwk_pib.flags.bits.nwkPending == 1)  {
			/**
			  *there is a solicitation packet pending in the lowsn_buf buffer. Handle it.
			  *there are 4 kinds of scenarios:
			  * (1.1) response to echo request message,
			  * (1.2) response to neighbor solicitation message,
			  * (1.3) response to router solicitation message(router only),
			  *
			  * (2) send ICMP error message.
			  *
			  * (3)send the packet triggering address resolution when the address resolution is successful.

			**/

				icmp_type = *(a_nwk_rx_data.orgpkt.data + a_nwk_rx_data.pload_offset);

				switch (icmp_type) {
             			case ICMP6_ECHO_REQUEST:

					if (phyTxUnLocked()) {
                   				phyGrabTxLock(); //grab the lock
                  		 		nwkState = NWK_STATE_SEND_PING_REPLY;
                   	
                   				goto nwkFSM_start;
                		 	}

				#ifdef LOWSN_FFD
				
             			case ICMP6_RS:

					DEBUG_STRING(DBG_INFO,"nwk: FSM, received RS. \n");

					// 目前未加入对RS帧的核对，仅仅发现是RS就回应RA.

					if (phyTxUnLocked()) {
                   				phyGrabTxLock(); //grab the lock
                  		 		nwkState = NWK_STATE_SEND_RA;
                   				
                   				goto nwkFSM_start;
                		 	}
				#endif	

				default:
               			DEBUG_STRING(1,"NWK: Received ICMP packet that is not currently implemented, discarding.\n");
              			nwk_pib.flags.bits.nwkPending = 0;
				}	

			
				//nwkState = NWK_STATE_NEXT_HOP_START;
				
				nwk_pib.flags.bits.nwkPending = 0; //release pending
				
				goto nwkFSM_start;
				
			}

			#ifdef LOWSN_COORDINATOR
			#ifdef LOWSN_SLIP_TO_HOST
			if (nwk_pib.flags.bits.slipforwardPending) {

				if (phyTxLocked())     break;
             			phyGrabTxLock();  
				nwkInjectSlipTxPacket();
				nwkState = NWK_STATE_NEXT_HOP_START;
				goto nwkFSM_start;


			}
			
			#endif
			#endif

		break;
			
			/*****************************************************/

			#if 0
			/*send the sencond paket in the swap buffer*/
			else if(lowsn_swap_buf.buflen != 0){
				/*Copy the swap paket to the lowsn_buf, clean the swap buffer and
				  * set the network FSM state to NWK_STATE_NEXT_HOP_START*/
				lowsn_len = lowsn_swap_buf.buflen;
				memcpy(LOWSN_IP_BUF, lowsn_swap_buf.buf, lowsn_len);
				lowsn_swap_buf.buflen = 0;
				nwkState = NWK_STATE_NEXT_HOP_START;
				goto nwkFSM_start;
				
			}

			/**
			  * Process the Periodic events or time out events.
			  * here if we want to process a time out or periodic event, we MUST guarantee
			  * the lowsn_buf buffer is free. if the lowsn_buf buffer is not empty, we
			  * should process the event some time later.
			**/
			else if(internalclock_ms < sysclock_get_ms()){
				if(lowsn_len !=0){
					break;
				}
				internalclock_ms = sysclock_get_ms();
				lowsn_ds6_schedule_ms(internalclock_ms);
				if(lowsn_len==0){
					if(internalclock < sysclock_get()){
						internalclock = sysclock_get();
						lowsn_ds6_schedule(internalclock);
						if(lowsn_len == 0){
							break;
						}
					}
				}
				nwkState = NWK_STATE_NEXT_HOP_START;
				goto nwkFSM_start;
			}
			break;

			#endif

			/*****************************************************/
			
		case NWK_STATE_CMD_START:
			switch(a_nwk_service.cmd) {
				
				case LOWSN_SVC_NWK_GENERIC_TX:
					/**only support udp protocol
					  * construct the ipv6 packet include the ipv6 heander may include a fragment header,
					  * if the packet size is exceeded the link MTU, we should fragment the udp packet.
					
					**/
					
                                  lowsn_len = a_nwk_tx_data.PayloadLength + LOWSN_IPH_LEN;
					

					#if 0
					//if(lowsn_len > LOWSN_LINK_MTU){
					if(lowsn_len > lowsn_ds6_if.link_mtu){

						conPrintROMString(" 13b\n");
						//here if the packet size excceded the link MTU, we just discard the packet!!!
						lowsn_len = 0;
						a_nwk_service.status = LOWSN_STATUS_NWK_PACKET_EXCEEDED;
						nwkState = NWK_STATE_IDLE;
						break;
					}
					#endif

					nwkState = NWK_STATE_NEXT_HOP_START;
					goto nwkFSM_start;
//					break;

				case LOWSN_SVC_NWK_PING6_TX:
					DEBUG_STRING(DBG_INFO,"nwk send a echo request. \n");

					#if 0
					//ping命令暂时不支持IP和适配层的分片重组，即不能超过127个字节
					if (a_nwk_service.args.tx_ping6.plen > LOWSN_MAX_FRAME_SIZE - LOWSN_ICMPH_LEN - LOWSN_IPH_LEN - 4) {
						a_nwk_service.status = LOWSN_STATUS_NWK_PING_OVERLEN;
						DEBUG_STRING(DBG_INFO,"NWK: Ping exceeds max length.\n");
						nwkState = NWK_STATE_IDLE;	
						break;
					}	

					#endif
					
					 //break if the TXBUFFER is locked
             				if (phyTxLocked()) break;
             				phyGrabTxLock();  //Grab the lock

					nwk_pib.flags.bits.nwkIsGetEchoReply = 0;
             				
					icmpFmtEchoRequest(a_nwk_service.args.tx_ping6.plen);
					icmpCommonFmt(&a_nwk_service.args.tx_ping6.dstaddr);
					nwkFindDstDLLAddr();
					
					nwk_pib.flags.bits.WaitingForEchoRelay = 1;
					
					nwkTxData(FALSE);
             				nwkState = NWK_STATE_PING_REQ_WAIT1;
					break;


				case LOWSN_SVC_NWK_GET_RA:
					DEBUG_STRING(DBG_INFO,"nwk send RS. \n");

             				if (phyTxLocked()) break;
             				phyGrabTxLock();  //Grab the lock

					nwk_pib.flags.bits.nwkIsGetRA = 0;
             				
					ndFmtRS();
					nwkFindDstDLLAddr();
					
					nwk_pib.flags.bits.WaitingForRA = 1;
					
					nwkTxData(FALSE);
             				nwkState = NWK_STATE_RS_WAIT1;
					break;



				case LOWSN_SVC_NWK_ADP_PASSTHRU:
		 			if (adpBusy()) break;  //wait until nwk is idle
		 			adpDoService();
		 			nwkState = NWK_STATE_ADP_PASSTHRU_WAIT;
		 		break;

				default:
					break;

			}
			
			break;

		case NWK_STATE_PING_REQ_WAIT1:

			// 暂时未设置NWK发送状态机，所以直接查看下层状态决定是否发送成功	
			if (adpBusy()) break;

			phyReleaseTxLock();	
		
			if (a_adp_service.status != LOWSN_STATUS_SUCCESS) {
				a_nwk_service.status = a_adp_service.status;
				nwk_pib.flags.bits.WaitingForEchoRelay = 0;
				nwkState = NWK_STATE_IDLE;			
             			break;
			}

           		//now need to wait for ping response
           		//start a timer
           		nwk_utility_timer = halGetMACTimer();
          		nwkState = NWK_STATE_PING_REQ_WAIT2;
           		break;

		case NWK_STATE_PING_REQ_WAIT2:
			if (nwk_pib.flags.bits.nwkIsGetEchoReply) {
             		//get ping relay successful
			a_nwk_service.status = LOWSN_STATUS_SUCCESS;
			nwk_pib.flags.bits.WaitingForEchoRelay = 0;
			nwkState = NWK_STATE_IDLE;	
			}else if ((halMACTimerNowDelta(nwk_utility_timer)) >  MSECS_TO_MACTICKS(NWK_DEFAULT_PING_WAIT_TIME)){
			//timeout on ping, give it up
			a_nwk_service.status = LOWSN_STATUS_NWK_PING_TIMEOUT;
			DEBUG_STRING(DBG_INFO,"NWK: Ping timeout\n");
			nwk_pib.flags.bits.WaitingForEchoRelay = 0;
			nwkState = NWK_STATE_IDLE;	
           		}

          		break;

		case NWK_STATE_RS_WAIT1:

			// 暂时未设置NWK发送状态机，所以直接查看下层状态决定是否发送成功	
			if (adpBusy()) break;

			phyReleaseTxLock();	
		
			if (a_adp_service.status != LOWSN_STATUS_SUCCESS) {
				a_nwk_service.status = a_adp_service.status;
				nwk_pib.flags.bits.WaitingForRA = 0;
				nwkState = NWK_STATE_IDLE;			
             			break;
			}

           		//now need to wait for ping response
           		//start a timer
           		nwk_utility_timer = halGetMACTimer();
          		nwkState = NWK_STATE_RS_WAIT2;
           		break;

		case NWK_STATE_RS_WAIT2:
			if (nwk_pib.flags.bits.nwkIsGetRA) {
             		//get RA relay successful
			a_nwk_service.status = LOWSN_STATUS_SUCCESS;
			nwk_pib.flags.bits.WaitingForRA = 0;
			nwkState = NWK_STATE_IDLE;	
			}else if ((halMACTimerNowDelta(nwk_utility_timer)) > MSECS_TO_MACTICKS(LOWSN_DEFAULT_RS_WAIT_DURATION)){
			//timeout on ping, give it up
			a_nwk_service.status = LOWSN_STATUS_NWK_RS_TIMEOUT;
			DEBUG_STRING(DBG_INFO,"nwk: RS timeout\n");
			nwk_pib.flags.bits.WaitingForRA = 0;
			nwkState = NWK_STATE_IDLE;	
           		}

          		break;
			
		case NWK_STATE_NEXT_HOP_START:

			nexthop = NULL;

			/*****************************************************/
			#if 0
			
			/*check if the fragment buffer is not free*/
			if(nwk_pib.flags.bits.nwktxfragment == 1){
				//TX other fragments of the original packet
				lowsn_tx_other_frag();
			}
			
			if(lowsn_len == 0){
				nwkState = NWK_STATE_IDLE;
				break;
			}
			/**
			  * check if the packet need to be fragmented.
			**/
			if(lowsn_len >= pmtu.size){	//pmtu 15 testing
            			DEBUG_STRING(DBG_INFO, "Fragment the packet.\r\n");
				lowsn_tx_first_frag();
			}

			#endif
			
			/*****************************************************/
			
			/*start of next hop determination*/
			/**
			  *  if the destination is a multicast address, we don't need
			  *  to execute nexthop determination and address resolution.
			**/
			nwkFindDstDLLAddr();

			#ifdef LOWSN_COORDINATOR
			#ifdef LOWSN_SLIP_TO_HOST
			if (nwk_pib.flags.bits.slipforwardPending) {
				nwkTxData(TRUE);
				// 网络传来的IP包已经分段好，无需再分片
				nwkState = NWK_STATE_SLIP_TX_WAIT;
				goto nwkFSM_start;
				//break;	
			}

			#endif
			#endif
	
			nwkTxData(FALSE);
			if(nwk_pib.flags.bits.nwktxfragment == 1){
				nwkState = NWK_STATE_FRAG_TX_WAIT;
			}
			else{
				nwkState = NWK_STATE_GENERIC_TX_WAIT;
			}
			goto nwkFSM_start;
				//break;	
			
		
		case NWK_STATE_ADDR_RESOL_START:
			
				DEBUG_STRING(DBG_INFO,"NWK_STATE_ADDR_RESOL_START, Not used in WPAN\n");
				nwkState = NWK_STATE_IDLE;
				break;					
			

		case NWK_STATE_FRAG_TX_WAIT:
			 if (adpBusy()) break;
			 //adp finished, copy status.
			 a_nwk_service.status = a_adp_service.status;
			 nwkState = NWK_STATE_NEXT_HOP_START;
			break;
		
		case NWK_STATE_GENERIC_TX_WAIT:

			 if (adpBusy()) break;
			 //adp finished, copy status.
			 a_nwk_service.status = a_adp_service.status;
			 nwkState = NWK_STATE_IDLE;
			break;
                #ifdef LOWSN_COORDINATOR
		#ifdef LOWSN_SLIP_TO_HOST
		case NWK_STATE_SLIP_TX_WAIT:

			 if (adpBusy()) break;
			 a_slip_service.status = a_adp_service.status;
			 nwk_pib.flags.bits.slipforwardPending = 0;
			 phyReleaseTxLock();
			 nwkState = NWK_STATE_IDLE;
			break;
                #endif
                #endif
                        
		case NWK_STATE_ADP_PASSTHRU_WAIT:
		 //for split-phase passthrus
		 if (adpBusy()) break;
		 a_nwk_service.status = a_adp_service.status;
		 nwkState = NWK_STATE_IDLE;
		 break;

		 case NWK_STATE_SEND_PING_REPLY:
			DEBUG_STRING(DBG_INFO,"NWK: Sending ping reply.\n");
           		icmpFmtEchoReply();
			icmpCommonFmt(&a_nwk_rx_data.SrcAddress);

			nwk_pib.flags.bits.nwkPending = 0; //release packet
			
			nwkFindDstDLLAddr();
           		nwkTxData(FALSE);
           		nwkState = NWK_STATE_GENERIC_TX_WAIT_AND_UNLOCK;
		break;

		#ifdef LOWSN_FFD
		case NWK_STATE_SEND_RA:
			DEBUG_STRING(DBG_INFO,"NWK: Sending RA.\n");
           		ndFmtRA(0);
				
			nwk_pib.flags.bits.nwkPending = 0; //release packet
				
			nwkFindDstDLLAddr();
           		nwkTxData(FALSE);
           		nwkState = NWK_STATE_GENERIC_TX_WAIT_AND_UNLOCK;
		break;
		#endif

		              //this is used by NWK ICMPs in general which send a packet.
		case NWK_STATE_GENERIC_TX_WAIT_AND_UNLOCK:
          		if (adpBusy()) break;
          		//TX is finished, copy status
          		 a_nwk_service.status = a_adp_service.status;
           		 nwkState = NWK_STATE_IDLE;	
           		//also unlock TX buffer
           		phyReleaseTxLock();
           		break;
			
		default:
			break;


       }//end of switch

}




/*
*brief: the network layer recieving FSM.
*arg: none
*return: none
*/
void nwkRxFSM(void){
		
//	char* locpointer=NULL;
	BYTE *ptr;

nwkRxFSM_start:
	switch(nwkRxState){
	case NWK_RXSTATE_IDLE:

		#ifdef LOWSN_COORDINATOR
		#ifdef LOWSN_SLIP_TO_HOST
		if (nwk_pib.flags.bits.slipRxPending)  {
			
			if (nwkInjectSlipRxPacket() == 0)  {
				a_slip_service.status = LOWSN_STATUS_SUCCESS;
				nwk_pib.flags.bits.slipRxPending = 0;
				nwkRxState = NWK_RXSTATE_START;
				goto nwkRxFSM_start;
			}
			else 
			{
				// 分配内存失败
				a_slip_service.status = LOWSN_STATUS_SLIP_NWK_RX_HEAP_ERROR;
				nwk_pib.flags.bits.slipRxPending = 0;
			}
		}
		#endif
		#endif
		
		break;
		
	case NWK_RXSTATE_START:

		//if(lladdr_dadfail == 1){
		//	nwkRxState = NWK_RXSTATE_IDLE;
		//	lowsn_len = 0;
		//	break;
		//}

		if (a_nwk_rx_data.hcflag == ADP_RX_IP_UNCOMPRESS) {	
			
			// 如果hcflag没压缩，表示需NWK自己解析IP Header

			// 如果负载长度比IP 头部40个字节还小，则丢弃，但ADP头部中没有长度域，只能
			// 从MAC域中减出来，较为复杂，暂不检查
			
			ptr = a_nwk_rx_data.orgpkt.data + a_nwk_rx_data.nwkOffset;

			if (!NWK_IS_IP6(*ptr)) {
				DEBUG_STRING(DBG_INFO,"NWK: Received is not IPv6 packet, discarding.\n");
				MemFree(a_nwk_rx_data.orgpkt.data);
				nwkRxState = NWK_RXSTATE_IDLE;
				break;
			}

			// 解析IPv6头部，对应内容填入a_nwk_rx_data 的相应域，方便下面代码进行判断
			nwkParseHdr(ptr);
			
		}

		else if (a_nwk_rx_data.hcflag == ADP_RX_IP_COMPRESS) {

			// 适配层已经恢复了IP头部的各个域，并在handoff函数中填充到了a_nwk_rx_data中
			DEBUG_STRING(DBG_INFO,"NWK: Received compressed IP packet. \n");
		}

		else {
				DEBUG_STRING(DBG_INFO,"NWK: Received wrong hcflag packet, discarding.\n");
				MemFree(a_nwk_rx_data.orgpkt.data);
				nwkRxState = NWK_RXSTATE_IDLE;
				break;
		}		
	
			
		// 检查负载头部长度域是否正确，暂略去

			/*********
			//check the payload length of the recieced packet
			if(((LOWSN_IP_BUF->len[0]<<8)+LOWSN_IP_BUF->len[1]) != (lowsn_len-LOWSN_IPH_LEN)){
				if(((LOWSN_IP_BUF->len[0]<<8) + LOWSN_IP_BUF->len[1]) < (lowsn_len-LOWSN_IPH_LEN)){

					lowsn_len = (LOWSN_IP_BUF->len[0]<<8) + LOWSN_IP_BUF->len[1] +LOWSN_IPH_LEN;
				}
				else if(((LOWSN_IP_BUF->len[0]<<8) + LOWSN_IP_BUF->len[1]) > (lowsn_len-LOWSN_IPH_LEN)){
					/.drop the packet
					DEBUG_STRING(DBG_INFO, "Discard the packet : invalid Payload Lenght\r\n");
					lowsn_len = 0;
					nwkRxState = NWK_RXSTATE_IDLE;
					break;
				}
			}

			*****************/
			
		//check the src address of the packet
		
		//if the source ipv6 address is a multicast address, drop the packet

		if(lowsn_is_addr_mcast(&a_nwk_rx_data.SrcAddress)){
			DEBUG_STRING(DBG_INFO,"NWK: Discard the packet : Muliticast src IP Addr.\n");
			MemFree(a_nwk_rx_data.orgpkt.data);
			lowsn_len = 0;
			nwkRxState = NWK_RXSTATE_IDLE;
			break;
		}


		//check the destination ipv6 address*/
		if(!lowsn_ds6_is_my_uaddr(&a_nwk_rx_data.DstAddress)&&
		   !lowsn_ds6_is_my_maddr(&a_nwk_rx_data.DstAddress)&&
                 !lowsn_ds6_is_my_aaddr(&a_nwk_rx_data.DstAddress)) {
			
			#ifdef LOWSN_COORDINATOR
			#ifdef LOWSN_SLIP_TO_HOST
			    if (slip_pib.forward_kind == SLIP_FORWARD_IP_PACKET)  {
			    		if (a_nwk_rx_data.hcflag == ADP_RX_IP_COMPRESS)  {
						nwkRestoreCompHeader();
					}	
        				nwkRxState = NWK_RXSTATE_FORWARD_HOST;
       			 	goto nwkRxFSM_start;
    			    }		
   			#endif
    			#endif
			
			DEBUG_STRING(DBG_INFO,"NWK: Discard the packet : Error dest IP Addr.\n");
			MemFree(a_nwk_rx_data.orgpkt.data);
			lowsn_len = 0;
			nwkRxState = NWK_RXSTATE_IDLE;
			break;
		}


		ptr = a_nwk_rx_data.orgpkt.data + a_nwk_rx_data.pload_offset;

			/********************************/
			/*								*/
			/*	  Process the next header field	*/
			/*								*/
			/********************************/
			lowsn_next_hdr = &(a_nwk_rx_data.NextHeader);
			lowsn_ext_len = 0;
			lowsn_ext_bitmap = 0;
		
			// 暂不支持扩展Header，所以暂不用while(1)
			//while(1){
				switch(a_nwk_rx_data.NextHeader){
					/*process the udp header*/
					case LOWSN_PROTO_UDP:
						
						//DEBUG_STRING(DBG_INFO, "UDP packet\r\n");
						
						#if LOWSN_CONF_UDP_CHECK
						// 目前的计算校验和函数只支持连续IP数据段，但采用压缩时，IP数据段的
						// 连续性会被破坏，所以需要重新设计一个针对压缩的校验函数，目前暂
						// 不对压缩包进行校验.

						if (a_nwk_rx_data.hcflag == ADP_RX_IP_UNCOMPRESS) 
						{
							UINT16 udpChecksum;
							udpChecksum = (((UINT16)*(ptr+6)) << 8);
							udpChecksum += *(ptr+7);
									
							if(udpChecksum != 0 && lowsn_udpchksum(a_nwk_rx_data.orgpkt.data+a_nwk_rx_data.nwkOffset, a_nwk_rx_data.PayloadLength) != 0xffff) { 
								DEBUG_STRING(DBG_INFO,"NWK: udp: bad checksum.\n  Check Num from rcv data is :  ");
								DEBUG_UINT16(DBG_INFO, udpChecksum);	
								DEBUG_STRING(DBG_INFO, "   Recalculated Number is :");
								DEBUG_UINT16(DBG_INFO, lowsn_udpchksum(a_nwk_rx_data.orgpkt.data+a_nwk_rx_data.nwkOffset, a_nwk_rx_data.PayloadLength));
								MemFree(a_nwk_rx_data.orgpkt.data);
								lowsn_len = 0;
								nwkRxState = NWK_RXSTATE_IDLE;
								goto nwkRxFSM_start;		
							}
						}	
						#endif

						// UDP的解析及出错后回复端口不可达报文由APS层处理

						nwkRxState = NWK_RXSTATE_APS_HANDOFF;
						goto nwkRxFSM_start;							

					case LOWSN_PROTO_ICMP6:
						
						//DEBUG_STRING(DBG_INFO, "Processing ICMP header \n");
						//DEBUG_STRING(DBG_INFO, "ICMP Type No.: ");
						//DEBUG_UINT8(DBG_INFO, *ptr);
						//DEBUG_STRING(DBG_INFO, "\n");

						if (a_nwk_rx_data.hcflag == ADP_RX_IP_UNCOMPRESS)  {					
							if(lowsn_icmp6chksum(a_nwk_rx_data.orgpkt.data+a_nwk_rx_data.nwkOffset, a_nwk_rx_data.PayloadLength) != 0xffff){  
								DEBUG_STRING(DBG_INFO,"Discard the ICMP packet : Error Checksum\r\n");
								DEBUG_STRING(DBG_INFO,"NWK: icmp: bad checksum.\n");
								MemFree(a_nwk_rx_data.orgpkt.data);
								lowsn_len = 0;
								nwkRxState = NWK_RXSTATE_IDLE;
								goto nwkRxFSM_start;
							}
						}	
							// check the icmp type
							switch(*ptr) {

								/**************************************************
								//parse the NS message and construct a NA massage
								case ICMP6_NS:
									lowsn_nd_ns_input();

									nwkRxState = NWK_RXSTATE_CMD_PENDING;

                                    				nwk_pib.flags.bits.nwkPending = 1;

									goto nwkRxFSM_start;

								//parse the NA message
								case ICMP6_NA:
									lowsn_nd_na_input();
									nwkRxState = NWK_RXSTATE_IDLE;
									goto nwkRxFSM_start;

								//parse the Redirect message
								case ICMP6_RED:
									lowsn_nd_red_input();
									nwkRxState = NWK_RXSTATE_IDLE;
									goto nwkRxFSM_start;

								case ICMP6_PACKET_TOO_BIG:
									lowsn_icmp6_pktbigerr_input();
									nwkRxState = NWK_RXSTATE_IDLE;
									goto nwkRxFSM_start;
									
								   **************************************************/

								#ifdef LOWSN_FFD

								case ICMP6_RS:

									DEBUG_STRING(DBG_INFO,"nwk: received RS. \n");
									
									nwk_pib.flags.bits.nwkPending = 1;
									nwkRxState = NWK_RXSTATE_CMD_PENDING;
									goto nwkRxFSM_start;

								#endif	


								#ifndef LOWSN_COORDINATOR
								
								case ICMP6_RA:

									DEBUG_STRING(DBG_INFO,"nwk: received RA. \n");
									
									// 暂设定为主动发出RS后才解析收到的RA帧，实际上应可自由接收RA
									if (nwk_pib.flags.bits.WaitingForRA){
        									ndParseRA();
										nwk_pib.flags.bits.nwkIsGetRA = 1;
      									}					
		
      									//free this packet, we are finished with it.
      									MemFree(a_nwk_rx_data.orgpkt.data);
										
									lowsn_len = 0;
									nwkRxState = NWK_RXSTATE_IDLE;
									goto nwkRxFSM_start;
									
								#endif

								case ICMP6_ECHO_REQUEST:
									//lowsn_icmp6_echo_request_input();
									// 暂不支持超过127个字节的PING请求
									if (a_nwk_rx_data.PayloadLength> LOWSN_MAX_FRAME_SIZE - LOWSN_ICMPH_LEN - LOWSN_IPH_LEN - 4) {
										DEBUG_STRING(DBG_INFO,"NWK: icmp: echo request overlength.\n");
										MemFree(a_nwk_rx_data.orgpkt.data);
										lowsn_len = 0;
										nwkRxState = NWK_RXSTATE_IDLE;
										goto nwkRxFSM_start;
									}	
									nwk_pib.flags.bits.nwkPending = 1;
									nwkRxState = NWK_RXSTATE_CMD_PENDING;
									goto nwkRxFSM_start;

								case ICMP6_ECHO_REPLY:
      									if (nwk_pib.flags.bits.WaitingForEchoRelay){
        									icmpParseEchoReply();
      									}					
		
      									//free this packet, we are finished with it.
      									MemFree(a_nwk_rx_data.orgpkt.data);
										
									lowsn_len = 0;
									nwkRxState = NWK_RXSTATE_IDLE;
									goto nwkRxFSM_start;
			
								default:
									MemFree(a_nwk_rx_data.orgpkt.data);
									lowsn_len = 0;
									nwkRxState = NWK_RXSTATE_IDLE;
									goto nwkRxFSM_start;
						
						}
					  //break;

                              /*************************************************************************/
					#if 0

					
					//process the hop-by-hop option header
					case LOWSN_PROTO_HBHO:
						DEBUG_STRING(DBG_INFO, "Processing hop-by-hop header\r\n");
						//the hop-by-hop option must immedately follow the ipv6 header
						if(lowsn_ext_bitmap != 0){
							//*the hop-by-hop option is not the first extention option header
							  //* or the option has already been present. the reciever drops the
							  //* packet and constructs an ICMP error message.
							
							lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_NEXTHEADER,
							 					(uint32_t)(lowsn_next_hdr - (uint8_t *)LOWSN_IP_BUF));
							if(lowsn_len>0){
								nwk_pib.flags.bits.nwkPending = 1;
								//nwk_pib.flags.bits.sendicmperr = 1;
								nwkRxState = NWK_RXSTATE_CMD_PENDING;
								goto nwkRxFSM_start;
							}
						}
						else{
							lowsn_ext_bitmap |= LOWSN_EXT_HDR_BITMAP_HBHO;
							// process the extention header
							switch(lowsn_ext_hdr_process()) {
							      //continue
							      case 0:
							     	 lowsn_next_hdr = &LOWSN_EXT_BUF->next;
									//
									//   verify that a node discard a packet that has a Next Header
									 //   field of zero in header orther than an IPv6 header and generates
									  // an ICMPv6 Parameter message to the source of the packet.

									 if(*lowsn_next_hdr == 0){
									 	DEBUG_STRING(DBG_INFO, "Discard, Next header is zero\r\n");
									 	lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_NEXTHEADER,
															(uint32_t)(lowsn_next_hdr - (uint8_t *)LOWSN_IP_BUF));
										if(lowsn_len>0){
											nwk_pib.flags.bits.nwkPending = 1;
											nwkRxState = NWK_RXSTATE_CMD_PENDING;
											goto nwkRxFSM_start;
										}
									 }
							     	 lowsn_ext_len += (LOWSN_EXT_BUF->len << 3) + 8;
							     	 break;
							       //silently discard
							      case 1:
							     	 lowsn_len = 0;
							     	 nwkRxState = NWK_RXSTATE_IDLE;
							     	 goto nwkRxFSM_start;
							     // send icmp error message (created in ext_hdr_options_process) and discard
							      case 2:
							     	 nwk_pib.flags.bits.nwkPending = 1;
							     	 //nwk_pib.flags.bits.sendicmperr = 1;
							     	 nwkRxState = NWK_RXSTATE_CMD_PENDING;
							     	 goto nwkRxFSM_start;
							}
						}
						break;
					//process the destination option header
					case LOWSN_PROTO_DESTO:
						DEBUG_STRING(DBG_INFO, "Processing dest header\r\n");
						if(lowsn_ext_bitmap & LOWSN_EXT_HDR_BITMAP_DESTO1){
							if(lowsn_ext_bitmap & LOWSN_EXT_HDR_BITMAP_DESTO2){
								//Destination option header. if we saw two already, drop
								 lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_NEXTHEADER,
								 					(uint32_t)(lowsn_next_hdr - (uint8_t *)LOWSN_IP_BUF));
								if(lowsn_len>0){
									nwk_pib.flags.bits.nwkPending = 1;
									//nwk_pib.flags.bits.sendicmperr = 1;
									nwkRxState = NWK_RXSTATE_CMD_PENDING;
									goto nwkRxFSM_start;
								}
							}
							else{
								uint8_t ret;
								lowsn_ext_bitmap |= LOWSN_EXT_HDR_BITMAP_DESTO2;
								ret = lowsn_ext_hdr_process();
								if(ret == 0){
								    lowsn_next_hdr = &LOWSN_EXT_BUF->next;
									if(*lowsn_next_hdr == 0){
										DEBUG_STRING(DBG_INFO, "Discard, Next header is zero\r\n");
										lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_NEXTHEADER,
															(uint32_t)(lowsn_next_hdr - (uint8_t *)LOWSN_IP_BUF));
										if(lowsn_len>0){
											nwk_pib.flags.bits.nwkPending = 1;
											nwkRxState = NWK_RXSTATE_CMD_PENDING;
											goto nwkRxFSM_start;
										}
									}
								     lowsn_ext_len += (LOWSN_EXT_BUF->len << 3) + 8;
								     break;
								}
								else if(ret == 1){
										lowsn_len = 0;
										nwkRxState = NWK_RXSTATE_IDLE;
										goto nwkRxFSM_start;
								}
								else if(ret == 2){
										nwk_pib.flags.bits.nwkPending = 1;
										nwkRxState = NWK_RXSTATE_CMD_PENDING;
										goto nwkRxFSM_start;
								}						
							}
						}
						else{
							uint8_t ret;
							lowsn_ext_bitmap |= LOWSN_EXT_HDR_BITMAP_DESTO1;
							//process the destination option header
							ret = lowsn_ext_hdr_process();
							if(ret == 0){
							    lowsn_next_hdr = &LOWSN_EXT_BUF->next;
								if(*lowsn_next_hdr == 0){
									DEBUG_STRING(DBG_INFO, "Discard, Next header is zero\r\n");
									lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_NEXTHEADER,
														(uint32_t)(lowsn_next_hdr - (uint8_t *)LOWSN_IP_BUF));
									if(lowsn_len>0){
										nwk_pib.flags.bits.nwkPending = 1;
										nwkRxState = NWK_RXSTATE_CMD_PENDING;
										goto nwkRxFSM_start;
									}
								}
							     lowsn_ext_len += (LOWSN_EXT_BUF->len << 3) + 8;
							     break;
							}
							else if(ret == 1){
									lowsn_len = 0;
									nwkRxState = NWK_RXSTATE_IDLE;
									goto nwkRxFSM_start;
							}
							else if(ret == 2){
									nwk_pib.flags.bits.nwkPending = 1;
									nwkRxState = NWK_RXSTATE_CMD_PENDING;
									goto nwkRxFSM_start;
							}						
						}
						break;
					//process the routing option header
					case LOWSN_PROTO_ROUTING:
						DEBUG_STRING(DBG_INFO, "Processing routing header\r\n");
						if(lowsn_ext_bitmap & LOWSN_EXT_HDR_BITMAP_ROUTING){
							 lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_NEXTHEADER,
							 					(uint32_t)(lowsn_next_hdr - (uint8_t *)LOWSN_IP_BUF));
							if(lowsn_len>0){
								nwk_pib.flags.bits.nwkPending = 1;
								//nwk_pib.flags.bits.sendicmperr = 1;
								nwkRxState = NWK_RXSTATE_CMD_PENDING;
								goto nwkRxFSM_start;
							}
						}
						else{
							lowsn_ext_bitmap |= LOWSN_EXT_HDR_BITMAP_ROUTING;
						    //
						     //Routing Header  length field is in units of 8 bytes, excluding
						     // As per RFC2460 section 4.4, if routing type is unrecognized:
						     // if segments left = 0, ignore the header
						     // if segments left > 0, discard packet and send icmp error pointing
						     // to the routing type
						     //
						    if(LOWSN_ROUTING_BUF->seg_left > 0) {
						     	 lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_HEADER,
							  		LOWSN_IPH_LEN + lowsn_ext_len + 2);
								if(lowsn_len>0){
									nwk_pib.flags.bits.nwkPending = 1;
									nwkRxState = NWK_RXSTATE_CMD_PENDING;
									goto nwkRxFSM_start;
								}
						    }
							else{
						    	lowsn_next_hdr = &LOWSN_EXT_BUF->next;
						    	lowsn_ext_len += (LOWSN_EXT_BUF->len << 3) + 8;
								break;
							}
						}
						break;
						
					//process the fragment header
					case LOWSN_PROTO_FRAG:
						DEBUG_STRING(DBG_INFO, "Processing frag header\r\n");
						ret = lowsn_reassembly();
						if(ret == 0){
							nwkRxState = NWK_RXSTATE_IDLE;
							goto nwkRxFSM_start;
						}
						else if(ret == 1){
							nwk_pib.flags.bits.nwkPending = 1;
							nwkRxState = NWK_RXSTATE_CMD_PENDING;
							goto nwkRxFSM_start;
						}
						else if(ret == 2){
							nwkRxState = NWK_RXSTATE_START;
							goto nwkRxFSM_start;
						}

					//no next header
					case LOWSN_PROTO_NONE:
						/*slient to discard the packet*/
						lowsn_len = 0;
						nwkRxState = NWK_RXSTATE_IDLE;
						goto nwkRxFSM_start;
					
					//process unrec
					default:
						lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_NEXTHEADER,
						 					(uint32_t)(lowsn_next_hdr - (uint8_t *)LOWSN_IP_BUF));
						if(lowsn_len>0){
							nwk_pib.flags.bits.nwkPending = 1;
							//nwk_pib.flags.bits.sendicmperr = 1;
							nwkRxState = NWK_RXSTATE_CMD_PENDING;
							goto nwkRxFSM_start;
						}
						break;


				#endif  //if 0	
				 /*************************************************************************/
				default:
					DEBUG_STRING(DBG_INFO,"NWK: unrecoginzed next header, drop it.  \n");
					MemFree(a_nwk_rx_data.orgpkt.data);
					lowsn_len = 0;
					nwkRxState = NWK_RXSTATE_IDLE;
				break;
				
			}
		

		break;
		
	case NWK_RXSTATE_APS_HANDOFF:
		if(apsRxBusy()) break;

		apsRxHandoff();
		nwkRxState = NWK_RXSTATE_IDLE;
		lowsn_len = 0;
		break;
		
	case NWK_RXSTATE_CMD_PENDING:
		if (nwk_pib.flags.bits.nwkPending ) break;
		MemFree(a_nwk_rx_data.orgpkt.data);
              nwkRxState = NWK_RXSTATE_IDLE;
              break;
			  
  	#ifdef LOWSN_COORDINATOR
  	#ifdef LOWSN_SLIP_TO_HOST
  	case NWK_RXSTATE_FORWARD_HOST:
  		if (slipTxLocked())  break;

		DEBUG_STRING(DBG_INFO,"NWK: Forward IP packet to the host.\n");

		slipGrabTxLock();
		slipSendIPFlags();
		
		if (a_nwk_rx_data.hcflag == ADP_RX_IP_COMPRESS)  {
			slipSend(&ip_header_buf[0], LOWSN_IPH_LEN, 3);
			slipSend(a_nwk_rx_data.orgpkt.data+a_nwk_rx_data.pload_offset, a_nwk_rx_data.PayloadLength, 1);
		}	
		else  {
			slipSend(a_nwk_rx_data.orgpkt.data+a_nwk_rx_data.nwkOffset, a_nwk_rx_data.PayloadLength+LOWSN_IPH_LEN, 1);
		}	
			
		slipReleaseTxLock();

		MemFree(a_nwk_rx_data.orgpkt.data);
		lowsn_len = 0;
		nwkRxState = NWK_RXSTATE_IDLE;

		break;
  	#endif
  	#endif

	default:
		break;


	}//end of switch

}


/**
  * handle the data to adp layer .
  * set the arguments used by adp service in the function.
  * argument : the link layer address of the nexthop, if the lladdr=NULL indicate that
  * the packet is a broadcast packet.
  **/
  // IETF标准网络字节顺序是大端模式，而IEEE 802.15.4标准是小端模式
  // CC2430是小端存储，发送缓冲是倒着往前放数据的。
  // forward_flag: 0 正常发送; 1: 转发包( 主要是来自SLIP的IP包)
void nwkTxData(BOOL forward_flag)
{


	BYTE i;
	UINT16 checksum;

	if (forward_flag == FALSE)
	{

	// add Destination Address
	for(i=0; i<16; i++) {
       	phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = a_nwk_tx_data.DstAddress.u8[15-i];
       }

	// add Source Address
	for(i=0; i<16; i++) {
       	phy_pib.currentTxFrm--;
              *phy_pib.currentTxFrm = a_nwk_tx_data.SrcAddress.u8[15-i];
       }
	
	// add Hop Limit
       phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = a_nwk_tx_data.HopLimit;

	// add Next Header
       phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = a_nwk_tx_data.NextHeader;

	//add Payload Length
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (a_nwk_tx_data.PayloadLength);
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (a_nwk_tx_data.PayloadLength >> 8);	

	//add Flow Label (last 16 bits)

	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (a_nwk_tx_data.FlowLabel);
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE) (a_nwk_tx_data.FlowLabel >> 8);

	// add Flow Label (first 4 bits), Traffic Class and Version
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = ((BYTE) (a_nwk_tx_data.FlowLabel >> 16)) & 0x0F + (BYTE) ((a_nwk_tx_data.TrafficClass & 0x0F) << 4);
	phy_pib.currentTxFrm--;
	*phy_pib.currentTxFrm = (BYTE)((a_nwk_tx_data.TrafficClass & 0xF0) >> 4) + (BYTE)((a_nwk_tx_data.Version & 0x0F) << 4);

	phy_pib.currentTxFlen = phy_pib.currentTxFlen + LOWSN_IPH_LEN;

	// 各个信息都已获取，构造伪IP头，计算校验和
	//a_nwk_tx_data.IPStartPtr = phy_pib.currentTxFrm;  //记录IP头部位置

#if 1
	checksum = 0;
	if (a_nwk_tx_data.NextHeader == LOWSN_PROTO_UDP)  {

		#if LOWSN_CONF_UDP_CHECK
		checksum = ~(lowsn_udpchksum(phy_pib.currentTxFrm, a_nwk_tx_data.PayloadLength));
		if(checksum == 0) {
			checksum = 0xffff;
		}
              *(phy_pib.currentTxFrm+LOWSN_IPH_LEN+7) = (BYTE)checksum;
		*(phy_pib.currentTxFrm+LOWSN_IPH_LEN+6) = (BYTE)(checksum >> 8);
		#endif
	}

	else if  (a_nwk_tx_data.NextHeader == LOWSN_PROTO_ICMP6)  {
							
		checksum = ~ lowsn_icmp6chksum(phy_pib.currentTxFrm, a_nwk_tx_data.PayloadLength);
		*(phy_pib.currentTxFrm+LOWSN_IPH_LEN+3) = (BYTE) checksum;
		*(phy_pib.currentTxFrm+LOWSN_IPH_LEN+2) = (BYTE) (checksum >> 8);	
	}	

	else  {
		DEBUG_STRING(DBG_INFO, "nwk: check sum is not supported for current next header. n");
							
	}

#endif	

	}

	// setup the adaption layer param
	// 这部分已经转移到ADP中压缩头部之后处理.

	lowsn_len = 0;
	a_adp_service.cmd = LOWSN_SVC_ADP_GENERIC_TX;
	// 传递参数给适配层

	a_adp_service.args.hc_info.HopLimit = a_nwk_tx_data.HopLimit;
	a_adp_service.args.hc_info.NextHeader = a_nwk_tx_data.NextHeader;
	a_adp_service.args.hc_info.TrafficClass = a_nwk_tx_data.TrafficClass;
	a_adp_service.args.hc_info.FlowLabel = a_nwk_tx_data.FlowLabel;
	a_adp_service.args.hc_info.SrcAddress = a_nwk_tx_data.SrcAddress;
	a_adp_service.args.hc_info.DstAddress = a_nwk_tx_data.DstAddress;
	
	adpDoService();

}

#if 0
/*---------------------------------------------------------------------------*/
/**
  *return value:
  *	0	skip over this option and continue processing the header.
  *	1	discard the packet and do not send any icmp error message.
  *	2	discard the packet and send an icmp error message.
**/
uint8_t lowsn_ext_hdr_process(void){
	
 /*
  * Length field in the extension header: length of the header in units of
  * 8 bytes, excluding the first 8 bytes
  * length field in an option : the length of data in the option
  */
  uint8_t local;
  lowsn_ext_opt_offset = 2;
  while(lowsn_ext_opt_offset < ((LOWSN_EXT_BUF->len << 3) + 8)) {
    switch(LOWSN_EXT_HDR_OPT_BUF->type) {
      /*
       * for now we do not support any options except padding ones
       * PAD1 does not make sense as the header must be 8bytes aligned,
       * hence we can only have
       */
      case LOWSN_EXT_HDR_OPT_PAD1:
        lowsn_ext_opt_offset += 1;
        break;
      case LOWSN_EXT_HDR_OPT_PADN:
        lowsn_ext_opt_offset += LOWSN_EXT_HDR_OPT_PADN_BUF->opt_len + 2;
        break;
        /*
         * check the two highest order bits of the option
         * - 00 skip over this option and continue processing the header.
         * - 01 discard the packet.
         * - 10 discard the packet and, regardless of whether or not the
         *   packet's Destination Address was a multicast address, send an
         *   ICMP Parameter Problem, Code 2, message to the packet's
         *   Source Address, pointing to the unrecognized Option Type.
         * - 11 discard the packet and, only if the packet's Destination
         *   Address was not a multicast address, send an ICMP Parameter
         *   Problem, Code 2, message to the packet's Source Address,
         *   pointing to the unrecognized Option Type.
         */
      default:
      	local = (LOWSN_EXT_HDR_OPT_BUF->type) & 0xC0;
		/*skip this option and cotinue processing the header*/
		if(local == 0x00){
			
            lowsn_ext_opt_offset += LOWSN_EXT_HDR_OPT_BUF->len + 2;
						
			break;
		}
		/*discard the packet and do not send icmp error message*/
		else if(local == 0x40){
			return 1;
		}
		/*discard the packet and send an icmp error message*/
		else if(local == 0x80){
            lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_OPTION,
                             (uint32_t)LOWSN_IPH_LEN + lowsn_ext_len + lowsn_ext_opt_offset);
            return 2;
		}
		/*discard the packet only if the destination address is not a multicast address */
		else if(local == 0xc0){
            if(lowsn_is_addr_mcast(&LOWSN_IP_BUF->destipaddr)) {
              return 1;
            }else{
				lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_OPTION,
                             (uint32_t)LOWSN_IPH_LEN + lowsn_ext_len + lowsn_ext_opt_offset);
				return 2;
            }
		}
/*		
        switch((LOWSN_EXT_HDR_OPT_BUF->type)&0xC0) {
          case 0x00:
            break;
          case 0x40:
            return 1;
          case 0xC0:
            if(lowsn_is_addr_mcast(&LOWSN_IP_BUF->destipaddr)) {
              return 1;
            }else{
				lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_OPTION,
                             (uint32_t)LOWSN_IPH_LEN + lowsn_ext_len + lowsn_ext_opt_offset);
				return 2;
            }
          case 0x80:
            lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_OPTION,
                             (uint32_t)LOWSN_IPH_LEN + lowsn_ext_len + lowsn_ext_opt_offset);
            return 2;
        }
*/
        /* in the cases were we did not discard, update ext_opt* */
  //      lowsn_ext_opt_offset += LOWSN_EXT_HDR_OPT_BUF->len + 2;
        break;
    }
  }
  return 0;
}

/*----------------------------------------------------------*/
/*ressemble the fragment packets*/
/*
	rerurn value:
		0	the fragment has been processed, the lowsn_buf is free;
		1	the fragment worng and an ICMPv6 error message is in the lowsn_buf;
		2	the original packet has been reassembly well and the reassembled packet is in the lowsn_buf;
*/
uint8_t lowsn_reassembly(void){
	static uint32_t prefragid = 0;		//the fragment id that has been reassembled successful or timeout right before
	static uint8_t preidvalid = 0;
	uint16_t len;
	uint16_t offset;
//	if((lowsn_reassbuf.isused==0) && (lowsn_ntohl(LOWSN_FRAG_BUF->id)!=prefragid && preidvalid ==1)){
	if(lowsn_reassbuf.isused==0){
		if(lowsn_ntohl(LOWSN_FRAG_BUF->id)==prefragid && preidvalid ==1){
			lowsn_len  = 0;
			DEBUG_STRING(DBG_INFO, "the reassembling of the fargment has been timeout\r\n");
			return 0;
		}
		DEBUG_STRING(DBG_INFO, "Start reassembly\r\n");
		memcpy(lowsn_reassbuf.buf, LOWSN_IP_BUF, lowsn_ext_len + LOWSN_IPH_LEN);
		stimer_set(&lowsn_reassbuf.reass_timer, LOWSN_REASS_TIMEROUT);
		lowsn_reassbuf.identification = lowsn_ntohl(LOWSN_FRAG_BUF->id);
		prefragid = lowsn_ntohl(LOWSN_FRAG_BUF->id);
		preidvalid = 1;
		lowsn_reassbuf.isused = 1;
	}
	if(lowsn_ds6_ipcmp(&FBUF->srcipaddr, &LOWSN_IP_BUF->srcipaddr) &&
			lowsn_ds6_ipcmp(&FBUF->destipaddr, &LOWSN_IP_BUF->destipaddr) &&
			lowsn_ntohl(LOWSN_FRAG_BUF->id) == lowsn_reassbuf.identification){
		/*get the lentgh of the pragment paylaod*/
		len = lowsn_len - lowsn_ext_len - LOWSN_IPH_LEN - LOWSN_FRAGH_LEN;
		/*get the offset of the fragment*/
		offset = (lowsn_ntohs(LOWSN_FRAG_BUF->offsetresmore) & 0xfff8);
		if(offset > LOWSN_FRAG_BUF_SIZE ||offset + len > LOWSN_FRAG_BUF_SIZE) {
			DEBUG_STRING(DBG_INFO, "fragment too bit\r\n");
			lowsn_reassbuf.isused = 0;
			stimer_set(&lowsn_reassbuf.reass_timer, 0);
			return 0;
		}
		
		/*Recieve the first fragment*/
		if(offset == 0){
			DEBUG_STRING(DBG_INFO, "recieve the first fragment\r\n");
			if(len%8!=0 && ((lowsn_ntohs(LOWSN_FRAG_BUF->offsetresmore) & 0x0001) != 0)){
				lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_HEADER, 4);
				memset(&lowsn_reassbuf, 0, sizeof(lowsn_reassbuf));
				stimer_set(&lowsn_reassbuf.reass_timer, 0);
				return 1;
			}
			*lowsn_next_hdr = LOWSN_FRAG_BUF->next;
			lowsn_reassbuf.firstfrag = 1;
			//get the unfragment len of the original packet
			lowsn_reassbuf.unfraglen = lowsn_ext_len + LOWSN_IPH_LEN;
			lowsn_reassbuf.reassem_len += lowsn_ext_len + LOWSN_IPH_LEN;
			memcpy(FBUF, LOWSN_IP_BUF, lowsn_ext_len + LOWSN_IPH_LEN);
			DEBUG_STRING(DBG_INFO, "src ");
			PRINT6ADDR(&FBUF->srcipaddr);
			DEBUG_STRING(DBG_INFO, "dest ");
			PRINT6ADDR(&FBUF->destipaddr);
			DEBUG_STRING(DBG_INFO, "next %d\r\n", LOWSN_IP_BUF->proto);
		}
		
		/*Recieve the last fragment*/
		if((lowsn_ntohs(LOWSN_FRAG_BUF->offsetresmore) & 0x0001) == 0){
			DEBUG_STRING(DBG_INFO, "recieve the last fragment\r\n");
			lowsn_reassbuf.fraglen = offset + len;	
			lowsn_reassbuf.lastfrag = 1;
		}
		else{
			if(len%8!=0){
				lowsn_icmp6_error_output(ICMP6_PARAM_PROB, ICMP6_PARAMPROB_HEADER, 4);
				memset(&lowsn_reassbuf, 0, sizeof(lowsn_reassbuf));
				stimer_set(&lowsn_reassbuf.reass_timer, 0);
				return 1;
			}
		}
		
		/*calculate the size of the original packet*/
		if((lowsn_reassbuf.orgpktlen == 0)&&(lowsn_reassbuf.lastfrag == 1)&&(lowsn_reassbuf.firstfrag == 1)){
			lowsn_reassbuf.orgpktlen = lowsn_reassbuf.unfraglen + lowsn_reassbuf.fraglen;
		}
		
		/* Copy the fragment into the reassembly buffer, at the right  offset. */
		memcpy((uint8_t *)FBUF + LOWSN_IPH_LEN + lowsn_ext_len + offset,
						(uint8_t *)LOWSN_FRAG_BUF + LOWSN_FRAGH_LEN, len);
		lowsn_reassbuf.reassem_len += len;
		
		/*check if we have a full packet in the buffer*/
		if(lowsn_reassbuf.firstfrag && lowsn_reassbuf.lastfrag){
			if(lowsn_reassbuf.orgpktlen == lowsn_reassbuf.reassem_len){
				memcpy(LOWSN_IP_BUF, FBUF, lowsn_reassbuf.orgpktlen);
				LOWSN_IP_BUF->len[0] = ((lowsn_reassbuf.orgpktlen - LOWSN_IPH_LEN) >> 8);
				LOWSN_IP_BUF->len[1] = ((lowsn_reassbuf.orgpktlen - LOWSN_IPH_LEN) & 0xff);	
				lowsn_len = lowsn_reassbuf.orgpktlen;
				memset(&lowsn_reassbuf, 0, sizeof(lowsn_reassbuf));
				stimer_set(&lowsn_reassbuf.reass_timer, 0);				
				return 2;
			}
		}
		lowsn_len = 0;
		return 0;
	}
	else{
		lowsn_len  = 0;
		DEBUG_STRING(DBG_INFO, "Already reassembling another packet\r\n");
		return 0;
	}
}



void lowsn_reass_timeout(void){

			memcpy(LOWSN_IP_BUF, FBUF, LOWSN_IPH_LEN);
			lowsn_len = LOWSN_IPH_LEN;
			lowsn_ext_len = 0;
			if(lowsn_reassbuf.firstfrag==1){
				lowsn_icmp6_error_output(ICMP6_TIME_EXCEEDED, ICMP6_TIME_EXCEED_REASSEMBLY, 0);
			}
			memset(&lowsn_reassbuf, 0, sizeof(lowsn_reassbuf));
			stimer_set(&lowsn_reassbuf.reass_timer, 0);

}

/**
  * parse the udp header
  * return value:
  * 	0		the checksum is error or the destport of the udp packet is zero
  * 	1		the destination unreachable---port unreachable, and send an icmp error message
  * 	2 		the udp header is parsed successfully and handoff the udp date to TL layer
**/

uint8_t lowsn_udp_parse(void){
	uint8_t i=0;
/*check the checksum in the udp header*/
#if LOWSN_UDP_CHECKSUMS
	if(LOWSN_UDP_BUF->udpchksum != 0 && lowsn_udpchksum() != 0xffff) {
		DEBUG_STRING(DBG_INFO, "udp: bad checksum 0x%04x 0x%04x\r\n", LOWSN_UDP_BUF->udpchksum, lowsn_udpchksum());
		return 0;	//the checksum is error
	}
#endif /* UIP_UDP_CHECKSUMS */

	/*make sure that the udp destination port number is not zero*/
	if(LOWSN_UDP_BUF->destport == 0){
		return 0;
	}

	 /* Demultiplex this UDP packet between the UDP "connections". */
	for(i=0; i<LOWSN_UDP_CONNS; i++){
		if((lowsn_udp_conns[i].lport == LOWSN_UDP_BUF->destport)&&
			(lowsn_udp_conns[i].rport == 0 ||LOWSN_UDP_BUF->srcport == lowsn_udp_conns[i].rport)&&
			(lowsn_is_addr_unspecified(&lowsn_udp_conns[i].ripaddr)||
				lowsn_ds6_ipcmp(&LOWSN_IP_BUF->srcipaddr, &lowsn_udp_conns[i].ripaddr))){
			//Found the udp connection that recieves the udp packet

			//Maybe, we should do some orthers thing at here!!
			
			return 2;
		}
	}
	lowsn_icmp6_error_output(ICMP6_DST_UNREACH, ICMP6_DST_UNREACH_NOPORT, 0);
	return 1;
}


void lowsn_tx_first_frag(void){
	uint8_t* nexthdp;
	uint8_t* bufp;
//	//set the fragment buffer isused flag
//	lowsn_fragbuf.isused = 1;
	//copy the original packet to the fragment buffer.
	memcpy(lowsn_fragbuf.buf, (uint8_t*)LOWSN_IP_BUF,lowsn_len);
	//save the original packet length.
	lowsn_fragbuf.origpktlen = lowsn_len;
	//get the unfragment part lengh of the original packet and
	//the first header of the fargmentable part of the original packet.
	lowsn_fragbuf.unfraglen = LOWSN_IPH_LEN;
	nexthdp = &((lowsn_ip_hdr_t*)lowsn_fragbuf.buf)->proto;
	while(*nexthdp == LOWSN_PROTO_HBHO || *nexthdp == LOWSN_PROTO_ROUTING){
		nexthdp = &lowsn_fragbuf.buf[lowsn_fragbuf.unfraglen];
		lowsn_fragbuf.unfraglen += *(nexthdp+1)*8+8;
	}
	lowsn_fragbuf.next = *nexthdp;
	*nexthdp = LOWSN_PROTO_FRAG;
	//set the fraglen to zero, and generate the identification.
	lowsn_fragbuf.fraglen = 0;
	lowsn_fragbuf.identification = sysclock_get_ms();
	//calculate the max payload length of the fragment packet.
	lowsn_fragbuf.maxpayloadlen =(( pmtu.size - 8 - lowsn_fragbuf.unfraglen)/8)*8;

	/*construct the first fragment of the original packet*/
	//copy the unfragmentable part to the lowsn_buf.
	bufp = (uint8_t*)LOWSN_IP_BUF;
	memcpy(bufp, lowsn_fragbuf.buf, lowsn_fragbuf.unfraglen);
	lowsn_len = lowsn_fragbuf.unfraglen;
	//add fragment header
	bufp += lowsn_fragbuf.unfraglen;
	((lowsn_frag_hdr_t*)bufp)->next = lowsn_fragbuf.next;
	((lowsn_frag_hdr_t*)bufp)->res = 0;
	((lowsn_frag_hdr_t*)bufp)->offsetresmore = lowsn_htons(lowsn_fragbuf.fraglen);
	((lowsn_frag_hdr_t*)bufp)->offsetresmore &= lowsn_htons(~LOWSN_FRAGH_RES_MASK);
	((lowsn_frag_hdr_t*)bufp)->offsetresmore |= lowsn_htons(LOWSN_FRAGH_M_MASK);
	((lowsn_frag_hdr_t*)bufp)->id = lowsn_htonl(lowsn_fragbuf.identification);
	lowsn_len += LOWSN_FRAGH_LEN;
	bufp += LOWSN_FRAGH_LEN;
	//copy the payload to the first fragment paket.
	memcpy(bufp, (uint8_t*)&lowsn_fragbuf.buf[lowsn_fragbuf.unfraglen+lowsn_fragbuf.fraglen], lowsn_fragbuf.maxpayloadlen);	
	lowsn_len += lowsn_fragbuf.maxpayloadlen;		
	lowsn_fragbuf.fraglen += lowsn_fragbuf.maxpayloadlen;
	//change the payload length field of the IPv6 header.
	LOWSN_IP_BUF->len[0] = (lowsn_len-LOWSN_IPH_LEN)>>8;
	LOWSN_IP_BUF->len[1] = (lowsn_len-LOWSN_IPH_LEN) & 0xff;

	//set the fragment flag
	nwk_pib.flags.bits.nwktxfragment = 1;
	DEBUG_STRING(DBG_INFO, "Send the first fragment packet.\r\n");

}

void lowsn_tx_other_frag(void){
	uint8_t* bufp;
	uint16_t lastfpdlen;	//last fragment payload len.
	lastfpdlen = lowsn_fragbuf.origpktlen-lowsn_fragbuf.unfraglen-lowsn_fragbuf.fraglen;
	/*construct the fragment*/
	bufp = (uint8_t*)LOWSN_IP_BUF;
	memcpy(bufp, lowsn_fragbuf.buf, lowsn_fragbuf.unfraglen);
	lowsn_len = lowsn_fragbuf.unfraglen;
	bufp += lowsn_fragbuf.unfraglen;
	//add fragment header
	((lowsn_frag_hdr_t*)bufp)->next = lowsn_fragbuf.next;
	((lowsn_frag_hdr_t*)bufp)->res = 0;
	((lowsn_frag_hdr_t*)bufp)->offsetresmore = lowsn_htons(lowsn_fragbuf.fraglen);
	((lowsn_frag_hdr_t*)bufp)->offsetresmore &= lowsn_htons(~LOWSN_FRAGH_RES_MASK);
	if(lastfpdlen > lowsn_fragbuf.maxpayloadlen){
		((lowsn_frag_hdr_t*)bufp)->offsetresmore |= lowsn_htons(LOWSN_FRAGH_M_MASK);
	}
	else{
		((lowsn_frag_hdr_t*)bufp)->offsetresmore &= lowsn_htons(~LOWSN_FRAGH_M_MASK);
	}
	((lowsn_frag_hdr_t*)bufp)->id = lowsn_htonl(lowsn_fragbuf.identification);
	lowsn_len += LOWSN_FRAGH_LEN;
	bufp += LOWSN_FRAGH_LEN;
	//copy the payload to the fragment.
	if(lastfpdlen > lowsn_fragbuf.maxpayloadlen){
		memcpy(bufp, (uint8_t*)&lowsn_fragbuf.buf[lowsn_fragbuf.unfraglen+lowsn_fragbuf.fraglen], lowsn_fragbuf.maxpayloadlen);	
		lowsn_len += lowsn_fragbuf.maxpayloadlen;		
		lowsn_fragbuf.fraglen += lowsn_fragbuf.maxpayloadlen;
		DEBUG_STRING(DBG_INFO, "Send another fragment packet.\r\n");
	}
	else{
		/*it is the last fragment of the original packet*/
		memcpy(bufp, (uint8_t*)&lowsn_fragbuf.buf[lowsn_fragbuf.unfraglen+lowsn_fragbuf.fraglen], lastfpdlen);	
		lowsn_len += lastfpdlen;		
		lowsn_fragbuf.fraglen += lastfpdlen;
		//reset the fragment flag.
		nwk_pib.flags.bits.nwktxfragment = 0;
		DEBUG_STRING(DBG_INFO, "Send the last fragment packet.\r\n");
	}
	//change the payload length field of the IPv6 header.
	LOWSN_IP_BUF->len[0] = (lowsn_len-LOWSN_IPH_LEN)>>8;
	LOWSN_IP_BUF->len[1] = (lowsn_len-LOWSN_IPH_LEN) & 0xff;

}

#endif



//Callback from ADP Layer
//Returns TRUE if nwk is still busy with last RX packet.

BOOL nwkRxBusy(void){
	return(nwkRxState != NWK_RXSTATE_IDLE);
}

BOOL nwk_buf_free(void){
	return(lowsn_len == 0);
}


void nwkRxHandoff(void){

	// 当适配层传递的是标准IPv6包时，nwkOffset指向IP Header起始位置，pload_offset指向IP Header之后的负载，
	// 扩展头部也算负载。
	// 当适配层传递的是压缩的IPv6包时，压缩IP Header的全部信息在适配层解析出来并传递给NWK，
	// NWK不在负责IP Header的解析，此时nwkOffset和pload_offset都指向IP Header之后的负载部分，也即网络层的
	// 负载与适配层的负载此时是一样的。
	// 当前假定6lowpan适配层不压缩扩展头，也不支持UDP等负载压缩.
	
	a_nwk_rx_data.orgpkt.data = a_adp_rx_data.orgpkt.data;
	a_nwk_rx_data.orgpkt.rssi = a_adp_rx_data.orgpkt.rssi;
	a_nwk_rx_data.nwkOffset = a_adp_rx_data.pload_offset;    // 由于适配层帧头长度不固定，适配层必须告诉网络层起始位置
	a_nwk_rx_data.pload_offset = a_adp_rx_data.pload_offset + LOWSN_IPH_LEN;
	a_nwk_rx_data.hcflag = a_adp_rx_data.hcflag;
	a_nwk_rx_data.srcSADDR = a_adp_rx_data.srcSADDR;

	if (a_adp_rx_data.hcflag == ADP_RX_IP_COMPRESS) {
		a_nwk_rx_data.TrafficClass = a_adp_rx_data.TrafficClass;
		a_nwk_rx_data.FlowLabel = a_adp_rx_data.FlowLabel;
		a_nwk_rx_data.PayloadLength = a_adp_rx_data.PayloadLength;
		a_nwk_rx_data.NextHeader = a_adp_rx_data.NextHeader;
		a_nwk_rx_data.SrcAddress = a_adp_rx_data.SrcAddress;
		a_nwk_rx_data.DstAddress = a_adp_rx_data.DstAddress;
		a_nwk_rx_data.pload_offset = a_adp_rx_data.pload_offset;   //此时网络层不解析IP Header，适配层的负载就是网络层的负载

	#if 0  //for test
	DEBUG_STRING(DBG_INFO,"NWK: Handoff information.  a_nwk_rx_data.TrafficClass:");
	DEBUG_UINT8(DBG_INFO, a_nwk_rx_data.TrafficClass);
	DEBUG_STRING(DBG_INFO,"\n");
	DEBUG_STRING(DBG_INFO,"a_nwk_rx_data.FlowLabel:");
	DEBUG_UINT32(DBG_INFO, a_nwk_rx_data.FlowLabel);
	DEBUG_STRING(DBG_INFO,"\n");
	DEBUG_STRING(DBG_INFO,"a_nwk_rx_data.PayloadLength:");
	DEBUG_UINT16(DBG_INFO, a_nwk_rx_data.PayloadLength);
	DEBUG_STRING(DBG_INFO,"\n");	
	DEBUG_STRING(DBG_INFO,"a_nwk_rx_data.NextHeader:");
	DEBUG_UINT8(DBG_INFO, a_nwk_rx_data.NextHeader);
	DEBUG_STRING(DBG_INFO,"\n");	
	DEBUG_STRING(DBG_INFO,"a_nwk_rx_data.SrcAddress:");
	DEBUG_IP6ADDR(DBG_INFO, &a_nwk_rx_data.SrcAddress, 1);
	DEBUG_STRING(DBG_INFO,"\n");	
	DEBUG_STRING(DBG_INFO,"a_nwk_rx_data.DstAddress:");
	DEBUG_IP6ADDR(DBG_INFO, &a_nwk_rx_data.DstAddress, 1 );
	DEBUG_STRING(DBG_INFO,"\n");	
	DEBUG_STRING(DBG_INFO,"a_nwk_rx_data.pload_offset:");
	DEBUG_UINT8(DBG_INFO, a_nwk_rx_data.pload_offset);
	DEBUG_STRING(DBG_INFO,"\n");
	#endif


	}	

	//lowsn_len = a_nwk_rx_data.adpOffset;
		
	nwkRxState = NWK_RXSTATE_START;
	
}


void nwkParseHdr(BYTE *ptr) {

	BYTE i;

	//为了防止存储大小端和网络大小端造成的移植困难和易出错，在操作时尽量不把指针进行类型强制
	// 转换，例如(UINT16 *)ptr，否则需考虑CC2430是小端存储格式的问题；而是先用BYTE型的指针把
	// char型数据取出来，再做扩展，即(UINT16)*ptr, 扩展后再通过移位操作数据。这种方法不容易出错
	// 即总是一个字节一个字节的取出数据，然后用数据本身的移位来拼接多字节数据；而不图
	// 省事，不用多字节指针一次性取数据。

	// 注意: IETF采用的网络字节序是大端格式，不同于MAC层所用的IEEE 小端网络字节顺序

	a_nwk_rx_data.TrafficClass = ((*ptr) & 0x0F) << 4 | ((*(ptr+1)) & 0xF0) >> 4;
	ptr++;

	a_nwk_rx_data.FlowLabel = (UINT32)((*ptr) & 0x0F) << 16;
	ptr++;
	a_nwk_rx_data.FlowLabel += (UINT32)(*ptr) << 8;
	ptr++;
	a_nwk_rx_data.FlowLabel += *ptr;
	ptr++;

	a_nwk_rx_data.PayloadLength = (UINT16)(*ptr) << 8;
	ptr++;
	a_nwk_rx_data.PayloadLength += *ptr;
	ptr++;

	a_nwk_rx_data.NextHeader = *ptr;
	ptr++;

	a_nwk_rx_data.HopLimit = *ptr;
	ptr++;

	for(i=0; i<16; i++) {
              a_nwk_rx_data.SrcAddress.u8[i] = *ptr;
		ptr++;
       }

	for(i=0; i<16; i++) {
              a_nwk_rx_data.DstAddress.u8[i] = *ptr;
		ptr++;
       }

}



/*------------------------------------------------------------------------------------*

恢复压缩的IP头部，以便发往Host

*------------------------------------------------------------------------------------*/
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
void nwkRestoreCompHeader(void) 
{
	UINT8 i;

	for(i=0; i<LOWSN_IPH_LEN; i++) {
              ip_header_buf[i] = 0;
       }
	
	ip_header_buf[0] = (BYTE)((a_nwk_rx_data.TrafficClass & 0xF0) >> 4) + (BYTE)((LOWSN_IP6_VER & 0x0F) << 4);
	ip_header_buf[1] = ((BYTE) (a_nwk_rx_data.FlowLabel >> 16)) & 0x0F + (BYTE) ((a_nwk_rx_data.TrafficClass & 0x0F) << 4);
	ip_header_buf[2] = (BYTE) (a_nwk_rx_data.FlowLabel >> 8);
	ip_header_buf[3] = (BYTE) (a_nwk_rx_data.FlowLabel);
	ip_header_buf[4] = (BYTE) (a_nwk_rx_data.PayloadLength >> 8);	
	ip_header_buf[5] = (BYTE) (a_nwk_rx_data.PayloadLength);
	ip_header_buf[6] = a_nwk_rx_data.NextHeader;
	ip_header_buf[7] = a_nwk_rx_data.HopLimit;
	
	for(i=0; i<16; i++) {
              ip_header_buf[8+i] = a_nwk_rx_data.SrcAddress.u8[i];
		ip_header_buf[24+i] = a_nwk_rx_data.DstAddress.u8[i];
       }
	
}


void nwkParseSlipHdr(BYTE *ptr) 
{

	BYTE i;

	//为了防止存储大小端和网络大小端造成的移植困难和易出错，在操作时尽量不把指针进行类型强制
	// 转换，例如(UINT16 *)ptr，否则需考虑CC2430是小端存储格式的问题；而是先用BYTE型的指针把
	// char型数据取出来，再做扩展，即(UINT16)*ptr, 扩展后再通过移位操作数据。这种方法不容易出错
	// 即总是一个字节一个字节的取出数据，然后用数据本身的移位来拼接多字节数据；而不图
	// 省事，不用多字节指针一次性取数据。

	// 注意: IETF采用的网络字节序是大端格式，不同于MAC层所用的IEEE 小端网络字节顺序

	a_nwk_tx_data.Version = LOWSN_IP6_VER;

	a_nwk_tx_data.TrafficClass = ((*ptr) & 0x0F) << 4 | ((*(ptr+1)) & 0xF0) >> 4;
	ptr++;

	a_nwk_tx_data.FlowLabel = (UINT32)((*ptr) & 0x0F) << 16;
	ptr++;
	a_nwk_tx_data.FlowLabel += (UINT32)(*ptr) << 8;
	ptr++;
	a_nwk_tx_data.FlowLabel += *ptr;
	ptr++;

	a_nwk_tx_data.PayloadLength = (UINT16)(*ptr) << 8;
	ptr++;
	a_nwk_tx_data.PayloadLength += *ptr;
	ptr++;

	a_nwk_tx_data.NextHeader = *ptr;
	ptr++;

	a_nwk_tx_data.HopLimit = *ptr;
	ptr++;

	for(i=0; i<16; i++) {
              a_nwk_tx_data.SrcAddress.u8[i] = *ptr;
		ptr++;
       }

	for(i=0; i<16; i++) {
              a_nwk_tx_data.DstAddress.u8[i] = *ptr;
		ptr++;
       }

}


// 注意: 未考虑分片问题! 要求主机发来的数据包必须小于128个字节，否则程序会崩溃
void nwkInjectSlipTxPacket(void)
{

	BYTE *src;

	// 若Host发来的IP数据是代表协调器发出的，则有可能会需要地址压缩，
	// 所以还是解析一下相关的信息，方便压缩.
	nwkParseSlipHdr(&slipbuf[0]);

	phy_pib.currentTxFrm = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];
	src = &slipbuf[0] + a_nwk_tx_data.PayloadLength+LOWSN_IPH_LEN;
	phy_pib.currentTxFlen = a_nwk_tx_data.PayloadLength+LOWSN_IPH_LEN;
			
	while (phy_pib.currentTxFlen) {
		src--;                
		phy_pib.currentTxFrm--;     
		phy_pib.currentTxFlen--;   
		*(phy_pib.currentTxFrm) = *src;
	}
		
	phy_pib.currentTxFlen =a_nwk_tx_data.PayloadLength+LOWSN_IPH_LEN;
		

}



BOOL nwkInjectSlipRxPacket(void)
{

	int len;
	len = ((((UINT16)(slipbuf[4])) << 8) | slipbuf[5]) + LOWSN_IPH_LEN;
	
	a_nwk_rx_data.orgpkt.data = MemAlloc(len);
	
	
	if (!a_nwk_rx_data.orgpkt.data ) {
		//can't even get started, return 
		return 1;
	}
	//copy tx buffer into rx space
	halUtilMemCopy(a_nwk_rx_data.orgpkt.data, &slipbuf[0], len);

	a_nwk_rx_data.nwkOffset =0;
	a_nwk_rx_data.pload_offset = LOWSN_IPH_LEN;
	a_nwk_rx_data.hcflag = ADP_RX_IP_UNCOMPRESS;
	a_nwk_rx_data.orgpkt.rssi = 0xFF;  //highest value since coming from wired SLIP

	return 0;

}




#endif
#endif


/*------------------------------------------------------------------------------------*

寻找目标IP地址所对应的链路层地址

在WSN中，由于采用了mesh under策略，可直接由IPv6地址推出对应的
链路层地址，无需再使用NS/NA来寻找，也无需维护IP地址与链路层
地址的对应关系表。

*------------------------------------------------------------------------------------*/
void nwkFindDstDLLAddr(void)
{

	#if 0
	DEBUG_STRING(DBG_INFO,"nwk: Find Link Addr for dstaddr:  ");
    	DEBUG_IP6ADDR(DBG_INFO, &a_nwk_tx_data.DstAddress, 0);
    	DEBUG_STRING(DBG_INFO,"\n");
	#endif


	if(lowsn_is_addr_mcast(&a_nwk_tx_data.DstAddress)){

		if (lowsn_is_addr_linklocal_allrouters_mcast(&a_nwk_tx_data.DstAddress))  {
			
			DEBUG_STRING(DBG_INFO,"NWK: Dest is all-routers multicast address, send to coord. \n");
			// mesh under机制下，对路由器的组播统一送往协调器
			//a_adp_tx_data.dstSADDR  = 0;
			a_adp_tx_data.dstSADDR = LOWSN_BCAST_SADDR;
		}

		else  {			
			DEBUG_STRING(DBG_INFO,"NWK: Dest is multi addr.\n");

			//nwkIP2MADDR(&a_nwk_tx_data.DstAddress, a_adp_tx_data.dstSADDR);
                            // RFC4944 单独对IEEE 802.15.4的短地址进行了扩展定义，其中100打头的是组播
                            // 原始的IEEE 802.15.4并不支持组播，协议栈中的MAC也不支持组播
                            // 所以可暂时把组播认为是广播, 虽然组播转换函数工作正确
			a_adp_tx_data.dstSADDR = LOWSN_BCAST_SADDR;
				
		}	
	}	

	else if (ds6TestOnLink(&a_nwk_tx_data.DstAddress) == 1){
				// 同一前缀的地址，包括link-local地址，可以直接推出链路层地址

		//DEBUG_STRING(DBG_INFO,"Dest addr is ON-LINK.\n");

		if (lowsn_is_addr_genfrom_shortaddr(&a_nwk_tx_data.DstAddress)) {

			ds6IPtoSAddr(&a_nwk_tx_data.DstAddress, a_adp_tx_data.dstSADDR);
					
			//DEBUG_STRING(DBG_INFO, "\n Dst Short ADDR:   ");
			//DEBUG_UINT16(DBG_INFO, a_adp_tx_data.dstSADDR);
			//DEBUG_STRING(DBG_INFO, "\n");

					
		}
		else if (lowsn_is_addr_genfrom_EUI64(&a_nwk_tx_data.DstAddress)) {
			
			a_adp_tx_data.dstSADDR = LOWSN_SADDR_USE_LADDR;
			ds6IPtoLAddr(&a_nwk_tx_data.DstAddress, &a_adp_tx_data.dstLADDRFull);	
			a_adp_tx_data.dstLADDR = &a_adp_tx_data.dstLADDRFull.bytes[0];

			DEBUG_STRING(DBG_INFO, "\n Dst long ADDR \n");
			DEBUG_LADDR(DBG_INFO, &a_adp_tx_data.dstLADDRFull);
			DEBUG_STRING(DBG_INFO, "\n");

		}

		else {
			a_adp_tx_data.dstSADDR  = 0;  //送往协调器
			DEBUG_STRING(DBG_INFO, "\n Dst on LINK to 0. \n");
			
		}
	}

	else {
		// 外网地址，或一跳范围外的地址，目标地址都设为协调器，由适配层去路由			

		DEBUG_STRING(DBG_INFO,"Dst is not multicast or ON-LINK addr.\n");
		a_adp_tx_data.dstSADDR  = 0;

	}
}



