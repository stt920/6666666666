/*********************************************************************
    文件名：ds.c
    作  者：ji shanyang
    版  本：v1.0.0
    日  期：2012.12
    描  述：define the generic data structure used in 6LoWSN stack
*********************************************************************/
#include <string.h>
#include <stdlib.h>
//#include <stdin.h>
#include <stddef.h>
#include "6lowsn_common_types.h"
#include "6lowsn_config.h"
#include "console.h"
#include "debug.h"
#include "halstack.h"
#include "ds.h"
#include "nd.h"
#include "nwk.h"
#include "mac.h"
#include "neighbor.h"



/**
  * @{\* define the data structures used by neighbor discovery protocol
**/
//lowsn_ds6_schedule_t lowsn_ds6_schedule;	//the envet scheduling data structure
lowsn_ds6_netif_t lowsn_ds6_if;		//brief the single ipv6 network interface

lowsn_ds6_prefix_t lowsn_ds6_prefix_list[LOWSN_DS6_PREFIX_NB];	//brief prefix list


uint8_t lladdr_dadfail = 0;	//--1 indicate the dad fail;  0 indicate the dad succes

//systime_s_t internalclock;
//systime_ms_t internalclock_ms;

//lowsn_ds6_nbr_t lowsn_ds6_nbr_cache[LOWSN_DS6_NBR_NB];	//brief neighbor cache
//lowsn_ds6_dest_t lowsn_ds6_dest_cache[LOWSN_DS6_DEST_NB];	//brief destination cache
//lowsn_ds6_defrt_t lowsn_ds6_defrt_list[LOWSN_DS6_DEFRT_NB];		//brief default router list


/**
  * @}\*
**/





/* "full" (as opposed to pointer) ip address used in this file,  */
static lowsn_ipaddr_t loc_fipaddr;

/* Pointers used in this file */
//static lowsn_ipaddr_t *locipaddr;		
static lowsn_ds6_uaddr_t *locuaddr;	
static lowsn_ds6_maddr_t *locmaddr;
static lowsn_ds6_aaddr_t *locaaddr;
static lowsn_ds6_prefix_t *locprefix;
//static lowsn_ds6_nbr_t *locnbr;
//static lowsn_ds6_defrt_t *locdefrt;
//static lowsn_ds6_dest_t *locdest;

/*extern variable*/
extern lowsn_ds6_pmtu_t pmtu;
//extern lowsn_lladdr_t lowsn_lladdr;
extern lowsn_ds6_reassbuf_t lowsn_reassbuf;

lowsn_lladdr_t lowsn_lladdr;



/*---------------------------------------------------------------------*
初始化系统的IPv6地址及其结构
(不包含短地址生成的link-local地址的初始化，该部分需在
获取短地址之后再运行)



*---------------------------------------------------------------------*/
void ds6Init(void)
{


	DEBUG_STRING(DBG_INFO,"ds6 init. \n");
	
	memset(&lowsn_ds6_if, 0, sizeof(lowsn_ds6_if));
	memset(lowsn_ds6_prefix_list, 0, sizeof(lowsn_ds6_prefix_list));

	lowsn_ds6_if.link_mtu = LOWSN_LINK_MTU;
	lowsn_ds6_if.cur_hop_limit = LOWSN_TTL;
	lowsn_ds6_if.base_reachable_time = (LOWSN_ND6_REACHABLE_TIME);
	lowsn_ds6_if.retrans_tminterval = LOWSN_ND6_RETRANS_TIMER;
	lowsn_ds6_if.maxdadns = LOWSN_ND6_DEF_MAXDADNS;
	lowsn_ds6_if.iid_gen_method = LOWSN_IID_METHOD_FOR_GLOABL;

	pmtu.size = lowsn_ds6_if.link_mtu;

	lowsn_create_linklocal_prefix(&loc_fipaddr);
#ifdef LOWSN_FFD
	lowsn_ds6_prefix_add(&loc_fipaddr, LOWSN_DEFAULT_PREFIX_LEN, 0,0,0,0);
#else
	lowsn_ds6_prefix_add(&loc_fipaddr, LOWSN_DEFAULT_PREFIX_LEN, 0);
#endif
	ds6GenInterfaceID64(&loc_fipaddr);
	lowsn_ds6_uaddr_add(&loc_fipaddr, 0, ADDR_AUTOCONF_EUI64);

	/*join the all-nodes multicast address*/
	lowsn_create_linklocal_allnodes_mcast(&loc_fipaddr);
	lowsn_ds6_maddr_add(&loc_fipaddr);

#ifdef LOWSN_FFD
	/*(Router Only) creat link local all router multicast address*/
	lowsn_create_linklocal_allrouters_mcast(&loc_fipaddr);
	lowsn_ds6_maddr_add(&loc_fipaddr);
#else
	lowsn_ds6_if.rscount = 0;
#endif
	

}


/*---------------------------------------------------------------------*

构造来自于短地址和PANID的link-local IPv6地址
在组网完成，入网或重新入网完成调用。


*---------------------------------------------------------------------*/
void ds6LocalIPFrom16(void)
{

	DEBUG_STRING(DBG_INFO,"set iid from short address. \n");
	// 添加由短地址生成的Interface ID
	lowsn_create_linklocal_prefix(&loc_fipaddr);
	ds6GenInterfaceID16(&loc_fipaddr, macGetPanID(), macGetShortAddr());
	lowsn_ds6_uaddr_add(&loc_fipaddr, 0, ADDR_AUTOCONF_SHORTADDR);

}

/*---------------------------------------------------------------------*/
/** \brief Generic loop routine on an abstract data structure, which generalizes
 * all data structures used in DS6
 * arguments:	list			the pointer pint to the header of the list
 *			size			the size of the list
 *			elementsize	the size of the list element
 *			ipaddr		ipv6 address
 *			ipaddrlen		the length of the ipv6 address
 *			out_element	the pointer point to the found element
 *
 * rev:		FOUND		indicate that we have found a suitable element, point by out_element
 *			FREESPACE	indicate that we have found a free space in the list, point by out_element
 *			NOSPACE		indicate that the list is full, the arg out_element equal NULL
 */
uint8_t lowsn_ds6_list_loop(lowsn_ds6_element_t *list, uint8_t size,
                  uint16_t elementsize, lowsn_ipaddr_t *ipaddr,
                  uint8_t ipaddrlen, lowsn_ds6_element_t **out_element)
{
  lowsn_ds6_element_t *element;

  *out_element = NULL;

  for(element = list; element <(lowsn_ds6_element_t *)((uint8_t *)list + (size * elementsize));
      element = (lowsn_ds6_element_t *)((uint8_t *)element + elementsize)) {
    if(element->isused) {
      if(lowsn_ipaddr_prefixcmp(&element->ipaddr, ipaddr, ipaddrlen)) {
        *out_element = element;
        return FOUND;
      }
    } else {
      *out_element = element;
    }
  }

  return *out_element != NULL ? FREESPACE : NOSPACE;
}


/*---------------------------------------------------------------------------*/
/** \brief add a new entry to the prefix list*/

#ifdef LOWSN_FFD
/**
  * the add prefix funtion used by router is defferent from that uesd by host,
  * so we should implement the fuction separately
*/
lowsn_ds6_prefix_t * lowsn_ds6_prefix_add(lowsn_ipaddr_t *ipaddr, uint8_t ipaddrlen,
                   uint8_t advertise, uint8_t flags, unsigned long vtime,
                   unsigned long ptime)
{

    if(lowsn_ds6_list_loop
     ((lowsn_ds6_element_t *)lowsn_ds6_prefix_list, LOWSN_DS6_PREFIX_NB,
      sizeof(lowsn_ds6_prefix_t), ipaddr, ipaddrlen,
      (lowsn_ds6_element_t **)&locprefix) == FREESPACE) {
    locprefix->isused = 1;
    lowsn_ipaddr_copy(&locprefix->ipaddr, ipaddr);
    locprefix->length = ipaddrlen;
    locprefix->advertise = advertise;
    locprefix->l_a_reserved = flags;
    locprefix->vlifetime = vtime;
    locprefix->plifetime = ptime;
	
    DEBUG_STRING(DBG_INFO,"Add a prefix:  ");
    DEBUG_IP6ADDR(DBG_INFO, &locprefix->ipaddr, 0);
    DEBUG_STRING(DBG_INFO,"Prefix Len: ");
    DEBUG_UINT16(DBG_INFO, ipaddrlen);
    DEBUG_STRING(DBG_INFO,"\n");
	
    return locprefix;
  } else {
    DEBUG_STRING(DBG_INFO, "No more space in Prefix list\n");
  }
  return NULL;

}

#else
lowsn_ds6_prefix_t *lowsn_ds6_prefix_add(lowsn_ipaddr_t *ipaddr, uint8_t ipaddrlen,
                   unsigned long interval)
{

	if(lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_prefix_list, LOWSN_DS6_PREFIX_NB,
		   sizeof(lowsn_ds6_prefix_t), ipaddr, ipaddrlen,
		(lowsn_ds6_element_t **)&locprefix) == FREESPACE) {
		locprefix->isused = 1;
		lowsn_ipaddr_copy(&locprefix->ipaddr, ipaddr);
		locprefix->length = ipaddrlen;
		if(interval != 0) {
			//stimer_set(&locprefix->invalidtime, interval);
			locprefix->isinfinite = 0;
		}
		else {
			locprefix->isinfinite = 1;
		}

		DEBUG_STRING(DBG_INFO,"Add a prefix:  ");
		DEBUG_IP6ADDR(DBG_INFO, &locprefix->ipaddr, 0);
		DEBUG_STRING(DBG_INFO,"Prefix Len: ");
		DEBUG_UINT16(DBG_INFO, ipaddrlen);
		DEBUG_STRING(DBG_INFO,"\n");
	}
	return NULL;
}
#endif

/*---------------------------------------------------------------------------*/
/** \brief remove an entry in the prefix list*/
void lowsn_ds6_prefix_rm(lowsn_ds6_prefix_t *prefix)
{
	if(prefix != NULL) {
		prefix->isused = 0;
		prefix->invalidtime = 0;
	}

	DEBUG_STRING(DBG_INFO,"Remove a prefix:  ");
	DEBUG_IP6ADDR(DBG_INFO, &prefix->ipaddr, 0);
	DEBUG_STRING(DBG_INFO,"Prefix Len: ");
	DEBUG_UINT16(DBG_INFO, prefix->length);
	DEBUG_STRING(DBG_INFO,"\n");

	return;
}


/*---------------------------------------------------------------------------*/
/** \brief lookup an entry in the prefix list*/
lowsn_ds6_prefix_t * lowsn_ds6_prefix_lookup(lowsn_ipaddr_t *ipaddr, uint8_t ipaddrlen)
{
    if(lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_prefix_list,
		       LOWSN_DS6_PREFIX_NB, sizeof(lowsn_ds6_prefix_t),
		       ipaddr, ipaddrlen,
		       (lowsn_ds6_element_t **)&locprefix) == FOUND) {
    return locprefix;
  }
  return NULL;
}


/*-----------------------------------------------------------------------*

 添加一个新的单播地址到单播地址列表


*-----------------------------------------------------------------------*/
lowsn_ds6_uaddr_t * lowsn_ds6_uaddr_add(lowsn_ipaddr_t *ipaddr, unsigned long vlifetime, uint8_t type)
{
  if(lowsn_ds6_list_loop
     ((lowsn_ds6_element_t *)lowsn_ds6_if.uaddr_list, LOWSN_DS6_UADDR_NB,
      sizeof(lowsn_ds6_uaddr_t), ipaddr, 128,
      (lowsn_ds6_element_t **)&locuaddr) == FREESPACE) {
    locuaddr->isused = 1;
    lowsn_ipaddr_copy(&locuaddr->ipaddr, ipaddr);
    locuaddr->type = type;

    // 暂时不考虑时间，永远有效	
    locuaddr->state = ADDR_PREFERRED;
	
    if(vlifetime == 0) {
      locuaddr->isinfinite = 1;
    } else {
      locuaddr->isinfinite = 0;
    }

    DEBUG_STRING(DBG_INFO,"Add a uaddr:  ");
    DEBUG_IP6ADDR(DBG_INFO, &locuaddr->ipaddr, 0);
    DEBUG_STRING(DBG_INFO,"type: ");
    DEBUG_UINT16(DBG_INFO, locuaddr->type);
    DEBUG_STRING(DBG_INFO,"\n");

    lowsn_create_solicited_node(ipaddr, &loc_fipaddr);
    lowsn_ds6_maddr_add(&loc_fipaddr);
    return locuaddr;
  }
  return NULL;
}


/*---------------------------------------------------------------------------*/
/**
  * \brief add a new unicast address to the unicast address list
**/
void lowsn_ds6_uaddr_rm(lowsn_ds6_uaddr_t *addr)
{
  if(addr != NULL) {
    lowsn_create_solicited_node(&addr->ipaddr, &loc_fipaddr);
    if((locmaddr = lowsn_ds6_maddr_lookup(&loc_fipaddr)) != NULL) {
      lowsn_ds6_maddr_rm(locmaddr);
    }
    addr->isused = 0;
  }
  return;
}


/*---------------------------------------------------------------------------*/
/**
  * \brief lookup a unicast address in the unicast address list
**/
lowsn_ds6_uaddr_t * lowsn_ds6_uaddr_lookup(lowsn_ipaddr_t *ipaddr)
{
  if(lowsn_ds6_list_loop
     ((lowsn_ds6_element_t *)lowsn_ds6_if.uaddr_list, LOWSN_DS6_UADDR_NB,
      sizeof(lowsn_ds6_uaddr_t), ipaddr, 128,
      (lowsn_ds6_element_t **)&locuaddr) == FOUND) {
    return locuaddr;
  }
  return NULL;
}



/*---------------------------------------------------------------------------*/
/**
  * \brief add a new mulitcast address to the mulitcast address list
**/
lowsn_ds6_maddr_t * lowsn_ds6_maddr_add(lowsn_ipaddr_t *ipaddr)
{
	uint8_t ret;
	ret = lowsn_ds6_list_loop	((lowsn_ds6_element_t *)lowsn_ds6_if.maddr_list, LOWSN_DS6_MADDR_NB,
												sizeof(lowsn_ds6_maddr_t), ipaddr, 128, (lowsn_ds6_element_t **)&locmaddr);
	if(ret == FOUND){
		locmaddr->assoc_count++;
		return locmaddr;
	}
	else if(ret == FREESPACE) {
		locmaddr->isused = 1;
		lowsn_ipaddr_copy(&locmaddr->ipaddr, ipaddr);
		locmaddr->assoc_count++;
		return locmaddr;
	}
	return NULL;
}



/*---------------------------------------------------------------------------*/
/**
  * \brief rmove a multicast address from the mulitcast address list
**/
void lowsn_ds6_maddr_rm(lowsn_ds6_maddr_t *maddr)
{
	if(maddr != NULL) {
		maddr->assoc_count--;
		if(maddr->assoc_count==0){
			maddr->isused = 0;
		}
	}
	return;
}

/*---------------------------------------------------------------------------*/
/**
  * \brief lookup a multicast address in the mulitcast address list
**/
lowsn_ds6_maddr_t * lowsn_ds6_maddr_lookup(lowsn_ipaddr_t *ipaddr)
{
  if(lowsn_ds6_list_loop
     ((lowsn_ds6_element_t *)lowsn_ds6_if.maddr_list, LOWSN_DS6_MADDR_NB,
      sizeof(lowsn_ds6_maddr_t), ipaddr, 128,
      (lowsn_ds6_element_t **)&locmaddr) == FOUND) {
    return locmaddr;
  }
  return NULL;
}


/*---------------------------------------------------------------------------*/
lowsn_ds6_aaddr_t * lowsn_ds6_aaddr_add(lowsn_ipaddr_t *ipaddr)
{
  if(lowsn_ds6_list_loop
     ((lowsn_ds6_element_t *)lowsn_ds6_if.aaddr_list, LOWSN_DS6_AADDR_NB,
      sizeof(lowsn_ds6_aaddr_t), ipaddr, 128,
      (lowsn_ds6_element_t **)&locaaddr) == FREESPACE) {
    locaaddr->isused = 1;
    lowsn_ipaddr_copy(&locaaddr->ipaddr, ipaddr);
    return locaaddr;
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
void lowsn_ds6_aaddr_rm(lowsn_ds6_aaddr_t *aaddr)
{
  if(aaddr != NULL) {
    aaddr->isused = 0;
  }
  return;
}

/*---------------------------------------------------------------------------*/
lowsn_ds6_aaddr_t * lowsn_ds6_aaddr_lookup(lowsn_ipaddr_t *ipaddr)
{
  if(lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_if.aaddr_list,
		       LOWSN_DS6_AADDR_NB, sizeof(lowsn_ds6_aaddr_t), ipaddr, 128,
		       (lowsn_ds6_element_t **)&locaaddr) == FOUND) {
    return locaaddr;
  }
  return NULL;
}




/*-----------------------------------------------------------------------*

判断一个IP地址是否是on-link

1: on-link; 0: off-link

on-link是指该IP地址的前缀与本节点内部所存的前缀列表中的
某一前缀完全相同，即相同的位数等于前缀的位数。

在原始IPv6协议中，前缀相同表明在同一个路由器内，
在WSN中，前缀相同，不论是link-local的前缀相同还是global的前缀
相同，都表明是在WSN内部。


*-----------------------------------------------------------------------*/
uint8_t ds6TestOnLink(lowsn_ipaddr_t *ipaddr)
{
  for(locprefix = lowsn_ds6_prefix_list;
      locprefix < lowsn_ds6_prefix_list + LOWSN_DS6_PREFIX_NB; locprefix++) {
    if(locprefix->isused &&
       lowsn_ipaddr_prefixcmp(&locprefix->ipaddr, ipaddr, locprefix->length)) {
      return 1;
    }
  }
  return 0;
}



/*---------------------------------------------------------------------------*/
/**
根据目标地址的类型选择源地址


*-----------------------------------------------------------------------*/
void ds6FindSrcIP(lowsn_ipaddr_t *src, lowsn_ipaddr_t *dst)
{
  uint8_t best = 0;             /* number of bit in common with best match */
  uint8_t n = 0;
  lowsn_ds6_uaddr_t *matchaddr = NULL;

// 注意: 对于on-link的地址，无论对方是长地址生成的还是短地址生成的，
// 本机地址一律采用短地址生成的进行回复.
//  if(!lowsn_is_addr_link_local(dst) && !lowsn_is_addr_mcast(dst)) {
  if((!lowsn_is_addr_link_local(dst) && !lowsn_is_addr_mcast(dst))||lowsn_is_addr_gmcast(dst)) {
    /* find longest match */
    for(locuaddr = lowsn_ds6_if.uaddr_list;
        locuaddr < lowsn_ds6_if.uaddr_list + LOWSN_DS6_UADDR_NB; locuaddr++) {
      /* Only preferred global (not link-local) addresses */
      if(locuaddr->isused && locuaddr->state == ADDR_PREFERRED &&
         !lowsn_is_addr_link_local(&locuaddr->ipaddr)) {
        n = ds6MatchNum(dst, &locuaddr->ipaddr);
        if(n >= best) {
          best = n;
          matchaddr = locuaddr;
        }
      }
    }
  }
  else {

	#if LOWSN_SRC_IP_MODE == 1 
		matchaddr = ds6FindLinkLocalIP(ADDR_PREFERRED, ADDR_AUTOCONF_SHORTADDR);
	#elif LOWSN_SRC_IP_MODE == 2	
		matchaddr = ds6FindLinkLocalIP(ADDR_PREFERRED, ADDR_AUTOCONF_EUI64);
	#else
		 if (lowsn_is_addr_genfrom_EUI64(dst))  {
			matchaddr = ds6FindLinkLocalIP(ADDR_PREFERRED, ADDR_AUTOCONF_EUI64);
		
		}
		else {	
    			matchaddr = ds6FindLinkLocalIP(ADDR_PREFERRED, ADDR_AUTOCONF_SHORTADDR);
		}
	#endif

	// 若对方是link-local的长地址生成的IPv6地址，则本机回复时也选用
	// 该类地址. 注意: 长地址不能进行MESH路由，仅能用于星形网.
	// 其他情况均用短地址回复.
	//if (lowsn_is_addr_genfrom_EUI64(dst))  {
	//	matchaddr = ds6FindLinkLocalIP(ADDR_PREFERRED, ADDR_AUTOCONF_EUI64);
	//	
	//}
	//else {	
    	//	matchaddr = ds6FindLinkLocalIP(ADDR_PREFERRED, ADDR_AUTOCONF_SHORTADDR);
	//}		
	
  }

  /* use the :: (unspecified address) as source if no match found */
  if(matchaddr == NULL) {
  	
    DEBUG_STRING(DBG_INFO,"ds: Can not find matched src, use unspecified address. \n ");
    lowsn_create_unspecified(src);
  } else {
    lowsn_ipaddr_copy(src, &matchaddr->ipaddr);
  }
}



/*---------------------------------------------------------------------------*

获取一个符合要求的link-local IPv6地址
 * state = -1 => any address is ok. Otherwise state = desired state of addr.
 * (TENTATIVE, PREFERRED, DEPRECATED)
  *type = -1 => any address is ok. Otherwise type = desired state of addr.
 * (ADDR_ANYTYPE, ADDR_MANUAL, ADDR_AUTOCONF_EUI48, ADDR_AUTOCONF_EUI64,
   ADDR_AUTOCONF_SHORTADDR, ADDR_AUTOCONF_OTHER)

---------------------------------------------------------------------------*/

lowsn_ds6_uaddr_t * ds6FindLinkLocalIP(int8_t state, int8_t type)
{
  for(locuaddr = lowsn_ds6_if.uaddr_list;
      locuaddr < lowsn_ds6_if.uaddr_list + LOWSN_DS6_UADDR_NB; locuaddr++) {
    if(locuaddr->isused && (state == -1 || locuaddr->state == state) && (type == -1 || locuaddr->type == type)
       && (lowsn_is_addr_link_local(&locuaddr->ipaddr))) {
      return locuaddr;
    }
  }
  return NULL;
}


/*---------------------------------------------------------------------------*/
/*
获取一个global IPv6地址
 * state = -1 => any address is ok. Otherwise state = desired state of addr.
 * (TENTATIVE, PREFERRED, DEPRECATED)
   *type = -1 => any address is ok. Otherwise type = desired state of addr.
 * (ADDR_ANYTYPE, ADDR_MANUAL, ADDR_AUTOCONF_EUI48, ADDR_AUTOCONF_EUI64,
   ADDR_AUTOCONF_SHORTADDR, ADDR_AUTOCONF_OTHER)
---------------------------------------------------------------------------*/

lowsn_ds6_uaddr_t * ds6FindGlobalIP(int8_t state,  int8_t type)
{
  for(locuaddr = lowsn_ds6_if.uaddr_list;
      locuaddr < lowsn_ds6_if.uaddr_list + LOWSN_DS6_UADDR_NB; locuaddr++) {
    if(locuaddr->isused && (state == -1 || locuaddr->state == state) && (type == -1 || locuaddr->type == type)
       && !(lowsn_is_addr_link_local(&locuaddr->ipaddr))) {
      return locuaddr;
    }
  }
  return NULL;
}



/*---------------------------------------------------------------------------*

由链路层地址生成IPv6地址的interface ID

genmethod: 0: 由64位长地址生成; 1或其它: 由48位MAC地址生成

通用函数，为了兼容时才使用

*---------------------------------------------------------------------------*/
void ds6GenInterfaceID_EUI(lowsn_ipaddr_t *ipaddr, BYTE *lladdr, uint8_t genmethod)
{

  if (genmethod == 0)  {	
  	memcpy(ipaddr->u8 + 8, lladdr, LOWSN_LLADDR_LEN);
  	ipaddr->u8[8] ^= 0x02;
  }
  else  {
  	 memcpy(ipaddr->u8 + 8, lladdr, 3);
 	 ipaddr->u8[11] = 0xff;
 	 ipaddr->u8[12] = 0xfe;
 	 memcpy(ipaddr->u8 + 13, (uint8_t *)lladdr + 3, 3);
  	 ipaddr->u8[8] ^= 0x02;
  }	

}



/*---------------------------------------------------------------------------*

由EUI64地址直接生成IPv6地址的interface ID，仅用于IEEE 802.15.4网络

直接获取本地EUI64地址生成

为了本地生成方便而设定的函数，更通用的函数是ds6GenInterfaceID_EUI

*---------------------------------------------------------------------------*/
void ds6GenInterfaceID64(lowsn_ipaddr_t *ipaddr)
{

	halGetProcessorIEEEAddress(ipaddr->u8 + 8);
  	ipaddr->u8[8] ^= 0x02;

}



/*---------------------------------------------------------------------------*

由短地址生成IPv6地址的interface ID，仅用于IEEE 802.15.4网络
RFC4944  pp. 13

*---------------------------------------------------------------------------*/
void ds6GenInterfaceID16(lowsn_ipaddr_t *ipaddr, UINT16 panid, UINT16 short_addr)
{

	//ipaddr->u8[8] = (uint8_t)(macGetPanID() >> 8);
	//ipaddr->u8[9] = (uint8_t)(macGetPanID());
	ipaddr->u8[8] = (uint8_t)(panid >> 8);
	ipaddr->u8[9] = (uint8_t)(panid);
	ipaddr->u8[10] = 0;
	ipaddr->u8[11] = 0xFF;
	ipaddr->u8[12] = 0xFE;
	ipaddr->u8[13] = 0;
	//ipaddr->u8[14] = (uint8_t)(macGetShortAddr() >> 8);
	//ipaddr->u8[15] = (uint8_t)(macGetShortAddr());
	ipaddr->u8[14] = (uint8_t)(short_addr >> 8);
	ipaddr->u8[15] = (uint8_t)(short_addr);
	
 	//the "Universal/Local" (U/L) bit SHALL be set to zero in keeping with the fact that this is not a globally unique value.
	ipaddr->u8[8] &= 0xFD;

}


/*---------------------------------------------------------------------------*

寻找目标地址与某个备选源地址前缀匹配的位数

*---------------------------------------------------------------------------*/
uint8_t ds6MatchNum(lowsn_ipaddr_t *src, lowsn_ipaddr_t *dst)
{
  uint8_t j, k, x_or;
  uint8_t len = 0;

  for(j = 0; j < 16; j++) {
    if(src->u8[j] == dst->u8[j]) {
      len += 8;
    } else {
      x_or = src->u8[j] ^ dst->u8[j];
      for(k = 0; k < 8; k++) {
        if((x_or & 0x80) == 0) {
          len++;
          x_or <<= 1;
        } else {
          break;
        }
      }
      break;
    }
  }
  return len;
}



/*---------------------------------------------------------------------------*/
/**
  *  Convert 16-bit quantity from host byte order to network byte order.
**/

uint16_t
lowsn_htons(uint16_t val)
{
  return LOWSN_HTONS(val);
}

uint32_t
lowsn_htonl(uint32_t val)
{
  return LOWSN_HTONL(val);
}






uint8_t lowsn_ds6_ipcmp(lowsn_ipaddr_t* srcip, lowsn_ipaddr_t* destip){
	uint8_t i;
	for(i=0; i<16 ; i++){
		if(srcip->u8[i] != destip->u8[i]){
			return 0;
		}
	}
	return 1;
}


uint8_t lowsn_ds6_lladdr_cmp(lowsn_lladdr_t* srcll, lowsn_lladdr_t* destll){
	uint8_t i;
	for(i=0; i<LOWSN_LLADDR_LEN; i++){
		if(srcll->addr[i] != destll->addr[i]){
			return 0;
		}
	}
	return 1;
}



#if 0


void lowsn_ds6_schedule_ms(systime_ms_t internal_clock_ms){
	int i;
	lowsn_ds6_defrt_t* tmpdefrt = NULL;
	/*Event 1--Send NS Message For Address Resolution And Duplicate Address Detectation*/
	for(i=0;i<LOWSN_DS6_NBR_NB;i++){
		if(lowsn_ds6_nbr_cache[i].isused && (lowsn_ds6_nbr_cache[i].sendns != 0)){
			if( lowsn_ds6_nbr_cache[i].sendns <= internal_clock_ms){
				//the entry is doing address resolution
				if(lowsn_ds6_nbr_cache[i].state== NBR_INCOMPLETE){
					if(lowsn_ds6_nbr_cache[i].nscount== LOWSN_ND6_MAX_MULTICAST_SOLICIT){
						DEBUG_STRING(DBG_INFO, "Addr Resolution Fail\r\n");
						lowsn_ds6_nbr_rm(&lowsn_ds6_nbr_cache[i]);
						/*update the destination cache table*/
						lowsn_ds6_dest_update(&lowsn_ds6_nbr_cache[i].ipaddr);
					}
					else{
						mstimer_set(&lowsn_ds6_nbr_cache[i].sendns, lowsn_ds6_if.retrans_tminterval);
						lowsn_ds6_nbr_cache[i].nscount++;
						lowsn_nd_ns_output(NULL, NULL, &lowsn_ds6_nbr_cache[i].ipaddr);
						DEBUG_STRING(DBG_INFO, "Retransmit a NS for Addr Resolution\r\n");
						return;
					}
				}
				//the entry is doing DAD
				else if(lowsn_ds6_nbr_cache[i].state== NBR_DELAY){
					lowsn_ds6_nbr_cache[i].state = NBR_PROBE;
					mstimer_set(&lowsn_ds6_nbr_cache[i].sendns, lowsn_ds6_if.retrans_tminterval);
					lowsn_ds6_nbr_cache[i].nscount++;
					lowsn_nd_ns_output(NULL, &lowsn_ds6_nbr_cache[i].ipaddr, &lowsn_ds6_nbr_cache[i].ipaddr);
					DEBUG_STRING(DBG_INFO, "Transmit a NS for NURD\r\n");
					return;
				}
				else if(lowsn_ds6_nbr_cache[i].state== NBR_PROBE){
					if(lowsn_ds6_nbr_cache[i].nscount== LOWSN_ND6_MAX_UNICAST_SOLICIT){
						DEBUG_STRING(DBG_INFO, "NURD Fail update DTC and remove the NBC\r\n");
						/*update the destination cache table*/
						lowsn_ds6_dest_update(&lowsn_ds6_nbr_cache[i].ipaddr);
//						for(j=0; j<LOWSN_DS6_DEST_NB; j++){
//							if(lowsn_ds6_dest_cache[j].isused &&
//										(lowsn_ds6_ipcmp(&lowsn_ds6_dest_cache[j].nexthop, &lowsn_ds6_nbr_cache[i].ipaddr))){
//								lowsn_ds6_dest_rm(&lowsn_ds6_dest_cache[j]);
//							}
//						}
						/*if the neighbor is a default router set the fail flag*/
						tmpdefrt=lowsn_ds6_defrt_lookup(&lowsn_ds6_nbr_cache[i].ipaddr);
						if(tmpdefrt != NULL){
							tmpdefrt->fail =1;
						}
						/*remove the neighbor from the neighbor cache table*/
						lowsn_ds6_nbr_rm(&lowsn_ds6_nbr_cache[i]);
					}
					else{
						mstimer_set(&lowsn_ds6_nbr_cache[i].sendns, lowsn_ds6_if.retrans_tminterval);
						lowsn_ds6_nbr_cache[i].nscount++;
						lowsn_nd_ns_output(NULL,&lowsn_ds6_nbr_cache[i].ipaddr, &lowsn_ds6_nbr_cache[i].ipaddr);
						DEBUG_STRING(DBG_INFO, "Retransmit a NS for NURD\r\n");
						return;
					}
				}
			}
		}
	}

	/*Event 2--Send NS Message For DAD*/
	if(lladdr_dadfail==0){
		for(i=0; i<LOWSN_DS6_UADDR_NB; i++){
			if(lowsn_ds6_if.uaddr_list[i].isused && (lowsn_ds6_if.uaddr_list[i].dadtimer != 0)){
				if(lowsn_ds6_if.uaddr_list[i].dadtimer <= internal_clock_ms){
					lowsn_ds6_dad(&lowsn_ds6_if.uaddr_list[i]);
				}
			}
		}
	}
}

/*----------------------------------------------------------------------*/
/**\lookup all the schedule event's schudule time and find the smallest*/
void lowsn_ds6_schedule(systime_s_t internal_clock){
	int i;
	/*Event 1--Send NS Message*/
/*========================
	for(i=0;i<LOWSN_DS6_NBR_NB;i++){
		if(lowsn_ds6_nbr_cache[i].isused && (lowsn_ds6_nbr_cache[i].sendns != 0)){
			if( lowsn_ds6_nbr_cache[i].sendns <= internal_clock){
				//the entry is doing address resolution
				if(lowsn_ds6_nbr_cache[i].state== NBR_INCOMPLETE){
					if(lowsn_ds6_nbr_cache[i].nscount== LOWSN_ND6_MAX_MULTICAST_SOLICIT){
						lowsn_ds6_nbr_rm(&lowsn_ds6_nbr_cache[i]);
						DEBUG_STRING(DBG_INFO, "Addr Resolution Fail\r\n");
					}
					else{
						stimer_set(&lowsn_ds6_nbr_cache[i].sendns, lowsn_ds6_if.retrans_tminterval);
						lowsn_ds6_nbr_cache[i].nscount++;
						lowsn_nd_ns_output(NULL, NULL, &lowsn_ds6_nbr_cache[i].ipaddr);
						DEBUG_STRING(DBG_INFO, "Retransmit a NS for Addr Resolution\r\n");
						return;
					}
				}
				//the entry is doing DAD
				else if(lowsn_ds6_nbr_cache[i].state== NBR_DELAY){
					lowsn_ds6_nbr_cache[i].state = NBR_PROBE;
					stimer_set(&lowsn_ds6_nbr_cache[i].sendns, lowsn_ds6_if.retrans_tminterval);
					lowsn_ds6_nbr_cache[i].nscount++;
					lowsn_nd_ns_output(NULL, &lowsn_ds6_nbr_cache[i].ipaddr, &lowsn_ds6_nbr_cache[i].ipaddr);
					DEBUG_STRING(DBG_INFO, "Transmit a NS for DAD\r\n");
					return;
				}
				else if(lowsn_ds6_nbr_cache[i].state== NBR_PROBE){
					if(lowsn_ds6_nbr_cache[i].nscount== LOWSN_ND6_MAX_UNICAST_SOLICIT){
						lowsn_ds6_nbr_rm(&lowsn_ds6_nbr_cache[i]);
						DEBUG_STRING(DBG_INFO, "DAD Fail\r\n");
					}
					else{
						stimer_set(&lowsn_ds6_nbr_cache[i].sendns, lowsn_ds6_if.retrans_tminterval);
						lowsn_ds6_nbr_cache[i].nscount++;
						lowsn_nd_ns_output(NULL,&lowsn_ds6_nbr_cache[i].ipaddr, &lowsn_ds6_nbr_cache[i].ipaddr);
						DEBUG_STRING(DBG_INFO, "Retransmit a NS for DAD\r\n");
						return;
					}
				}
			}
		}
	}
========================*/
	/*Event 1--Remove an invaild uincast address*/
	for(i=0; i<LOWSN_DS6_UADDR_NB; i++){
		if(lowsn_ds6_if.uaddr_list[i].isused && !lowsn_ds6_if.uaddr_list[i].isinfinite){
			if((lowsn_ds6_if.uaddr_list[i].invalidtime<=internal_clock) &&
						(lowsn_ds6_if.uaddr_list[i].type == ADDR_AUTOCONF)){
				/*remove invalidation address (autoconfigration)*/
				lowsn_ds6_uaddr_rm(&lowsn_ds6_if.uaddr_list[i]);
				DEBUG_STRING(DBG_INFO, "Remove an invalidation uaddr\r\n");
			}
		}
	}
	
	/*Event 2--Remove invalidation Prefix*/
	for(i=0; i<LOWSN_DS6_PREFIX_NB; i++){
		if(lowsn_ds6_prefix_list[i].isused && !lowsn_ds6_prefix_list[i].isinfinite){
			if( lowsn_ds6_prefix_list[i].invalidtime <=internal_clock){
				lowsn_ds6_prefix_rm(&lowsn_ds6_prefix_list[i]);
				DEBUG_STRING(DBG_INFO, "Remove an invalidation Prefix\r\n");
			}
		}
	}

	/*Event 3--Remove invalidation Default Router*/
	for(i=0; i<LOWSN_DS6_DEFRT_NB; i++){
		if(lowsn_ds6_defrt_list[i].isused && !lowsn_ds6_defrt_list[i].isinfinite){
			if(lowsn_ds6_defrt_list[i].invalidtime <=internal_clock){
				/*_ MUST update the Destination Cache_*/
				lowsn_ds6_dest_update(&lowsn_ds6_defrt_list[i].ipaddr);
//				for(j=0; j<LOWSN_DS6_DEST_NB; j++){
//					if(lowsn_ds6_dest_cache[j].isused &&
//								(lowsn_ds6_ipcmp(&lowsn_ds6_dest_cache[j].nexthop, &lowsn_ds6_defrt_list[i].ipaddr))){
//						lowsn_ds6_dest_rm(&lowsn_ds6_dest_cache[j]);
//					}
//				}		
				lowsn_ds6_defrt_rm(&lowsn_ds6_defrt_list[i]);
				DEBUG_STRING(DBG_INFO, "Remove an invalidation Default Router\r\n");
			}
		}
	}

	/*Event 4--Host send RS Message*/
#ifdef LOWSN_COORDINATOR
#else
	if((lowsn_ds6_if.sendrs <= internal_clock)&&(lowsn_ds6_if.sendrs != 0)){
		if(lowsn_ds6_if.rscount == LOWSN_ND6_MAX_RTR_SOLICITATIONS){
			lowsn_ds6_if.sendrs = 0;
		}
		else{
			if(lladdr_dadfail == 1){
				stimer_set(&lowsn_ds6_if.sendrs, 0);
				return;
			}
			stimer_set(&lowsn_ds6_if.sendrs, LOWSN_ND6_RTR_SOLICITATION_INTERVAL);
			lowsn_ds6_if.rscount++;
			lowsn_nd_rs_output();
			DEBUG_STRING(DBG_INFO, "Host send an RS\r\n");
			return;	
		}
	}
#endif

	/*Event 5--IPv6 packet reassembly timeout*/
	if(lowsn_reassbuf.isused){
		if(lowsn_reassbuf.reass_timer <= internal_clock){
			DEBUG_STRING(DBG_INFO, "reassembly timeout\r\n");
			lowsn_reass_timeout();
			 /* copy the header for src and dest address*/
//			memcpy(LOWSN_IP_BUF, FBUF, LOWSN_IPH_LEN);
//			lowsn_icmp6_error_output(ICMP6_TIME_EXCEEDED, ICMP6_TIME_EXCEED_REASSEMBLY, 0);
//			memset(&lowsn_reassbuf, 0, sizeof(lowsn_reassbuf));
//			stimer_set(&lowsn_reassbuf.reass_timer, 0);
			return;
		}
	}

	/*Event 6--update the PMTU*/
	if(pmtu.age<=internal_clock){
		DEBUG_STRING(DBG_INFO, "update the PMTU.\r\n");
		pmtu.size = lowsn_ds6_if.link_mtu;
		stimer_set(&pmtu.age, PMTU_UPDATE_INTERVAL);
	}
}



/*---------------------------------------------------------------------------*/
/** \brief add a new entry to the Neighbor Cache Table
 * arguments:	ipaddr		the on-link ipv6 address of the neighbor's
 *			lladdr		the link layer address of the neighbor's
 *			isrouter		indicate wether the neighbor is an router
 *			state		the state of the new Neighbor Cache Table entry
 *
 */
 lowsn_ds6_nbr_t *
 lowsn_ds6_nbr_add(lowsn_ipaddr_t *ipaddr, lowsn_lladdr_t *lladdr,
				 uint8_t isrouter, uint8_t state)
{
	int r;

	r = lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_nbr_cache, LOWSN_DS6_NBR_NB,
		sizeof(lowsn_ds6_nbr_t), ipaddr, 128, (lowsn_ds6_element_t **)&locnbr);

	//we have found a free space, add the new entry
	if(r == FREESPACE) {
		locnbr->isused = 1;
		lowsn_ipaddr_copy(&locnbr->ipaddr, ipaddr);
		if(lladdr != NULL) {
			memcpy(&locnbr->lladdr, lladdr, LOWSN_LLADDR_LEN);
		} else {
			memset(&locnbr->lladdr, 0, LOWSN_LLADDR_LEN);
		}
		locnbr->isrouter = isrouter;
		locnbr->state = state;

		/* timers are set separately, for now we put them in expired state
        and set the reachable and sendns timer to 0*/
		stimer_set(&(locnbr->reachable), 0);
		stimer_set(&(locnbr->sendns), 0);
		locnbr->nscount = 0;
		locnbr->last_lookup =sysclock_get();
			
		DEBUG_STRING(DBG_INFO, "Adding neighbor with ip addr ");
		PRINT6ADDR(ipaddr);
		DEBUG_STRING(DBG_INFO, "link addr ");
		PRINTLLADDR((&(locnbr->lladdr)));
		DEBUG_STRING(DBG_INFO, "state %u\r\n", state);

		return locnbr;
	}
	else if(r == NOSPACE) {
		/* We did not find any empty slot on the neighbor list, so we need
     	  to remove one old entry to make room. */
		lowsn_ds6_nbr_t *n, *oldest;
		systime_s_t oldest_time;

		oldest = NULL;
		oldest_time = sysclock_get();

		for(n = lowsn_ds6_nbr_cache; n < &lowsn_ds6_nbr_cache[LOWSN_DS6_NBR_NB];n++) {
			if(n->isused) {
				if(n->last_lookup < oldest_time) {
					oldest = n;
					oldest_time = n->last_lookup;
				}
			}
		}
		if(oldest != NULL) {
			lowsn_ds6_nbr_rm(oldest);
			return lowsn_ds6_nbr_add(ipaddr, lladdr, isrouter, state);
		}
	}
	return NULL;
}


/*---------------------------------------------------------------------------*/
/** \brief remove a entry existing in  the Neighbor Cache Table
 * arguments:	nbr		the pointer point to the entry that need to be removed
 *
 * retrv:	none
 */
 void lowsn_ds6_nbr_rm(lowsn_ds6_nbr_t *nbr){
  if(nbr != NULL) {
	nbr->isused = 0;
	nbr->sendns = 0;
	nbr->nscount = 0;
#if LOWSN_CONF_IPV6_QUEUE_PKT
    /** free the packet existing in the queue buffer,
      *  waiting for address resolution complete.
      */
      nbr->queue_buflen = 0;

#endif /* LOWSN_CONF_IPV6_QUEUE_PKT */
  }
  return;
}


/*---------------------------------------------------------------------------*/
/** \brief lookup a sutiable entry in the neighbor cache based on ipv6 address
 * arguments:	ipaddr	the ipv6 address
 *
 * retrv:	the pointer of the found entry
 */
lowsn_ds6_nbr_t *
lowsn_ds6_nbr_lookup(lowsn_ipaddr_t *ipaddr)
{
  if(lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_nbr_cache, LOWSN_DS6_NBR_NB,
      sizeof(lowsn_ds6_nbr_t), ipaddr, 128,
      (lowsn_ds6_element_t **)&locnbr) == FOUND) {
    locnbr->last_lookup = sysclock_get();
    return locnbr;
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
/** \brief lookup a sutiable entry in the neighbor cache based on link layer address
 * arguments:	lladdr	the link layer address
 *
 * retrv:	the pointer of the found entry
 */
lowsn_ds6_nbr_t *
lowsn_ds6_nbr_ll_lookup(lowsn_lladdr_t *lladdr)
{
  lowsn_ds6_nbr_t *fin;

  for(locnbr = lowsn_ds6_nbr_cache, fin = locnbr + LOWSN_DS6_NBR_NB;
       locnbr < fin;
       ++locnbr) {
    if(locnbr->isused) {
      if(!memcmp(lladdr, &locnbr->lladdr, LOWSN_LLADDR_LEN)) {
        return locnbr;
      }
    }
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
/**
  * destination cache table basic routines
**/
lowsn_ds6_dest_t *
lowsn_ds6_dest_add(lowsn_ipaddr_t *ipaddr, lowsn_ipaddr_t *nexthop)
{
	int r;
	r = lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_dest_cache, LOWSN_DS6_DEST_NB,
			sizeof(lowsn_ds6_dest_t), ipaddr, 128, (lowsn_ds6_element_t **)&locdest) ;
	if(r == FREESPACE) {
		lowsn_ipaddr_copy(&locdest->destaddr, ipaddr);
		lowsn_ipaddr_copy(&locdest->nexthop, nexthop);
		locdest->last_lookup = sysclock_get();
		locdest->isused = 1;
			
		DEBUG_STRING(DBG_INFO, "Adding dest with ip addr ");
		PRINT6ADDR(ipaddr);
		DEBUG_STRING(DBG_INFO, "next addr ");
		PRINT6ADDR(nexthop);
		DEBUG_STRING(DBG_INFO, "\r\n");
		return locdest;
	}
	else if(r == NOSPACE){
		/* We did not find any empty slot on the destination cache list, so we need
     	  to remove one old entry to make room. */
		lowsn_ds6_dest_t *n, *oldest;
		systime_s_t oldest_time;

		oldest = NULL;
		oldest_time = sysclock_get();

		for(n = lowsn_ds6_dest_cache; n < &lowsn_ds6_dest_cache[LOWSN_DS6_DEST_NB];n++) {
			if(n->isused) {
				if(n->last_lookup < oldest_time) {
					oldest = n;
					oldest_time = n->last_lookup;
				}
			}
		}
		if(oldest != NULL) {
			lowsn_ds6_dest_rm(oldest);
			return lowsn_ds6_dest_add(ipaddr, nexthop);
		}
	}
	return NULL;
}



void
lowsn_ds6_dest_rm(lowsn_ds6_dest_t *destcach)
{
  if(destcach != NULL) {
    destcach->isused = 0;
  }
  return;
}



lowsn_ds6_dest_t *
lowsn_ds6_dest_lookup(lowsn_ipaddr_t *ipaddress)
{
	if(lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_dest_cache,
		       LOWSN_DS6_DEST_NB, sizeof(lowsn_ds6_dest_t), ipaddress, 128,
		       (lowsn_ds6_element_t **)&locdest) == FOUND) {
		locdest->last_lookup = sysclock_get();
		return locdest;
	}
	return NULL;
}

/**\update the destination cache table*/
void lowsn_ds6_dest_update(lowsn_ipaddr_t* ipaddress){
	uint8_t i = 0;
	for(i=0; i< LOWSN_DS6_DEST_NB; i++){
		if(lowsn_ds6_ipcmp(&lowsn_ds6_dest_cache[i].nexthop, ipaddress)){
   			lowsn_ds6_dest_cache[i].isused = 0;
		}
	}
}





/*---------------------------------------------------------------------------*/
/** \brief add a new entry to the default router table*/
lowsn_ds6_defrt_t *
lowsn_ds6_defrt_add(lowsn_ipaddr_t *ipaddr, unsigned long interval)
{
	if(lowsn_ds6_list_loop
		  ((lowsn_ds6_element_t *)lowsn_ds6_defrt_list, LOWSN_DS6_DEFRT_NB,
		  sizeof(lowsn_ds6_defrt_t), ipaddr, 128,
	  	  (lowsn_ds6_element_t **)&locdefrt) == FREESPACE)
	{
		locdefrt->isused = 1;
		locdefrt->fail = 0;
		lowsn_ipaddr_copy(&locdefrt->ipaddr, ipaddr);
		if(interval != 0) {
			stimer_set(&locdefrt->invalidtime, interval);
			locdefrt->isinfinite = 0;
		}
		else {
			locdefrt->isinfinite = 1;
		}
		
		DEBUG_STRING(DBG_INFO, "Adding defrt with ip addr:");
		PRINT6ADDR(ipaddr);
		DEBUG_STRING(DBG_INFO, "isinfnite==%d",locdefrt->isinfinite);
		DEBUG_STRING(DBG_INFO, "\r\n");
		
		return locdefrt;
	}
	return NULL;
}


/*---------------------------------------------------------------------------*/
/** \brief remove a entry existing in the default router table*/
void lowsn_ds6_defrt_rm(lowsn_ds6_defrt_t *defrt)
{
	if(defrt != NULL) {
		/*update the destination cache table*/
		lowsn_ds6_dest_update(&defrt->ipaddr);
		
		defrt->isused = 0;
		defrt->invalidtime = 0;
		defrt->fail = 0;
	}
	return;
}




/*---------------------------------------------------------------------------*/
/** \brief lookup a sutiable entry in the default router table based on ipv6 address*/
lowsn_ds6_defrt_t *
lowsn_ds6_defrt_lookup(lowsn_ipaddr_t *ipaddr)
{
  if(lowsn_ds6_list_loop((lowsn_ds6_element_t *)lowsn_ds6_defrt_list,
		       LOWSN_DS6_DEFRT_NB, sizeof(lowsn_ds6_defrt_t), ipaddr, 128,
		       (lowsn_ds6_element_t **)&locdefrt) == FOUND) {
    return locdefrt;
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
/** \brief choose a router as the next hop from the default router table*/
/*the best router is that its link layer address exists in the neighbor cache table*/
lowsn_ipaddr_t *
lowsn_ds6_defrt_choose(void)
{
 	lowsn_ds6_nbr_t *bestnbr;
	uint8_t defrtpriority=0, tmppriority=0;
	lowsn_ds6_defrt_t* bestdefrt = NULL;

	locipaddr = NULL;
	for(locdefrt = lowsn_ds6_defrt_list; locdefrt < lowsn_ds6_defrt_list + LOWSN_DS6_DEFRT_NB; locdefrt++) {
		if(locdefrt->isused) {
			bestnbr = lowsn_ds6_nbr_lookup(&locdefrt->ipaddr);
			if(bestnbr != NULL && bestnbr->state != NBR_INCOMPLETE){
				tmppriority = 3;
			}
			else if(locdefrt->fail == 0){
				tmppriority = 2;
			}
			else if(locdefrt->fail == 1){
				tmppriority = 1;
			}
			
			if(tmppriority > defrtpriority){
				bestdefrt = locdefrt;
				locipaddr = &bestdefrt->ipaddr;
				defrtpriority = tmppriority;
			}
		}
	}
	if(locipaddr == NULL){
		printf("there is no default router\r\n");
	}
	return locipaddr;
}




/*----------------------------------------------------------------------*/
/**breif: compute the reachable time based on base reachable time. see RFC 4861*/
uint32_t
lowsn_ds6_compute_reachable_time(void)
{
	return (uint32_t)(LOWSN_ND6_MIN_RANDOM_FACTOR(lowsn_ds6_if.base_reachable_time))+
		((uint16_t)(random_rand()<<8) + (uint16_t)random_rand())%
		(uint32_t) (LOWSN_ND6_MAX_RANDOM_FACTOR(lowsn_ds6_if.base_reachable_time) -
                LOWSN_ND6_MIN_RANDOM_FACTOR(lowsn_ds6_if.base_reachable_time));

}

unsigned short random_rand(void)
{
  return (unsigned short)rand();
}





/*--------------------------------------------------------------------*/
#if LOWSN_ND6_DEF_MAXDADNS > 0
void
lowsn_ds6_dad(lowsn_ds6_uaddr_t *addr)
{
  /* send maxdadns NS for DAD  */
  if(addr->dadnscount < lowsn_ds6_if.maxdadns) {
    lowsn_nd_ns_output(NULL, NULL, &addr->ipaddr);
    addr->dadnscount++;
    mstimer_set(&addr->dadtimer, (lowsn_ds6_if.retrans_tminterval));
    return;
  }
  /*
   * If we arrive here it means DAD succeeded, otherwise the dad process
   * would have been interrupted in ds6_dad_ns/na_input
   */
  DEBUG_STRING(DBG_INFO, "DAD succeeded, ipaddr:");
  PRINT6ADDR(&addr->ipaddr);
  DEBUG_STRING(DBG_INFO, "\r\n");

  addr->state = ADDR_PREFERRED;
  addr->dadtimer = 0;
  if(lowsn_is_addr_link_local(&addr->ipaddr)){
  	stimer_set(&lowsn_ds6_if.sendrs, LOWSN_ND6_MAX_RTR_SOLICITATION_DELAY);
  }
  return;
}

/*---------------------------------------------------------------------------*/
/*
 * Calling code must handle when this returns 0 (e.g. link local
 * address can not be used).
 */
int
lowsn_ds6_dad_failed(lowsn_ds6_uaddr_t *addr)
{
	if(lowsn_is_addr_link_local(&addr->ipaddr)) {
		printf("6LoWSN shutdown, DAD for link local address failed\r\n");
		lladdr_dadfail = 1;
		return 0;
	}
	lowsn_ds6_uaddr_rm(addr);
	return 1;
}
#endif /*LOWSN_ND6_DEF_MAXDADNS > 0 */





























#endif   // #if  0



