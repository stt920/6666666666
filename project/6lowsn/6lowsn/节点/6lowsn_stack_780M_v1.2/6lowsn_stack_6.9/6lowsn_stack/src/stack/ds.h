/*********************************************************************
    文件名：ds.c
    作  者：ji shanyang
    版  本：v1.0.0
    日  期：2012.12
    描  述：define the generic data structure used in 6LoWSN stack
*********************************************************************/

#ifndef _DS_H
#define _DS_H
//#include <stdin.h>
#include "compiler.h"               //compiler specific
#include "6lowsn_common_types.h"
#include "6lowsn_config.h"
#include "ieee_lrwpan_defs.h"
#include "halstack.h"

/*-----------------------------------------------------------------------*/
/**@{*/
/**
 * Configuration. For all tables (Neighbor cache, Prefix List, Routing Table,
 * Default Router List, Unicast address list, multicast address list, anycast address list),
 * we define:
 * - the number of elements requested by the user in 6lowsn config file(name suffixed by _NBU)
 * - the number of elements assigned by the system (name suffixed by _NBS)
 * - the total number of elements is the sum (name suffixed by _NB)
*/
/* Neighbor cache */
#define LOWSN_DS6_NBR_NBS 0
#ifndef LOWSN_CONF_DS6_NBR_NBU
#define LOWSN_DS6_NBR_NBU  4
#else
#define LOWSN_DS6_NBR_NBU LOWSN_CONF_DS6_NBR_NBU
#endif
#define LOWSN_DS6_NBR_NB LOWSN_DS6_NBR_NBS + LOWSN_DS6_NBR_NBU

/*Destination cache*/
#define LOWSN_DS6_DEST_NBS 0
#ifndef LOWSN_CONF_DS6_DEST_NBU
#define LOWSN_DS6_DEST_NBU 4
#else
#define LOWSN_DS6_DEST_NBU LOWSN_CONF_DS6_DEST_NBU
#endif
#define LOWSN_DS6_DEST_NB LOWSN_DS6_DEST_NBS + LOWSN_DS6_DEST_NBU


/* Default router list */
#define LOWSN_DS6_DEFRT_NBS 0
#ifndef LOWSN_CONF_DS6_DEFRT_NBU
#define LOWSN_DS6_DEFRT_NBU 2
#else
#define LOWSN_DS6_DEFRT_NBU LOWSN_CONF_DS6_DEFRT_NBU
#endif
#define LOWSN_DS6_DEFRT_NB LOWSN_DS6_DEFRT_NBS + LOWSN_DS6_DEFRT_NBU

/* Prefix list */
#define LOWSN_DS6_PREFIX_NBS  1
#ifndef LOWSN_CONF_DS6_PREFIX_NBU
#define LOWSN_DS6_PREFIX_NBU  3
#else
#define LOWSN_DS6_PREFIX_NBU LOWSN_CONF_DS6_PREFIX_NBU
#endif
#define LOWSN_DS6_PREFIX_NB LOWSN_DS6_PREFIX_NBS + LOWSN_DS6_PREFIX_NBU

/* Routing table */
/*
#define LOWSN_DS6_ROUTE_NBS 0
#ifndef LOWSN_CONF_DS6_ROUTE_NBU
#define LOWSN_DS6_ROUTE_NBU 4
#else
#define LOWSN_DS6_ROUTE_NBU LOWSN_CONF_DS6_ROUTE_NBU
#endif
#define LOWSN_DS6_ROUTE_NB LOWSN_DS6_ROUTE_NBS + LOWSN_DS6_ROUTE_NBU
*/

/**@}*/

/*----------------------------------------------------------------------------*/
/**@{*/
/**
* config ipv6 address (include unicast address, anycast address ane multicast address)
* we define:
* - the number of elements requested by the user in 6lowsn config file(name suffixed by _NBU)
* - the number of elements assigned by the system (name suffixed by _NBS)
* - the total number of elements is the sum (name suffixed by _NB)
*/
/* Unicast address list*/
#define LOWSN_DS6_UADDR_NBS 1
#ifndef LOWSN_CONF_DS6_UADDR_NBU
#define LOWSN_DS6_UADDR_NBU 3
#else
#define LOWSN_DS6_UADDR_NBU LOWSN_CONF_DS6_UADDR_NBU
#endif
#define LOWSN_DS6_UADDR_NB LOWSN_DS6_UADDR_NBS + LOWSN_DS6_UADDR_NBU

/* Multicast address list */
#ifdef LOWSN_FFD
#define LOWSN_DS6_MADDR_NBS 2 + LOWSN_DS6_UADDR_NB   /* all routers + all nodes + one solicited per unicast */
#else
#define LOWSN_DS6_MADDR_NBS 1 + LOWSN_DS6_UADDR_NB   /* all nodes + one solicited per unicast */
#endif

#ifndef LOWSN_CONF_DS6_MADDR_NBU
#define LOWSN_DS6_MADDR_NBU 0
#else
#define LOWSN_DS6_MADDR_NBU LOWSN_CONF_DS6_MADDR_NBU
#endif
#define LOWSN_DS6_MADDR_NB LOWSN_DS6_MADDR_NBS + LOWSN_DS6_MADDR_NBU

/* Anycast address list */
#ifdef LOWSN_FFD
#define LOWSN_DS6_AADDR_NBS LOWSN_DS6_PREFIX_NB - 1 /* One per non link local prefix (subnet prefix anycast address) */
#else
#define LOWSN_DS6_AADDR_NBS 0
#endif

#ifndef LOWSN_CONF_DS6_AADDR_NBU
#define LOWSN_DS6_AADDR_NBU 1
#else
#define LOWSN_DS6_AADDR_NBU LOWSN_CONF_DS6_AADDR_NBU
#endif
#define LOWSN_DS6_AADDR_NB LOWSN_DS6_AADDR_NBS + LOWSN_DS6_AADDR_NBU
/**@}*/

#ifndef LOWSN_CONF_ND6_DEF_MAXDADNS
#define LOWSN_ND6_DEF_MAXDADNS 1
#else/*LOWSN_CONF_ND6_DEF_MAXDADNS*/
#define LOWSN_ND6_DEF_MAXDADNS LOWSN_CONF_ND6_DEF_MAXDADNS
#endif/*LOWSN_CONF_ND6_DEF_MAXDADNS*/



/*------------------------------------------------------------------------------*/
/**
 * \defgroup lowsnoptip IPv6 configuration options
 * @{
 *
 */
/**
 * The IP TTL (time to live) of IP packets sent by lowsn.
 *
 * This should normally not be changed.
 */
#define LOWSN_TTL         64

/**
  * the default prefix lenght of the ipv6 address
  */
#define LOWSN_DEFAULT_PREFIX_LEN 64

/**
 * The maximum time an IP fragment should wait in the reassembly
 * buffer before it is dropped.
 *
 */
#define LOWSN_REASS_MAXAGE 60       /*60s*/

/**
 * Turn on support for IPv6 packet reassembly.
 *
 * 6LoWSN supports reassembly of fragmented IPv6 packets. This features
 * requires an additional amount of RAM to hold the reassembly buffer
 * and the reassembly code size is approximately 700 bytes.  The
 * reassembly buffer is of the same size as the lowsn_buf buffer
 * (configured by LOWSN_BUFSIZE).
 *
 * \note IP packet reassembly is not heavily tested.
 *
 * \hideinitializer
 */
#ifdef LOWSN_CONF_REASSEMBLY
#define LOWSN_REASSEMBLY (LOWSN_CONF_REASSEMBLY)
#else /* LOWSN_CONF_REASSEMBLY */
#define LOWSN_REASSEMBLY 0
#endif /* LOWSN_CONF_REASSEMBLY */


#if 0
/** The maximum transmission unit at the IP Layer*/
#define LOWSN_LINK_MTU 1500
#define LOWSN_IPV6_MIN_LINK_MTU 1280
#define LOWSN_FRAG_BUF_SIZE 1500
#endif

/** The maximum transmission unit at the IP Layer for IEEE 802.15.4*/
#define LOWSN_LINK_MTU 127
#define LOWSN_IPV6_MIN_LINK_MTU 100
#define LOWSN_FRAG_BUF_SIZE 100

/** @} */


/*----------------------------------------------------------------------------*/
/** \brief Possible states for the nbr cache entries */
#define  NBR_INCOMPLETE 0
#define  NBR_REACHABLE 1
#define  NBR_STALE 2
#define  NBR_DELAY 3
#define  NBR_PROBE 4

/** \brief Possible states for the an address  (RFC 4862) */
#define ADDR_TENTATIVE 0
#define ADDR_PREFERRED 1
#define ADDR_DEPRECATED 2

/** \brief How the address was acquired: Autoconf, DHCP or manually */
#define  ADDR_ANYTYPE 0
#define  ADDR_MANUAL  1
#define  ADDR_AUTOCONF_EUI48   2
#define  ADDR_AUTOCONF_EUI64   3
#define  ADDR_AUTOCONF_SHORTADDR  4
#define  ADDR_AUTOCONF_OTHER  5


/** \brief General DS6 definitions */
#define FOUND 0
#define FREESPACE 1
#define NOSPACE 2


/**\default route priority*/
#define DEFRT_PRIORITY3	3
#define DEFRT_PRIORITY2	2
#define DEFRT_PRIORITY1	1

/*----------------------------------------------------------------------------*/
/**@{*/
/**\brief Possible schedule event flags */
/*----------------------------------------------------------------------------*/

#ifdef LOWSN_FFD
#define SCHEDULE_EVENT_SEND_RS	0x80		//send RS message
#endif/*LOWSN_COORDINATOR == 0*/
#define SCHEDULE_EVENT_SEND_NS	0x40		//send NS message
#define SCHEDULE_EVENT_DEFRT_INVALID 0x20	//a default router become invalid
#define SCHEDULE_EVENT_PREFIX_INVALID 0x10 	//a prefix become invalid
#define SCHEDULE_EVENT_REASSEMB_TIMEOUT	0x04		//ipv6 packet reassemble timeout (60s)

#ifdef LOWSN_FFD
#define SCHEDULE_EVENT_SEND_RA	0x08		//the router send RA message
#endif/*LOWSN_COORDINATOR*/


#define SCHEDULE_INFINITE	0xFFFFFFFF


// 在使用global prefix时，interface ID的产生方式
#define LOWSN_IID_GLOBAL_EUI64  0
#define LOWSN_IID_GLOBAL_SHORTADDR  1


/*----------------------------------------------------------------------------*/
/**@{*/


/**
* define all of the address types. include link layer addreee,
* ipv6 unicast address, ipv6 anycast address, and ipv6 multicast address.
*/
#if LOWSN_CONF_LL_802154
/**\brief define the ieee 802.15.4 link layer address type*/

#else
/** \brief 802.3 address (link layer address)*/
typedef struct lowsn_eth_addr {
  uint8_t addr[8];
} lowsn_lladdr_t;

//#define LOWSN_LLADDR_LEN 6		//the length of link layer address

#define LOWSN_LLADDR_LEN 8		//the length of link layer address

//#define LOWSN_ETHER_LMTU 1600
#define LOWSN_ETHER_LMTU 127

/**
 * \creat IPv6 muliticast link layer address
 * a is a pointer points to the multicast ipv6 addr
 * b is a pointer points to the multicast link-layer address
 * */
#define lowsn_create_mlladdr(a, b)  do {(((b)->addr[0]) = 0x33);\
  (((b)->addr[1]) = 0x33);\
  (((b)->addr[2]) = ((a)->u8[12]));\
  (((b)->addr[3]) = ((a)->u8[13]));\
  (((b)->addr[4]) = ((a)->u8[14]));\
  (((b)->addr[5]) = ((a)->u8[15])); \
} while(0)


/*---------------------------------------------*

 寻找组播IPv6地址对应的组播短地址

An IPv6 packet with a multicast destination address (DST),
consisting of the sixteen octets DST[1] through DST[16], is
transmitted to the following 802.15.4 16-bit multicast address:
0 1
0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|1 0 0|DST[15]* | DST[16] |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
Figure 8
Here, DST[15]* refers to the last 5 bits in octet DST[15], that is,
bits 3-7 within DST[15]. The initial 3-bit pattern of "100" follows
the 16-bit address format for multicast addresses (Section 12).

 *---------------------------------------------*/
#define ds6IPtoMAddr(ip, maddr)  do{maddr = ((ip)->u8[15]) + ((ip)->u8[14]) & 0x1F + 0x4) << 8;}while(0)


/*---------------------------------------------*

寻找IPv6地址对应的EUI 64地址

 
 *---------------------------------------------*/
#define ds6IPtoLAddr(ip, laddr)  do{ \
  (((laddr)->bytes[0]) = ((ip)->u8[8])^0x02);\
  (((laddr)->bytes[1]) = ((ip)->u8[9]));\
  (((laddr)->bytes[2]) = ((ip)->u8[10]));\
  (((laddr)->bytes[3]) = ((ip)->u8[11]));\
  (((laddr)->bytes[4]) = ((ip)->u8[12]));\
  (((laddr)->bytes[5]) = ((ip)->u8[13]));\
  (((laddr)->bytes[6]) = ((ip)->u8[14]));\
  (((laddr)->bytes[7]) = ((ip)->u8[15])); \
  }while (0)


/*---------------------------------------------*

 寻找IPv6地址对应的16位短地址


 *---------------------------------------------*/
#define ds6IPtoSAddr(ip, saddr)  do{ ( saddr = (((uint16_t)((ip)->u8[14]))<<8) | (uint16_t)((ip)->u8[15]) );}while (0)


typedef struct lowsn_eth_hdr{
	lowsn_lladdr_t destlladdr;
	lowsn_lladdr_t srclladdr;
	uint16_t type;
}lowsn_eth_hdr_t;
#define LOWSN_LLH_LEN 14		//the lenght of link layer header
#endif

/** *\brief ipv6 address structure*/
typedef union lowsn_ip6addr {
  uint8_t u8[16];			/* Initializer, must come first!!! */
  uint16_t u16[8];
} lowsn_ipaddr_t;

typedef union lowsn_ip6addr  IPADDR;

/** * \brief Unicast address structure */
typedef struct lowsn_ds6_uaddr {
  uint8_t isused;
  lowsn_ipaddr_t ipaddr;
  uint8_t state;
  uint8_t type;
  uint8_t isinfinite;
  systime_s_t invalidtime;	 /* in second : */
} lowsn_ds6_uaddr_t;

/** \brief Anycast address  */
typedef struct lowsn_ds6_aaddr {
  uint8_t isused;
  lowsn_ipaddr_t ipaddr;
} lowsn_ds6_aaddr_t;

/** \brief A multicast address */
typedef struct lowsn_ds6_maddr {
  uint8_t isused;
  lowsn_ipaddr_t ipaddr;
  uint8_t assoc_count;		//record how many uincast address associate with the muliticast address
} lowsn_ds6_maddr_t;

/**@}*/


/*define IPv6 reassemble buffer*/
typedef struct lowsn_ds6_reassembuf{
	uint8_t isused;
	uint8_t buf[LOWSN_FRAG_BUF_SIZE];
	uint16_t orgpktlen;
	uint16_t unfraglen;
	uint16_t fraglen;
	uint16_t reassem_len;	
	uint8_t firstfrag;
	uint8_t lastfrag;
	uint32_t identification;
	lowsn_ipaddr_t srcipaddr;
	lowsn_ipaddr_t destipaddr;
	systime_s_t reass_timer;
}lowsn_ds6_reassbuf_t;

typedef struct lowsn_ds6_fragbuf{
	uint8_t isused;
	uint8_t buf[LOWSN_FRAG_BUF_SIZE];
	uint16_t origpktlen;
	uint8_t next;
	uint16_t unfraglen;
	uint16_t fraglen;
	uint32_t identification;
	uint16_t maxpayloadlen;
}lowsn_ds6_fragbuf_t;

	/*the PMTU*/
typedef struct lowsn_ds6_pmtu{
	uint16_t size;		//the size of the path min mtu
	systime_s_t  age;	//the age of the pmtu
}lowsn_ds6_pmtu_t;

#define PMTU_UPDATE_INTERVAL 600		//the interval of update the pmtu : 10 mins

/*----------------------------------------------------------------------------*/
/**@{*/
/**
* define the net interface structure and the structure of neighbor cache entry, destination cache entry,
* prefix list entry and default router list entry.
*/
/** \brief  Interface structure (contains all the interface variables) */
typedef struct lowsn_ds6_netif {
  uint32_t link_mtu;
  uint8_t cur_hop_limit;
  uint8_t rscount;
  systime_s_t sendrs;		/* 0: indicate that there is no need to seed RS message	\in second*/
  stimer_t base_reachable_time;  /* in msec */
  stimer_t reachable_tminterval;      /* in msec : the time interval consider neighbor to be connective */
  stimer_t retrans_tminterval;       /* in msec : The time interval retransmit Neighbor Solicitation messages*/
  uint8_t maxdadns;			/*max duplicate address detectation neighbor solictation*/
  uint8_t iid_gen_method;   // Interface gengeate method under global prefixs: 0: from EUI 64; 1: from short address and panid
  lowsn_ds6_uaddr_t uaddr_list[LOWSN_DS6_UADDR_NB];		/*unicast address list*/
  lowsn_ds6_aaddr_t aaddr_list[LOWSN_DS6_AADDR_NB];		/*anycast address list*/
  lowsn_ds6_maddr_t maddr_list[LOWSN_DS6_MADDR_NB];	/*mulitcast address list*/
} lowsn_ds6_netif_t;


/** \brief An entry in the nbr cache */
typedef struct lowsn_ds6_nbr {
  uint8_t isused;
  lowsn_ipaddr_t ipaddr;		//ip address
  lowsn_lladdr_t lladdr;		//link layer address
  systime_s_t reachable;		//in msec	: the neighbor consider to be connective before the time arrived
  systime_s_t sendns;			//in msec : the time to retransmit the NS message
  systime_s_t last_lookup;		//in msec : the time lookup the entry last time
  uint8_t nscount;			//the count of NS has been transmited
  uint8_t isrouter;
  uint8_t state;
#if LOWSN_CONF_IPV6_QUEUE_PKT
  uint8_t queue_buf[LOWSN_LINK_MTU];
  uint16_t queue_buflen;	//the packet lenght in the queuing buffer
//  uint8_t qbuffull;	//\ 1--the queuing buffer is used; 0--the queuing buffer is empty
#endif /*LOWSN_CONF_QUEUE_PKT */
} lowsn_ds6_nbr_t;


/** \brief A prefix list entry */
#ifdef LOWSN_FFD
typedef struct lowsn_ds6_prefix {
  uint8_t isused;
  lowsn_ipaddr_t ipaddr;
  uint8_t length;
  uint8_t advertise;
  systime_s_t invalidtime;
  uint32_t plifetime;
  uint32_t vlifetime;
  uint8_t l_a_reserved; /**< on-link and autonomous flags + 6 reserved bits */
} lowsn_ds6_prefix_t;
#else /* LOWSN_FFD */
typedef struct lowsn_ds6_prefix {
  uint8_t isused;
  lowsn_ipaddr_t ipaddr;
  uint8_t length;
  systime_s_t invalidtime;		//in sencond : the time when the entry become invalid
  uint8_t isinfinite;
} lowsn_ds6_prefix_t;
#endif /*LOWSN_FFD */


/** \brief An entry in the destination cache */
typedef struct lowsn_ds6_dest{
	uint8_t isused;
	lowsn_ipaddr_t destaddr;
	lowsn_ipaddr_t nexthop;
	systime_s_t last_lookup;
}lowsn_ds6_dest_t;


/** \brief An entry in the default router list */
typedef struct lowsn_ds6_defrt {
  uint8_t isused;
  lowsn_ipaddr_t ipaddr;
  systime_s_t invalidtime;		//in second : the time when the entry become invalid
  uint8_t isinfinite;
  uint8_t fail;			//indicate weather the default router failed earlier or not
} lowsn_ds6_defrt_t;


/**\breif : Generic type for a DS6, to use a common loop though all DS*/
typedef struct lowsn_ds6_element{
    uint8_t isused;
    lowsn_ipaddr_t ipaddr;
}lowsn_ds6_element_t;

/**@}*/

/*----------------------------------------------------------------------------*/
/**@{*/
/**\brief Possible schedule event flags and data structure*/
/*----------------------------------------------------------------------------*/
typedef struct lowsn_ds6_ev_schedule{
	systime_s_t schedule_mstimer;
	systime_s_t schedule_stimer;
	uint8_t schedule_flag;
	lowsn_ds6_element_t* schedule_entry;	//useb by default router table, neighbor cache table, and prefix table.
}lowsn_ds6_schedule_t;


/*----------------------------------------------------------------------------------*/

/**\brief compare to ipv6 address with prefix length*/
#define lowsn_ipaddr_prefixcmp(addr1, addr2, length) (memcmp(addr1, addr2, length>>3) == 0)

/**\brief copy an ipv6 address from one place to another*/
#define lowsn_ipaddr_copy(dest, src) (*(dest) = *(src))
/*
#define lowsn_ipaddr_copy(dest, src)\
	(((dest)->u16[0]) = ((src)->u16[0]));\
	(((dest)->u16[1]) = ((src)->u16[1]));\
	(((dest)->u16[2]) = ((src)->u16[2]));\
	(((dest)->u16[3]) = ((src)->u16[3]));\
	(((dest)->u16[4]) = ((src)->u16[4]));\
	(((dest)->u16[5]) = ((src)->u16[5]));\
	(((dest)->u16[6]) = ((src)->u16[6]));\
	(((dest)->u16[7]) = ((src)->u16[7]))
*/


/**
 * \briefput in b the solicited node address corresponding to address a
 * both a and b are of type lowsn_ipaddr_t*
 * */
#define lowsn_create_solicited_node(a, b) \
  (((b)->u8[0]) = 0xFF);\
  (((b)->u8[1]) = 0x02);\
  (((b)->u16[1]) = 0);\
  (((b)->u16[2]) = 0);\
  (((b)->u16[3]) = 0);\
  (((b)->u16[4]) = 0);\
  (((b)->u8[10]) = 0);\
  (((b)->u8[11]) = 0x01);\
  (((b)->u8[12]) = 0xFF);\
  (((b)->u8[13]) = ((a)->u8[13]));\
  (((b)->u16[7]) = ((a)->u16[7]))


/**
 * \briefput in b the solicited node address corresponding to address a
 * both a and b are of type uip_ipaddr_t*
 * */
 /*
#define lowsn_create_solicited_node(a, b)    \
  (((b)->u8[0]) = 0xFF);                        \
  (((b)->u8[1]) = 0x02);                        \
  (((b)->u16[1]) = 0);                          \
  (((b)->u16[2]) = 0);                          \
  (((b)->u16[3]) = 0);                          \
  (((b)->u16[4]) = 0);                          \
  (((b)->u8[10]) = 0);                          \
  (((b)->u8[11]) = 0x01);                       \
  (((b)->u8[12]) = 0xFF);                       \
  (((b)->u8[13]) = ((a)->u8[13]));              \
  (((b)->u16[7]) = ((a)->u16[7]))
*/

/** \brief set IP address a to the link local all-nodes multicast address */
#define lowsn_create_linklocal_allnodes_mcast(a) lowsn_ip6addr(a, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001)
/** \brief set IP address a to the link local all-routers multicast address */
#define lowsn_create_linklocal_allrouters_mcast(a) lowsn_ip6addr(a, 0xff02, 0, 0, 0, 0, 0, 0, 0x0002)
#define lowsn_create_linklocal_prefix(addr) do { \
    (addr)->u16[0] = LOWSN_HTONS(0xfe80);            \
    (addr)->u16[1] = 0;                        \
    (addr)->u16[2] = 0;                        \
    (addr)->u16[3] = 0;                        \
  } while(0)
/**
 * \brief Is IPv6 address a the unspecified address
 * a is of type uip_ipaddr_t
 */
#define lowsn_is_addr_loopback(a)                  \
  ((((a)->u16[0]) == 0) &&                       \
   (((a)->u16[1]) == 0) &&                       \
   (((a)->u16[2]) == 0) &&                       \
   (((a)->u16[3]) == 0) &&                       \
   (((a)->u16[4]) == 0) &&                       \
   (((a)->u16[5]) == 0) &&                       \
   (((a)->u16[6]) == 0) &&                       \
   (((a)->u8[14]) == 0) &&                       \
   (((a)->u8[15]) == 0x01))
/**
 * \brief Is IPv6 address a the unspecified address
 * a is of type uip_ipaddr_t
 */
#define lowsn_is_addr_unspecified(a)               \
  ((((a)->u16[0]) == 0) &&                       \
   (((a)->u16[1]) == 0) &&                       \
   (((a)->u16[2]) == 0) &&                       \
   (((a)->u16[3]) == 0) &&                       \
   (((a)->u16[4]) == 0) &&                       \
   (((a)->u16[5]) == 0) &&                       \
   (((a)->u16[6]) == 0) &&                       \
   (((a)->u16[7]) == 0))



/** \brief Is IPv6 address a the link local all-nodes multicast address */
#define lowsn_is_addr_linklocal_allnodes_mcast(a)     \
  ((((a)->u8[0]) == 0xff) &&                        \
   (((a)->u8[1]) == 0x02) &&                        \
   (((a)->u16[1]) == 0) &&                          \
   (((a)->u16[2]) == 0) &&                          \
   (((a)->u16[3]) == 0) &&                          \
   (((a)->u16[4]) == 0) &&                          \
   (((a)->u16[5]) == 0) &&                          \
   (((a)->u16[6]) == 0) &&                          \
   (((a)->u8[14]) == 0) &&                          \
   (((a)->u8[15]) == 0x01))

/** \brief Is IPv6 address a the link local all-routers multicast address */
#define lowsn_is_addr_linklocal_allrouters_mcast(a)     \
  ((((a)->u8[0]) == 0xff) &&                        \
   (((a)->u8[1]) == 0x02) &&                        \
   (((a)->u16[1]) == 0) &&                          \
   (((a)->u16[2]) == 0) &&                          \
   (((a)->u16[3]) == 0) &&                          \
   (((a)->u16[4]) == 0) &&                          \
   (((a)->u16[5]) == 0) &&                          \
   (((a)->u16[6]) == 0) &&                          \
   (((a)->u8[14]) == 0) &&                          \
   (((a)->u8[15]) == 0x02))

/**
 * \brief  is addr (a) a solicited node multicast address, see RFC3513
 *  a is of type uip_ipaddr_t*
 */
#define lowsn_is_addr_solicited_node(a)          \
  ((((a)->u8[0])  == 0xFF) &&                  \
   (((a)->u8[1])  == 0x02) &&                  \
   (((a)->u16[1]) == 0x00) &&                  \
   (((a)->u16[2]) == 0x00) &&                  \
   (((a)->u16[3]) == 0x00) &&                  \
   (((a)->u16[4]) == 0x00) &&                  \
   (((a)->u8[10]) == 0x00) &&                  \
   (((a)->u8[11]) == 0x01) &&                  \
   (((a)->u8[12]) == 0xFF))


/**
 * \brief is addr (a) a link local unicast address, see RFC3513
 *  i.e. is (a) on prefix FE80::/10
 *  a is of type lowsn_ipaddr_t*
 */
#define lowsn_is_addr_link_local(a) \
  ((((a)->u8[0]) == 0xFE) && \
  (((a)->u8[1]) == 0x80))


/**
判断是否是由16位短地址生成的伪接口指示符
该函数的使用必须在判定完on-link之后，即位于
同一前缀网内或是link-local地址
 */
#define lowsn_is_addr_genfrom_shortaddr(a)   \
 ((((a)->u8[11]) == 0xFF) && \
  (((a)->u8[12]) == 0xFE) &&  \
 ((((a)->u8[8]) & 0x02) >> 1 == 0)) 


/**
判断是否是由64位长地址生成的
简单判断U/L位
为1则认为是长地址，否则是短地址
注意: 若是手动分配的全球唯一接口描述符，则这样的
判断是不完备的。将来可进一步增加EUI-64的特征来
增强判断能力

 */
#define lowsn_is_addr_genfrom_EUI64(a)   (((((a)->u8[8]) & 0x02) >> 1) == 1)


/**
 * \brief is address a multicast address, see RFC 3513
 * a is of type lowsn_ipaddr_t*
 * */
#define lowsn_is_addr_mcast(a)                    \
  (((a)->u8[0]) == 0xFF)

#define lowsn_is_addr_gmcast(a)	\
	(((a)->u8[0]) == 0xFF && (((a)->u8[1])&0x0e) == 0x0e)
/**
 * Construct an IPv6 address from sixteen 8-bit words.
 *
 * This function constructs an IPv6 address.
 *
 * \hideinitializer
 */
#define lowsn_ip6addr_u8(addr, addr0,addr1,addr2,addr3,addr4,addr5,addr6,addr7,addr8,addr9,addr10,addr11,addr12,addr13,addr14,addr15) do { \
    (addr)->u8[0] = addr0;                                       \
    (addr)->u8[1] = addr1;                                       \
    (addr)->u8[2] = addr2;                                       \
    (addr)->u8[3] = addr3;                                       \
    (addr)->u8[4] = addr4;                                       \
    (addr)->u8[5] = addr5;                                       \
    (addr)->u8[6] = addr6;                                       \
    (addr)->u8[7] = addr7;                                       \
    (addr)->u8[8] = addr8;                                       \
    (addr)->u8[9] = addr9;                                       \
    (addr)->u8[10] = addr10;                                     \
    (addr)->u8[11] = addr11;                                     \
    (addr)->u8[12] = addr12;                                     \
    (addr)->u8[13] = addr13;                                     \
    (addr)->u8[14] = addr14;                                     \
    (addr)->u8[15] = addr15;                                     \
  } while(0)


/**
  * Convert 16-bit quantity from host byte order to network byte order.
**/

#if LOWSN_COMPILER_BIG_ENDIAN
#define LOWSN_HTONS(n) (n)
#define LOWSN_HTONL(n) (n)
#else /* LOWSN_COMPILER_BIG_ENDIAN */
#define LOWSN_HTONS(n) (uint16_t)((((uint16_t) (n)) << 8) | (((uint16_t) (n)) >> 8))
#define LOWSN_HTONL(n) (((uint32_t)LOWSN_HTONS(n) << 16) | LOWSN_HTONS((uint32_t)(n) >> 16))
#endif /* LOWSN_COMPILER_BIG_ENDIAN */

#define lowsn_ntohs lowsn_htons
#define lowsn_ntohl lowsn_htonl

/**
 * Construct an IPv6 address from eight 16-bit words.
 *
 * This function constructs an IPv6 address.
 *
 * \hideinitializer
 */
#define lowsn_ip6addr(addr, addr0,addr1,addr2,addr3,addr4,addr5,addr6,addr7) do { \
    (addr)->u16[0] = LOWSN_HTONS(addr0);                                      \
    (addr)->u16[1] = LOWSN_HTONS(addr1);                                      \
    (addr)->u16[2] = LOWSN_HTONS(addr2);                                      \
    (addr)->u16[3] = LOWSN_HTONS(addr3);                                      \
    (addr)->u16[4] = LOWSN_HTONS(addr4);                                      \
    (addr)->u16[5] = LOWSN_HTONS(addr5);                                      \
    (addr)->u16[6] = LOWSN_HTONS(addr6);                                      \
    (addr)->u16[7] = LOWSN_HTONS(addr7);                                      \
  } while(0)

/** \brief set IP address a to unspecified */
#define lowsn_create_unspecified(a) lowsn_ip6addr(a, 0, 0, 0, 0, 0, 0, 0, 0)

/** \name Macros to check if an IP address (unicast, multicast or anycast) is mine */
/**argument addr is a pointer point to the address*/
/** @{ */
#define lowsn_ds6_is_my_uaddr(addr)  (lowsn_ds6_uaddr_lookup(addr) != NULL)
#define lowsn_ds6_is_my_maddr(addr) (lowsn_ds6_maddr_lookup(addr) != NULL)
#define lowsn_ds6_is_my_aaddr(addr) (lowsn_ds6_aaddr_lookup(addr) != NULL)
/** @} */
/*---------------------------------------------------------------------------*/

void ds6Init(void);

void ds6LocalIPFrom16(void);



uint8_t
lowsn_ds6_list_loop(lowsn_ds6_element_t *list, uint8_t size,
                  uint16_t elementsize, lowsn_ipaddr_t *ipaddr,
                  uint8_t ipaddrlen, lowsn_ds6_element_t **out_element);





//----------------------------------------------------------------


#ifdef LOWSN_FFD
lowsn_ds6_prefix_t *
lowsn_ds6_prefix_add(lowsn_ipaddr_t *ipaddr, uint8_t ipaddrlen,
                   uint8_t advertise, uint8_t flags, unsigned long vtime,
                   unsigned long ptime);

#else /* LOWSN_COORDINATOR */
lowsn_ds6_prefix_t *
lowsn_ds6_prefix_add(lowsn_ipaddr_t *ipaddr, uint8_t ipaddrlen,
                   unsigned long interval);
#endif


void
lowsn_ds6_prefix_rm(lowsn_ds6_prefix_t *prefix);


lowsn_ds6_prefix_t *
lowsn_ds6_prefix_lookup(lowsn_ipaddr_t *ipaddr, uint8_t ipaddrlen);

uint8_t ds6TestOnLink(lowsn_ipaddr_t *ipaddr);


//----------------------------------------------------------------



lowsn_ds6_uaddr_t *
lowsn_ds6_uaddr_add(lowsn_ipaddr_t *ipaddr, unsigned long vlifetime, uint8_t type);

void
lowsn_ds6_uaddr_rm(lowsn_ds6_uaddr_t *addr);

lowsn_ds6_uaddr_t *
lowsn_ds6_uaddr_lookup(lowsn_ipaddr_t *ipaddr);

lowsn_ds6_uaddr_t * ds6FindLinkLocalIP(int8_t state, int8_t type);

lowsn_ds6_uaddr_t * ds6FindGlobalIP(int8_t state, int8_t type);

//----------------------------------------------------------------
lowsn_ds6_maddr_t *
lowsn_ds6_maddr_add(lowsn_ipaddr_t *ipaddr);

void
lowsn_ds6_maddr_rm(lowsn_ds6_maddr_t *maddr);

lowsn_ds6_maddr_t *
lowsn_ds6_maddr_lookup(lowsn_ipaddr_t *ipaddr);

//----------------------------------------------------------------

lowsn_ds6_aaddr_t *
lowsn_ds6_aaddr_add(lowsn_ipaddr_t *ipaddr);

void
lowsn_ds6_aaddr_rm(lowsn_ds6_aaddr_t *aaddr);

lowsn_ds6_aaddr_t *
lowsn_ds6_aaddr_lookup(lowsn_ipaddr_t *ipaddr);

//----------------------------------------------------------------

void ds6FindSrcIP(lowsn_ipaddr_t *src, lowsn_ipaddr_t *dst);

void ds6GenInterfaceID_EUI(lowsn_ipaddr_t *ipaddr, BYTE *lladdr, uint8_t genmethod);

void ds6GenInterfaceID64(lowsn_ipaddr_t *ipaddr);

void ds6GenInterfaceID16(lowsn_ipaddr_t *ipaddr, UINT16 panid, UINT16 short_addr);

uint8_t ds6MatchNum(lowsn_ipaddr_t *src, lowsn_ipaddr_t *dst);



uint16_t
lowsn_htons(uint16_t val);

uint32_t
lowsn_htonl(uint32_t val);


void
lowsn_ds6_dad(lowsn_ds6_uaddr_t *addr);


int
lowsn_ds6_dad_failed(lowsn_ds6_uaddr_t *addr);


uint8_t lowsn_ds6_ipcmp(lowsn_ipaddr_t* srcip, lowsn_ipaddr_t* destip);

uint8_t lowsn_ds6_lladdr_cmp(lowsn_lladdr_t* srcll, lowsn_lladdr_t* destll);

extern lowsn_ds6_netif_t lowsn_ds6_if;




#if 0


void lowsn_ds6_schedule_ms(systime_ms_t internal_clock_ms);

void lowsn_ds6_schedule(systime_s_t internal_clock);


 //----------------------------------------------------------------

 lowsn_ds6_nbr_t *
 lowsn_ds6_nbr_add(lowsn_ipaddr_t *ipaddr, lowsn_lladdr_t *lladdr,
				 uint8_t isrouter, uint8_t state);

 void
 lowsn_ds6_nbr_rm(lowsn_ds6_nbr_t *nbr);

lowsn_ds6_nbr_t *
lowsn_ds6_nbr_lookup(lowsn_ipaddr_t *ipaddr);

lowsn_ds6_nbr_t *
lowsn_ds6_nbr_ll_lookup(lowsn_lladdr_t *lladdr);


 //----------------------------------------------------------------


lowsn_ds6_dest_t *
lowsn_ds6_dest_add(lowsn_ipaddr_t *ipaddr, lowsn_ipaddr_t *nexthop);


void
lowsn_ds6_dest_rm(lowsn_ds6_dest_t *destcach);


lowsn_ds6_dest_t *
lowsn_ds6_dest_lookup(lowsn_ipaddr_t *ipaddress);

void lowsn_ds6_dest_update(lowsn_ipaddr_t* ipaddress);


 //----------------------------------------------------------------


lowsn_ds6_defrt_t *
lowsn_ds6_defrt_add(lowsn_ipaddr_t *ipaddr, unsigned long interval);

void
lowsn_ds6_defrt_rm(lowsn_ds6_defrt_t *defrt);

lowsn_ds6_defrt_t *
lowsn_ds6_defrt_lookup(lowsn_ipaddr_t *ipaddr);

lowsn_ipaddr_t *
lowsn_ds6_defrt_choose(void);


//----------------------------------------------------------------


 uint32_t
 lowsn_ds6_compute_reachable_time(void);
 unsigned short random_rand(void);

//----------------------------------------------------------------

#endif


#endif


