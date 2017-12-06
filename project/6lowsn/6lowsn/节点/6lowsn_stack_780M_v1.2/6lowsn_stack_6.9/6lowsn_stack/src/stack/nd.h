/*********************************************************************
    文件名：nd.c
    作  者：ji shanyang
    版  本：v1.0.0
    日  期：2012.12
    描  述：Neighbor Discover Protocol
*********************************************************************/
#ifndef _ND_H
#define _ND_H

/**
 *  \name General
 * @{
 */
/** \brief HOP LIMIT to be used when sending ND messages (255) */
#define LOWSN_ND6_HOP_LIMIT               255
/** \brief INFINITE lifetime */
#define LOWSN_ND6_INFINITE_LIFETIME       0xFFFFFFFF
/** @} */

/** The maximum transmission unit at the IP Layer*/
/**
 * Turn on support for IPv6 Duplicate Address Detection.
 *
 * 6LoWSN supports IPv6 unicast address Duplicate address detection,
 * the MACRO define bellow decide the MAX number of NS messages 
 * shuld be transmitted 
 * 
 * default : disable
 */



/** \name RFC 4861 Host constant */
/** @{ */
#define LOWSN_ND6_MAX_RTR_SOLICITATION_DELAY 1	/*in second*/
#define LOWSN_ND6_RTR_SOLICITATION_INTERVAL  4		/*in second*/
#define LOWSN_ND6_MAX_RTR_SOLICITATIONS	   3		/*transmissions*/
/** @} */

/** \name RFC 4861 Router constants */
/** @{ */
#ifdef LOWSN_COORDINATOR
#ifndef LOWSN_CONF_ND6_SEND_RA
#define LOWSN_ND6_SEND_RA                     1   /* enable/disable RA sending */
#else
#define LOWSN_ND6_SEND_RA LOWSN_CONF_ND6_SEND_RA
#endif


#define LOWSN_ND6_MAX_RA_INTERVAL             600
#define LOWSN_ND6_MIN_RA_INTERVAL             (LOWSN_ND6_MAX_RA_INTERVAL / 3)
#define LOWSN_ND6_M_FLAG                      0
#define LOWSN_ND6_O_FLAG                      0
#define LOWSN_ND6_ROUTER_LIFETIME             3 * LOWSN_ND6_MAX_RA_INTERVAL

#define LOWSN_ND6_MAX_INITIAL_RA_INTERVAL     16  /*seconds*/
#define LOWSN_ND6_MAX_INITIAL_RAS             3   /*transmissions*/
#define LOWSN_ND6_MIN_DELAY_BETWEEN_RAS       3   /*seconds*/
//#define LOWSN_ND6_MAX_RA_DELAY_TIME           0.5 /*seconds*/
#define LOWSN_ND6_MAX_RA_DELAY_TIME_MS        500 /*milli seconds*/
#endif
/** @} */


/** \name RFC 4861 Node constant */
//#define LOWSN_ND6_MAX_MULTICAST_SOLICIT  3
//#define LOWSN_ND6_MAX_UNICAST_SOLICIT    3
#define LOWSN_ND6_MAX_MULTICAST_SOLICIT  3
#define LOWSN_ND6_MAX_UNICAST_SOLICIT    3
#ifdef LOWSN_CONF_ND6_REACHABLE_TIME
#define LOWSN_ND6_REACHABLE_TIME         LOWSN_CONF_ND6_REACHABLE_TIME
#else
#define LOWSN_ND6_REACHABLE_TIME         30000
#endif

#ifdef LOWSN_CONF_ND6_RETRANS_TIMER
#define LOWSN_ND6_RETRANS_TIMER	       LOWSN_CONF_ND6_RETRANS_TIMER
#else
#define LOWSN_ND6_RETRANS_TIMER	       1000
#endif

#define LOWSN_ND6_DELAY_FIRST_PROBE_TIME 4000 /*5s*/
#define LOWSN_ND6_MIN_RANDOM_FACTOR(x)   (x / 2)
#define LOWSN_ND6_MAX_RANDOM_FACTOR(x)   ((x) + (x) / 2)
/** @} */
/*------------------------------------------------------------------------------*/



/** \name ND6 option types */
/** @{ */
#define LOWSN_ND6_OPT_SLLAO               1
#define LOWSN_ND6_OPT_TLLAO               2
#define LOWSN_ND6_OPT_PREFIX_INFO         3
#define LOWSN_ND6_OPT_REDIRECTED_HDR      4
#define LOWSN_ND6_OPT_MTU                 5
/** @} */

/** \name ND6 option fields offset*/
/** @{ */
#define LOWSN_ND6_OPT_TYPE_OFFSET         0
#define LOWSN_ND6_OPT_LEN_OFFSET          1
#define LOWSN_ND6_OPT_DATA_OFFSET         2

/** \name ND6 message length (excluding options and excluding ICMPv6 hesder) */
/** @{ */
#define LOWSN_ND6_NA_LEN                  20
#define LOWSN_ND6_NS_LEN                  20
#define LOWSN_ND6_RA_LEN                  12
#define LOWSN_ND6_RS_LEN                  4
#define LOWSN_ND6_RED_LEN				 36
/** @} */


/**
  * \name ND6 option length in bytes.
  * option shuld be padded when necessary to  ensure
  * that that they end on their natuarl 64-bitsboundaries.
  * (the length of the option in units of 8 octets)
**/

/** @{ */
#define LOWSN_ND6_OPT_HDR_LEN            2
#define LOWSN_ND6_OPT_LLAO_LEN	   8
#define LOWSN_ND6_OPT_PREFIX_INFO_LEN    32
#define LOWSN_ND6_OPT_MTU_LEN            8


/** \name Neighbor Advertisement flags masks */
/** @{ */
#define LOWSN_ND6_NA_FLAG_ROUTER          0x80
#define LOWSN_ND6_NA_FLAG_SOLICITED       0x40
#define LOWSN_ND6_NA_FLAG_OVERRIDE        0x20
#define LOWSN_ND6_RA_FLAG_ONLINK          0x80
#define LOWSN_ND6_RA_FLAG_AUTONOMOUS      0x40
/** @} */



/*-----------------------------------------------------------------*/
/**
 * \name ND message structures(excluding the ICMPv6 header)
 * @{
 */

/**
 * \brief A neighbor solicitation(NS) constant part
 *
 * Possible option is: SLLAO(source link layer address)
 */
typedef struct lowsn_nd6_ns {
  uint32_t reserved;
  lowsn_ipaddr_t tgtipaddr;
}lowsn_nd6_ns_t;

/**
 * \brief A neighbor advertisement(NA) constant part.
 *
 * Possible option is: TLLAO(target link layer address)
 */
typedef struct lowsn_nd6_na {
  uint8_t flagsreserved;
  uint8_t reserved[3];
  lowsn_ipaddr_t tgtipaddr;
} lowsn_nd6_na_t;

/**
 * \brief A router solicitation(RS)  constant part
 *
 * Possible option is: SLLAO(source link layer address)
 */
typedef struct lowsn_nd6_rs {
  uint32_t reserved;
} lowsn_nd6_rs_t;

/**
 * \brief A router advertisement(RA) constant part
 *
 * Possible options are: 
 * SLLAO(source link layer address);
 * MTU; 
 * Prefix Information;
 */
typedef struct lowsn_nd6_ra {
  uint8_t cur_ttl;	//current hop limit
  uint8_t flags_reserved;
  uint16_t router_lifetime;
  uint32_t reachable_time;
  uint32_t retrans_timer;
} lowsn_nd6_ra_t;

/**
 * \brief A redirect message(RDR) constant part
 *
 * Possible options are: TLLAO, redirected header
 */
typedef struct lowsn_nd6_redirect {
  uint32_t reserved;
  lowsn_ipaddr_t tgtipaddr;
  lowsn_ipaddr_t destipaddr;
} lowsn_nd6_red_t;
/** @} */

/*-----------------------------------------------------------------*/
/**
 * \name ND Option structures
 * @{
 */

/** \brief ND option header */
typedef struct lowsn_nd6_opt_hdr {
  uint8_t type;
  uint8_t len;
} lowsn_nd6_opt_hdr_t;

/** \brief ND option prefix information */
typedef struct lowsn_nd6_opt_prefix_info {
  uint8_t type;
  uint8_t len;
  uint8_t preflen;
  uint8_t flagsreserved1;
  uint32_t validlt;
  uint32_t preferredlt;
  uint32_t reserved2;
  lowsn_ipaddr_t prefix;
} lowsn_nd6_opt_prefix_info_t;

/** \brief ND option MTU */
typedef struct lowsn_nd6_opt_mtu {
  uint8_t type;
  uint8_t len;
  uint16_t reserved;
  uint32_t mtu;
} lowsn_nd6_opt_mtu_t;

/** \struct Redirected header option */
typedef struct lowsn_nd6_opt_redirected_hdr {
  uint8_t type;
  uint8_t len;
  uint8_t reserved[6];
} lowsn_nd6_opt_redirected_hdr_t;
/** @} */

/*-----------------------------------------------------------------*/




void ndFmtRS(void);
#ifdef LOWSN_FFD
void ndFmtRA(BYTE active_flag);
#endif
void ndParseRA(void);
BOOL usrGetPrefixCallback(IPADDR *prefix_ptr, UINT8 prefix_len);




#if 0

void
lowsn_nd_ns_input(void);

void
lowsn_nd_ns_output(lowsn_ipaddr_t *src, lowsn_ipaddr_t *dest, lowsn_ipaddr_t *tgt);

void
lowsn_nd_na_input(void);

void 
lowsn_nd_rs_output(void);

void
lowsn_nd_ra_input(void);

void
lowsn_nd_rs_input(void);


void 
lowsn_nd_red_input(void);


#endif









#endif


