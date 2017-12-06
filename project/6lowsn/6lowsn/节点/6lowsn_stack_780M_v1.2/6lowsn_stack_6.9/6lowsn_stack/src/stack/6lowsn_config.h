


#ifndef LOWSN_CONFIG_H
#define LOWSN_CONFIG_H

#define LOWSN_VERSION  "s0.13"


// size for dynamically allocation functions in memalloc
#ifndef LOWSN_HEAPSIZE
#define LOWSN_HEAPSIZE  8192
//#define LOWSN_HEAPSIZE  8192
#endif

//only support 2.4GHz right now
#define LOWSN_DEFAULT_FREQUENCY PHY_FREQ_780M //5.4changed  780now
#define LOWSN_SYMBOLS_PER_SECOND   62500

// 4 ( 0 |1|2|3 )个信道为779 至787MHz，信道间隔为2MHz。中心频率为780，782，784 及786MHz。
#define LOWSN_DEFAULT_START_CHANNEL                           1

//a zero indicates the channel should not be used, so 0xFFFFFFFF allows all 16 channels
// 2.4GHz channels are in bits 11-26
#define LOWSN_DEFAULT_CHANNEL_MASK 0xFFFFFFFF
//PANID to use for this network
#define LOWSN_USE_STATIC_PANID      //if this is defined, then DEFAULT PANID always used
#define LOWSN_DEFAULT_PANID 0x1347
#define LOWSN_DEFAULT_CHANNEL_SCAN_DURATION 4

//maximum number of buffered RX packets in MAC layer
#define LOWSN_MAX_MAC_RX_PKTS   10//4

//maximum number of packets waiting to be forwarded to other nodes in ADP layer
//only has to be defined for FFDs
#define LOWSN_MAX_ADP_RX_PKTS   4


//maximum number of indirect packets waiting to be resolved
//only has to be defined by the coordinator.
#define LOWSN_MAX_INDIRECT_RX_PKTS 2



/*
If LOWSN_ENABLE_SLOW_TIMER is defined, then the HAL layer
will configure a timer for periodic interrupt using the SLOWTICKS_PER_SECOND value
Also, the hal layer will call the usr function usrSlowTimerInt() each time
the interrupt occurs.

If the slow timer is enabled, then the EVB switches will be sampled as this rate.

Look in the halStack.c file to see what timer resource is used for this.
It may be different from the timer resource used for the macTimer.

Disable this if you do not want to use this timer resource.

*/

// 基本的端口号，来源于RFC4944  ，实际的端口号=  LOWSN_BASE_PORT+ short_port
// short_port: 0始终用于管理进程; 用户使用的从1开始.

#define LOWSN_BASE_PORT   0xF0B0

//uncomment this if you want the ASSOC_RESPONSE, ASSOC_REQUEST to be 802.15.4 compatible
#define IEEE_802_COMPLY


// 规定B0-B7为阅读顺序，即B0为含有global的字节

//this is only used if device does not have
//some other way to set the address
//if you redefine one byte, must redefine all bytes
#ifndef aExtendedAddress_B7
#define aExtendedAddress_B7 0x00
#define aExtendedAddress_B6 0x00
#define aExtendedAddress_B5 0x00
#define aExtendedAddress_B4 0x00
#define aExtendedAddress_B3 0x00
#define aExtendedAddress_B2 0x00
#define aExtendedAddress_B1 0x00
#define aExtendedAddress_B0 0x00
#endif


//uncomment this if you want to force association to a particular target
//#ifdef LOWSN_FORCE_ASSOCIATION_TARGET
//set the following to the long address of the parent to associate with
//if using forced association.
//if you use forced association, then you must NOT define IEEE_802_COMPLY
//as forced association depends upon our special associate request/response
#define parent_addr_B0 0x00
#define parent_addr_B1 0x00
#define parent_addr_B2 0x00
#define parent_addr_B3 0x00
#define parent_addr_B4 0x00
#define parent_addr_B5 0x00
#define parent_addr_B6 0x00
#define parent_addr_B7 0x00



//MAC Capability Info

//if either router or coordinator, then one of these must be defined
//#define LOWSN_COORDINATOR
//#define LOWSN_ROUTER




#if (defined (LOWSN_COORDINATOR) || defined (LOWSN_ROUTER) )
#define LOWSN_FFD
#define LOWSN_ROUTING_CAPABLE
#endif
#if !defined (LOWSN_FFD)
#define LOWSN_RFD
#endif

//define this if ACMAIN POWERED
#define LOWSN_ACMAIN_POWERED
//define this if Receiver on when idle
#define LOWSN_RCVR_ON_WHEN_IDLE
//define this if capable of RX/TX secure frames
//#define LOWSN_SECURITY_CAPABLE



//comment this if you want the phy to call the EVBPOLL function
//do this if you want to poll EVB inputs during the stack idle
//time
//#define LOWSN_PHY_CALL_EVBPOLL

#define LOWSN_ZIGBEE_PROTOCOL_ID   0
#define LOWSN_ZIGBEE_PROTOCOL_VER  0
#define LOWSN_STACK_PROFILE  0         //indicates this is a closed network.
#define LOWSN_APP_PROFILE    0xFFFF    //filter data packets by this profile number
#define LOWSN_APP_CLUSTER    0x2A    //default cluster, random value for debugging

//define this if you want the beacon payload to comply with the Zigbee standard
#define LOWSN_ZIGBEE_BEACON_COMPLY

//Network parameters



//this is a magic number exchanged with nodes wishing to join our
//network. If they do not match this number, then they are rejected.
//Sent in beacon payload
#define LOWSN_ADP_MAGICNUM_B0 0x0AA
#define LOWSN_ADP_MAGICNUM_B1 0x055
#define LOWSN_ADP_MAGICNUM_B2 0x0C3
#define LOWSN_ADP_MAGICNUM_B3 0x03C



/*
These numbers determine affect the size of the neighbor
table, and the maximum number of nodes in the network,
and how short addresses are assigned to nodes.

*/
#define LOWSN_MAX_DEPTH                   5
#define LOWSN_MAX_ROUTERS_PER_PARENT      4
//these are total children, includes routers!
//#define LOWSN_MAX_CHILDREN_PER_PARENT    17
#define LOWSN_MAX_CHILDREN_PER_PARENT    40
#define LOWSN_MAX_NON_ROUTER_CHILDREN    (LOWSN_MAX_CHILDREN_PER_PARENT-LOWSN_MAX_ROUTERS_PER_PARENT)



//if using Indirect addressing, then this number determines the
//maximum size of the address table map used by the coordinator
//that matches long addresses with short addresses.
//You should set this value to the maximum number of RFDs that
//use indirect addressing. The value below is just chosen for testing.
//Its minimum value must be the maximum number of neighbors (RFDs+Routers+1), as this
//is also used in the neighbor table construction.
#ifdef LOWSN_COORDINATOR
#define LOWSN_MAX_ADDRESS_MAP_ENTRIES   (LOWSN_MAX_CHILDREN_PER_PARENT*2)
#endif


#ifndef LOWSN_MAX_ADDRESS_MAP_ENTRIES
//this is the minimum value for this, minimum value used by routers
#ifdef LOWSN_ROUTER
#define LOWSN_MAX_ADDRESS_MAP_ENTRIES (LOWSN_MAX_CHILDREN_PER_PARENT+1)
#endif
#ifdef LOWSN_RFD
#define LOWSN_MAX_ADDRESS_MAP_ENTRIES 1
#endif
#endif

#ifdef LOWSN_FFD
#if (LOWSN_MAX_ADDRESS_MAP_ENTRIES < (LOWSN_MAX_CHILDREN_PER_PARENT+1))
#error "In 6lowsn_config.h, LOWSN_MAX_ADDRESS_MAP_ENTRIES too small!"
#endif
#endif



//these precalculated based upon MAX_DEPTH, MAX_ROUTERS, MAX_CHILDREN
//Coord at depth 0, only endpoints are at depth MAX_DEPTH
//LOWSN_CSKIP_(MAX_DEPTH) must be a value of 0.
//this hardcoding supports a max depth of 10, should be PLENTY
//Use the spreadsheet supplied with the distribution to calculate these

#define LOWSN_CSKIP_1     3401
#define LOWSN_CSKIP_2      841
#define LOWSN_CSKIP_3       201
#define LOWSN_CSKIP_4       41
#define LOWSN_CSKIP_5       1
/*#define LOWSN_CSKIP_1     1446
#define LOWSN_CSKIP_2      358
#define LOWSN_CSKIP_3       86
#define LOWSN_CSKIP_4       18
#define LOWSN_CSKIP_5       0*/
#define LOWSN_CSKIP_6       0
#define LOWSN_CSKIP_7       0
#define LOWSN_CSKIP_8       0
#define LOWSN_CSKIP_9       0
#define LOWSN_CSKIP_10      0


#define LOWSN_ADP_MESH_MAXHOP  LOWSN_MAX_DEPTH*2

//Binding
//if the following is defined, then the EVB binding functions use
//the binding resolution functions defined in stack/staticbind.c
//#define LOWSN_USE_DEMO_STATIC_BIND

//Define this if you want to use the binding functions
//in pcbind.c/h that store the binding table on a PC client
//using the bindingdemo application
//#define LOWSN_USE_PC_BIND
//PC_BIND_CACHE_SIZE only needed if USE_PC_BIND is defined
//number of bindings cached by the PC bind code
#define LOWSN_PC_BIND_CACHE_SIZE  4

//these are defaults, can be changed by user
#define LOWSN_APS_ACK_WAIT_DURATION 200  //in milliseconds, for depth=1
#define LOWSN_ADP_JOIN_WAIT_DURATION 200  //in milliseconds!


#define LOWSN_APS_MAX_FRAME_RETRIES 3  //for acknowledge frames.
#define LOWSN_MAC_MAX_FRAME_RETRIES 3  //for MAC ack requests .


//maximum number of endpoints, controls size of endpoint data structure
//in APS.h
#define LOWSN_MAX_ENDPOINTS    6

//data for node descriptor, not  currently used
#define LOWSN_MAX_USER_PAYLOAD   93      //currently 93 bytes.
#define LOWSN_MANUFACTURER_CODE  0x0000  //assigned by Zigbee Alliance



//unsupported at this time
// #define LOWSN_ALT_COORDINATOR
// #define LOWSN_SECURITY_ENABLED

//HAL Stuff

//将TIM2优先级设定为最高了，将会打断其他中断，不宜再为用户提供
// 基于TIM2的比较中断的TIMER了，用户可选其他TIMER进行使用
// 协议栈代码中是支持SLOW TIMER的，只是不推荐使用。

#define LOWSN_ENABLE_SLOW_TIMER

#define SLOWTICKS_PER_SECOND  1
#define LOWSN_DEFAULT_BAUDRATE 115200
#define LOWSN_ASYNC_RX_BUFSIZE   150

#define LOWSN_ASYNC_INTIO




/*************************************************


IPv6 options


**************************************************/



/**
 * The type of the node.
 *
 * config the type of the node (router or host), default host.
 *
 * \hideinitializer
 */
//#undef LOWSN_CONF_ROUTER
//#define LOWSN_CONF_ROUTER		0




/*------------------------------------------------------------------------------*/
/**
 * \defgroup lowsnoptip IPv6 configuration options
 * @{
 *
 */

/** Turn on support for IPv6 packet reassembly.(default: no) */
#ifndef LOWSN_CONF_REASSEMBLY
#define LOWSN_CONF_REASSEMBLY      0
#endif

#ifndef LOWSN_CONF_IPV6_QUEUE_PKT
/** Do we do per %neighbor queuing during address resolution (default: no) */
#define LOWSN_CONF_IPV6_QUEUE_PKT       1
#endif

#ifndef LOWSN_CONF_IPV6_CHECKS
/** Do we do IPv6 consistency checks (highly recommended, default: yes) */
#define LOWSN_CONF_IPV6_CHECKS          1
#endif

#ifndef LOWSN_CONF_IPV6_REASSEMBLY
/** Do we do IPv6 fragmentation (default: no) */
#define LOWSN_CONF_IPV6_REASSEMBLY      0
#endif

#ifndef LOWSN_CONF_NETIF_MAX_ADDRESSES
/** Default number of IPv6 addresses associated to the node's interface */
#define LOWSN_CONF_NETIF_MAX_ADDRESSES  3
#endif

#ifndef LOWSN_CONF_DS6_PREFIX_NBU
/** Default number of IPv6 prefixes associated to the node's interface */
#define LOWSN_CONF_DS6_PREFIX_NBU     2
#endif

#ifndef LOWSN_CONF_DS6_NBR_NBU
/** Default number of neighbors that can be stored in the %neighbor cache */
#define LOWSN_CONF_DS6_NBR_NBU    4
#endif

#ifndef LOWSN_CONF_DS6_DEFRT_NBU
/** Minimum number of default routers */
#define LOWSN_CONF_DS6_DEFRT_NBU       2
#endif
/** @} */


/*------------------------------------------------------------------------------*/
/**
 * \defgroup lowsnoptip UDP configuration options
 * @{
 *
 */
 /**enable or disable the UDP header check. default: disable 0*/
#ifndef LOWSN_CONF_UDP_CHECK
#define LOWSN_CONF_UDP_CHECK	1
#endif

/*define the number of udp connections used in the protocol*/
//#ifndef LOWSN_CONF_UDP_CONNS
//#define LOWSN_CONF_UDP_CONNS
//#endif


#define LOWSN_DEFAULT_RS_WAIT_DURATION   1000  //in milliseconds!

#define NWK_DEFAULT_PING_WAIT_TIME       1000


//  在使用global prefix时，interface ID的产生方式
//  0: from EUI 64; 1: from short address and panid

#define LOWSN_IID_METHOD_FOR_GLOABL   1

// 定义是否需要IEEE 802.15.4的入网过程
//#define  LOWSN_NO_JOIN_PROCESS  

//定义是否需要首先获取全局前缀过程
//#define  LOWSN_NO_PREFIX_PROCESS  

// 定义是否需要MESH头部
//#define  LOWSN_NO_MESH_HEADER  


// 定义是否放松PHY和MAC的检查, 仅用于调试
//#define LOWSN_NO_CHECK_PHYMAC

//定义发出的数据包中MAC地址的表示形式:
// 1:强制使用短地址; 2:强制使用长地址; 0或其他值: 自动选择，当目标地址是长地址时，用长地址
// 当目标地址是短地址时，用短地址.

#define LOWSN_SRC_IP_MODE          0

//同样的方法适用于判断由长地址或短地址生成的IP地址. 0:目标IP地址是长地址时，源地址使用长地址生成
//的本地IP，目标地址是短地址时，源地址使用短地址生成的IP; 1: 强制使用短地址生成的IP; 2: 强制使用长地址
//生成的IP 

#define LOWSN_SRC_MAC_MODE     0

//开启该选项后，在SLIP模式下，系统仍然向串口发送调试信息，默认为关闭
#define   LOWSN_COM_KEEP_OUTPUT

#define LOWSN_RA_COMPRESS
// 原始RA中所含的负载太长，开启该选项后，将对用处不大的RA选项进行压缩


//#define LOWSN_MANUAL_MAC_ACK 
//开启该选项后，协议栈将自己发送MAC层的ACK。对于硬件不支持自动ACK的芯片，例如CC1101等，可选择协议栈
// 自发ACK。对于硬件支持自动ACK的CC2530等芯片，不需要开启。

 //#define LOWSN_SLIP_COM_QUIET 
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
#ifndef LOWSN_COM_KEEP_OUTPUT
  #define LOWSN_SLIP_COM_QUIET 
#endif
#endif
#endif



#endif
