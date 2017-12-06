
/******************************************************************************************************
*
* 文 件 名：mp.c
*
* 文件描述：网络管理进程
*
* 创 建 者：Wang Heng
*
* 当前版本：0.50
*
* 修 改 者：
*
* 修改历史：



********************************************************************************************************/

#include "compiler.h"               //compiler specific
#include "6lowsn_common_types.h"
#include "6lowsn_config.h"
#include "ieee_lrwpan_defs.h"
#include "hal.h"
#include "halStack.h"
#include "console.h"
#include "debug.h"
#include "phy.h"
#include "mac.h"
#include "adp.h"
#include "nwk.h"
#include "aps.h"
#include "mp.h"
#include "neighbor.h"
#include "evboard.h"
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST
#include "slip.h"
#endif
#endif


MP_PIB mp_pib;
MP_RESPONSE_PARAM mp_rsp_param;

static void mpCommonFmt(IPADDR dst_saddr, BYTE command_id, BYTE aps_fcf, BYTE len);


void mpInit(void)
{
	mp_pib.flags.val = 0;
	mp_pib.eventType = MP_EVENT_NONE; 
	mp_pib.sensorType = LOWSN_SENSOR_TYPE;
	// 设置协调器为默认的管理信息发送地址, 依然会根据具体的数据包选择目标地址
	aplSetLinkLocalAddr16(&mp_pib.dstAddr, aplGetPanID(), 0);
	
	return;
}


LOWSN_STATUS_ENUM mpHandleRxReport(void) 
{
	LOWSN_STATUS_ENUM rstatus;
        BYTE *ptr;

	rstatus = LOWSN_STATUS_SUCCESS;
	ptr = aplGetRxMsgData();

	if (a_aps_rx_data.dstObjID >= APS_SHORTCUT_BASE)  {

		// 采用了SHORTCUT 机制
		DEBUG_STRING(DBG_INFO,"MP: Find Shortcut. \n");
		switch (a_aps_rx_data.srcObjID) {
			
			case MP_COMMAND_DEVICE_INFORMATION:
				   rstatus = mpHandleDeviceInfo();
				   break;

			case MP_COMMAND_TREE_NEIGHBOR_INFORMATION:
				   rstatus = mpHandleTreeNeighborInfo();
				   break;

		       default:
				   DEBUG_STRING(DBG_INFO,"MP: Unhandled MP Command, discarding.\n");
		}


	}
	else  {

		//普通对象读写

		DEBUG_STRING(DBG_INFO,"MP: Not support common object read/wirte now. \n");

	}
	 
	return(rstatus);
	
}


// 若是后台管理，则暂时不作动作
LOWSN_STATUS_ENUM mpHandleDeviceInfo(void)
{
	BYTE *ptr;
	SADDR saddr;

	ptr = aplGetRxMsgData();  

	return(LOWSN_STATUS_SUCCESS);
}


// 若是后台管理，则暂时不作动作
LOWSN_STATUS_ENUM mpHandleTreeNeighborInfo(void)
{
	BYTE *ptr;
	SADDR saddr;

	ptr = aplGetRxMsgData();  

	return(LOWSN_STATUS_SUCCESS);
}



/*---------------------------------------------------------------------*

构造管理进程命令帧信息

注意: FCF域要求各命令帧自己构造好直接传递过来

*---------------------------------------------------------------------*/
static void mpCommonFmt(IPADDR dst_saddr, BYTE command_id, BYTE aps_fcf, BYTE len)
{

	a_aps_tx_data.flags.val = 0;
	a_aps_tx_data.srcObjID = command_id;
	a_aps_tx_data.usrPlen = len;
	a_aps_tx_data.serviceID = apsGenServiceID();
	a_aps_tx_data.seqNum= aps_pib.apsSeqNum++;
	a_aps_tx_data.dstObjID = APS_SHORTCUT_BASE;
	
	a_aps_tx_data.dstPort = LOWSN_BASE_PORT;
	a_aps_tx_data.dstIPADDR = dst_saddr;
	a_aps_tx_data.srcPort = LOWSN_BASE_PORT;
	
	a_aps_tx_data.fcf = aps_fcf;
	a_aps_tx_data.flags.bits.loopback = 0; 

}

/*---------------------------------------------------------------------*

构造C/S 响应帧信息

*---------------------------------------------------------------------*/
static void mpRspCommonFmt(BYTE len)
{

	a_aps_tx_data.flags.val = 0;
	a_aps_tx_data.srcObjID = aplGetRxDstObjectID();
	a_aps_tx_data.usrPlen = len;
	a_aps_tx_data.serviceID = aplGetRxServiceID();
	a_aps_tx_data.seqNum= aps_pib.apsSeqNum++;
	a_aps_tx_data.dstObjID = aplGetRxSrcObjectID();
	
	a_aps_tx_data.dstPort = LOWSN_BASE_PORT;
	a_aps_tx_data.dstIPADDR = aplGetRxSrcIPAddr();
	a_aps_tx_data.srcPort = LOWSN_BASE_PORT;

	a_aps_tx_data.fcf = 0;
	APS_SET_SERVICE_TYPE(a_aps_tx_data.fcf, APS_SERVICE_TYPE_CS);
	APS_SET_ACTION_TYPE(a_aps_tx_data.fcf, aplGetRxActType());
	APS_SET_ACTION_RESPONSE(a_aps_tx_data.fcf);

	//APS_SET_FCF_ACK_FLAG(a_aps_tx_data.fcf);  

	a_aps_tx_data.flags.bits.loopback = 0; 

}


/*---------------------------------------------------------------------*

发送设备信息

格式为:  应反向填写

节点短地址：2 Bytes
节点深度：1 Byte
节点父节点短地址：2 Bytes
节点EUI64地址： 8 Bytes
节点能力：1 Byte  （内含节点角色，FFD/RFD，是否安全，是否电池供电等）
节点种类代号：用户自定义  1 Byte


*---------------------------------------------------------------------*/
void mpFmtDeviceInfo(IPADDR dst_ipaddr)
{
	BYTE *ptr;
	BYTE len=0, fcf=0;
	UINT32 timer_temp;        
	timer_temp = halGetMACTimer();
	//wait for 2 seconds
	#ifdef RFD
		while ((halMACTimerNowDelta(timer_temp))< MSECS_TO_MACTICKS(50))
        	{}
	#endif
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];	

	// 节点前缀列表
	// TO DO.

	// 节点应用序列号

	// 节点种类代号
	ptr--;
	*ptr = LOWSN_SENSOR_TYPE;
	len++;

	//节点能力
	ptr--;
	*ptr = mac_pib.macCapInfo;
	len++;

	// 节点长地址
	ptr = ptr - 8;
	halGetProcessorIEEEAddress(ptr);
	len = len + 8;

	// 节点父节点短地址
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress());
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress()>>8);
	len = len + 2;

	//节点深度
	ptr--;
	*ptr = aplGetMyDepth();
	len++;
	

	// 节点短地址
	ptr--;
	*ptr = (BYTE) (macGetShortAddr());
	ptr--;
	*ptr = (BYTE) (macGetShortAddr()>>8);
	len = len + 2;


	// 过早限制了服务类型，以后考虑改进
	APS_SET_SERVICE_TYPE(fcf, APS_SERVICE_TYPE_REPORT);
	//APS_SET_FCF_ACK_FLAG(fcf);  

	mpCommonFmt(dst_ipaddr, MP_COMMAND_DEVICE_INFORMATION, fcf, len);
		
}


/*---------------------------------------------------------------------*

发送拓扑信息(邻居节点信息)

格式为:  应反向填写

节点自身短地址：2 Bytes
节点深度：1 Bytes
节点EUI64地址： 8 Bytes
节点能力：1 Bytes （主要用于判断节点是路由器还是终端设备）
节点父节点短地址：2 Bytes
节点父节点EUI64地址：8 Bytes
节点子节点个数：2 Bytes
节点子节点短地址： 2 Bytes*个数
节点子节点长地址：8 Bytes*个数





*---------------------------------------------------------------------*/
void mpFmtTreeNeighborInfo(IPADDR dst_ipaddr)
{
	BYTE *ptr;
	BYTE len=0, fcf=0;
	BYTE i;

	
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];	

	#if 0  //暂不汇报子节点
	#ifdef LOWSN_RFD
	ptr--;
	*ptr = 0;
	len++;

	#else



	#endif
	#endif

	// 节点父节点长地址
	for (i=0; i<8; i++) {
		ptr--;
		*ptr = mac_pib.macCoordExtendedAddress.bytes[7-i];
	}	
	len = len + 8;

	// 节点父节点短地址
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress());
	ptr--;
	*ptr = (BYTE) (aplGetParentShortAddress()>>8);
	len = len + 2;

	//节点能力
	ptr--;
	*ptr = mac_pib.macCapInfo;
	len++;

	// 节点长地址
	ptr = ptr - 8;
	halGetProcessorIEEEAddress(ptr);
	len = len + 8;
	
	//节点深度
	ptr--;
	*ptr = aplGetMyDepth();
	len++;
	
	// 节点短地址
	ptr--;
	*ptr = (BYTE) (macGetShortAddr());
	ptr--;
	*ptr = (BYTE) (macGetShortAddr()>>8);
	len = len + 2;

	
	// 过早限制了服务类型，以后考虑改进
	APS_SET_SERVICE_TYPE(fcf, APS_SERVICE_TYPE_REPORT);
	//APS_SET_FCF_ACK_FLAG(fcf);  

	mpCommonFmt(dst_ipaddr, MP_COMMAND_TREE_NEIGHBOR_INFORMATION, fcf, len);
		
}



// 检查发来读取管理信息的指令是否有效
// TRUE: 有效;  FALSE: 无效
BOOL mpReadCheck(void)
{
	BYTE *ptr;
	ptr = aplGetRxMsgData();

	// 暂时只支持单个变量的操作
	if (*ptr != 1)  {
		
		DEBUG_STRING(DBG_INFO,"MP: Only support single value opertion. \n");
		return FALSE;
	}

	// 可在此对支持的管理对象ID进行进一步的限制 

	
	return TRUE;
}


// 检查发来写管理信息的指令是否有效
// TRUE: 有效;  FALSE: 无效
BOOL mpWriteCheck(void)
{
	BYTE *ptr;
	ptr = aplGetRxMsgData();

	// 暂时只支持单个变量的操作
	if (*ptr != 1)  {
		
		DEBUG_STRING(DBG_INFO,"MP: Only support single value opertion. \n");
		return FALSE;
	}

	// 可在此对支持的管理对象ID进行进一步的限制 

	
	return TRUE;
}


// 检查发来的执行指令是否有效
// TRUE: 有效;  FALSE: 无效
BOOL mpExecuteCheck(void)
{
	BYTE *ptr;
	ptr = aplGetRxMsgData();

	// 暂时只支持单个操作
	if (*ptr != 1)  {
		
		DEBUG_STRING(DBG_INFO,"MP: Only support single value opertion. \n");
		return FALSE;
	}

	// 可在此对支持的管理对象ID进行进一步的限制 

	
	return TRUE;
}



UINT8 mpFmtCurrentVoltageResponsePayload(void)
{

	BYTE *ptr;
	BYTE len=0;
	UINT16 voltage;

	voltage = getVoltageValue();

	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];	

	// 填写电池电压值
	ptr--;
	*ptr = (BYTE) (voltage);
	ptr--;
	*ptr = (BYTE) (voltage >> 8);
	len = len + 2;

	//填写属性值长度
	ptr--;
	*ptr = 2;
	len++;

	// 填写二级属性ID和属性ID
	ptr--;
	*ptr = 0;
	ptr--;
	*ptr = MP_ATTRIBUTE_ID_CURRENT_VOLTAGE;
	len = len + 2;

	// 填写执行成功的结果和执行结果个数
	ptr--;
	*ptr = APS_RESULT_OK;
	ptr--;
	*ptr = 1;
	len = len + 2;

	return len;

}

void mpFmtReadResponse(void)
{
	BYTE *ptr;
	UINT8 plen;

	ptr = aplGetRxMsgData();
	
	if  (aplGetRxDstObjectID() ==  MP_OBJECT_ID_POWER)   {

		if (*(ptr+1) == MP_ATTRIBUTE_ID_CURRENT_VOLTAGE) {
					
			plen = mpFmtCurrentVoltageResponsePayload();
		}	

		else
		{
			// 不应该执行该语句，对系统暂时不支持的对象和属性，
			// mpReadCheck()函数应该已经负责检查了
			// 应通知系统出错了
			DEBUG_STRING(DBG_INFO,"MP: Sytem Error, Invalid attribute ID for  power management. \n");
			plen = 0;
			

		}
	}	

	else  {
		
			DEBUG_STRING(DBG_INFO,"MP: Sytem Error, Invalid object ID for READ. \n");
			plen = 0;
	}
					
	if (plen > 0)  {
		
		mpRspCommonFmt(plen);
		
	} 
	else  {

		// 不应该出现的分支，以后进行系统出错处理
	}	
		
	return;
	
}


// 与mpFmtReadResponse()写法类似
void mpFmtWriteResponse(void)
{
	BYTE *ptr;

	ptr = aplGetRxMsgData();
	
	return;

}

// 与mpFmtReadResponse()写法类似
void mpExcuteMethod(void)
{

	BYTE *ptr;

	ptr = aplGetRxMsgData();
		
	return;

}


#if 0

void zepFSM(void) {
	//does not do much right now

	if (zep_pib.flags.bits.Alarm) {
		//flash LED1 at 200ms rate
		if ((halMACTimerNowDelta(zep_pib.alarm_timer)) > MSECS_TO_MACTICKS(200)) {
			//toggle LED1
			if (EVB_LED1_STATE()) {
		     EVB_LED1_OFF();
			 } else {
                  EVB_LED1_ON();
				 }
            zep_pib.alarm_timer = halGetMACTimer();
		}


	}

}


//send a payload that has our short address, so that a ping
//can be used to send our short address to somebody
//The main goal is to the APS ACK back
//as verification that this was delivered
//this ping packet is also forwarded to the PC client
//if PC binding is being used.

void zepFmtPing(SADDR dst_saddr) {
	BYTE *ptr;
   	a_aps_tx_data.usrPlen = 3;
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
    *ptr = (BYTE) (macGetShortAddr()>>8);
    --ptr;
    *ptr = (BYTE) (macGetShortAddr());
    --ptr;
	*ptr = ZEP_EXT_PING;
	zepCommonFmt(dst_saddr,ZEP_EXTENDED_CMD);
}

//turns an alarm on at the node
//if mode is nonzero, turn on the alarm
//if mode is zero, turn off the alarm
void zepFmtAlarm(SADDR dst_saddr, BYTE mode) {
	BYTE *ptr;
   	a_aps_tx_data.usrPlen = 2;
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE-1];
    *ptr = mode;
    --ptr;
	*ptr = ZEP_EXT_SEND_ALARM;
	zepCommonFmt(dst_saddr,ZEP_EXTENDED_CMD);
}



//put our long address, short address into the tmpTxBuffer
//so that it can be sent to the coordinator.
void zepFmtEndDeviceAnnounce(SADDR dst_saddr){
	BYTE *ptr;

	//first, do the payload
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE] - 8;	
	//copy in the long address
	halGetProcessorIEEEAddress(ptr);
	//now put our short address
	--ptr;
    *ptr = (BYTE) (macGetShortAddr()>>8);

    --ptr;
    *ptr = (BYTE) (macGetShortAddr());
	
    a_aps_tx_data.usrPlen = ZEP_PLEN_END_DEVICE_ANNOUNCE;

	//now, the rest
	zepCommonFmt(dst_saddr,ZEP_END_DEVICE_ANNOUNCE);
		
}

/* this is a custom Zero End point command that sends data that is spread
across several different Zigbee response commands into one. This info
is needed by the PC client in the binding demo
*/
void zepFmtNodeInfoRsp(SADDR dst_saddr){

	BYTE *ptr;
	BYTE i;

	//endpoints first
	ptr = &tmpTxBuff[LOWSN_MAX_FRAME_SIZE];
	//do the endpoints first.
	for (i=0;i<aps_pib.activeEPs;i++){
		--ptr;
		*ptr = apsEndPoints[i].epNum;
	}
    --ptr;
	*ptr = aps_pib.activeEPs;
	
	//MAC capability code
    --ptr;
	*ptr = mac_pib.macCapInfo;

	//Node type
    --ptr;
#if defined(LOWSN_COORDINATOR)
	*ptr = NODE_TYPE_COORD;
#elif defined(LOWSN_FFD)
	*ptr = NODE_TYPE_ROUTER;
#else
    *ptr = NODE_TYPE_ENDDEVICE;
#endif
    //my parent
	  --ptr;
	*ptr = (mac_pib.macCoordShortAddress >> 8) & 0xFF;
	

	--ptr;	
    *ptr = mac_pib.macCoordShortAddress & 0xFF;


	//my short address
    --ptr;
	*ptr = (macGetShortAddr() >> 8) & 0xFF;
	

	--ptr;	
    *ptr = macGetShortAddr() & 0xFF;

	//copy in the long address
	ptr = ptr - 8;
	halGetProcessorIEEEAddress(ptr);

	//indentify this extended command
	--ptr;
	*ptr = ZEP_EXT_NODE_INFO_RSP;

	a_aps_tx_data.usrPlen = aps_pib.activeEPs + ZEP_PLEN_NODE_INFO_RSP ;

	//now, the rest
	zepCommonFmt(dst_saddr,ZEP_EXTENDED_CMD);

}

#endif



