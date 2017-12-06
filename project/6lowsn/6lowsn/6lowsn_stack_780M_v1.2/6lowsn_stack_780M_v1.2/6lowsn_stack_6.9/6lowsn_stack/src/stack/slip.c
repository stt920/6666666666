
/******************************************************************************************************
*
* 文 件 名：slip.c
*
* 文件描述： SLIP驱动及与Host之间的接口
*
* 创 建 者：Wang Heng
*
* 当前版本：0.50
*
* 修 改 者：
*
* 修改历史：


********************************************************************************************************/

#include "compiler.h"          
#include "6lowsn_common_types.h"
#include "6lowsn_config.h"
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
#include "ds.h"
#include "slip.h"


#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST


#define SERIAL_INPUT_TIMEOUT MSECS_TO_MACTICKS(400)


SLIP_PIB slip_pib;
BYTE slipbuf[MAX_SLIP_BUF_SIZE];
UINT16 slipbuf_len;

SLIP_STATE_ENUM slipState;
SLIP_SERVICE a_slip_service;


BOOL slipGetchWithTimeOut(BYTE *c)
{
	UINT32 my_timer;

	if (halGetchRdy()) {
            *c = halGetch();
			return(TRUE);
	}
	//now do timeout
	my_timer= halGetMACTimer();
	do {
		if (halGetchRdy()) {
			*c = halGetch();
			return(TRUE);
		}
	}while ((halMACTimerNowDelta(my_timer))< SERIAL_INPUT_TIMEOUT);
	*c = 0;  //return reasonable default
    return (FALSE);
}


void slipInit(void)
{
	slip_pib.flags.val = 0;
	//slip_pib.forward_kind = SLIP_FORWARD_NONE;
	slip_pib.forward_kind = SLIP_FORWARD_IP_PACKET;
	slip_pib.aps_forward = 1;
	slipFlushBuf(MAX_SLIP_BUF_SIZE);
	slipState = SLIP_STATE_IDLE;

}


// 
/*------------------------------------------------------------------------------------*

Slip发送函数

flag用于控制是否发送起始的END和结束的END


0  : 有起始END，有结束END
1  : 无起始END，有结束END
2  : 有起始END，无结束END
3  : 无起始END，无结束END
>=4: 有起始END，有结束END


*------------------------------------------------------------------------------------*/
void slipSend(BYTE *pbuf, UINT16 len, UINT8 flag)
{
  if(len>128)
  {
    return;
  }
	if (flag > 3) {
		flag = 0;
	}	

	if ((flag == 0) || (flag == 2)) {
		halRawPut(END);
	}

       while(len--) {
	   	
		switch(*pbuf) {
			
                   case END:
                           halRawPut(ESC);
                           halRawPut(ESC_END);
                           break;

                   case ESC:
                           halRawPut(ESC);
                           halRawPut(ESC_ESC);
                           break;
						   
                   default:
                           halRawPut(*pbuf);
              }
		
              pbuf++;
       }

	if ((flag == 0) || (flag == 1))  {
      		halRawPut(END);
	}		
		   
}

// 返回接收数据的长度

UINT16 slipRcv(void)
{

	BYTE c;
	int received = 0;

	UINT32 slip_rx_timer = halGetMACTimer();;
	

	// 注意: 这个while死循环有可能跳不出来，尤其在线路出错的情况下
	// 应该加入定时器超时控制才更加合理
	while(1) {

		if( TRUE == slipGetchWithTimeOut(&c) )
		{
			//这里未考虑出错，第一个字节不为END 符情况，不合理
			switch(c) {

				case END:

					if(received)
						return received;
					else
						break;

				case ESC:
					slipGetchWithTimeOut(&c);

					switch(c) {
						case ESC_END:
							c = END;
							break;
						case ESC_ESC:
							c = ESC;
							break;
					}

				default:
					if(received < MAX_SLIP_BUF_SIZE)
						slipbuf[received++] = c;
					else{
						slipFlushBuf(MAX_SLIP_BUF_SIZE-1);
						return 0;
					}
			}
		}

		if((halMACTimerNowDelta(slip_rx_timer)) > MSECS_TO_MACTICKS(10*1000))
		{
			slipFlushBuf(received);
			return 0;
		}
					
	}
}


void slipRequestPrefix(void)
{
	BYTE c;

	c = SLIP_CMD_QUERY_FIRST;
	slipSend(&c, 1, 2);
	
	c = SLIP_CMD_PREFIX_REQ;
	slipSend(&c, 1, 1);

	return;
	
}

BOOL slipParsePrefix(IPADDR *prefix, UINT8 *p_len)
{
        BYTE i;

	if ((slipbuf[0] == SLIP_CMD_INFO_FIRST) && (slipbuf[1] == SLIP_CMD_PREFIX_REQ) ) {
		
		// 格式: 第3-10个字节为64位前缀，目前限制前缀只能是64位，且只传输前缀，不传输整个IPv6地址
		for (i=0; i<8; i++)  {
			// 需审查大小端模式
			prefix->u8[i] = slipbuf[2+i];
		}	

		for (i=8; i<16; i++)  {
			prefix->u8[i] = 0;
		}	

		*p_len = 64;
		
		return 0;
	}	
	else  {
		return 1;
	}	
}	

// 处理在网络运行过程中收到的SLIP专用命令帧
// 因为获取前缀是协调器运行前提，所以将前缀获取独立出来
// 目前其他命令暂空，包括更改前缀命令
void slipParseCmd(UINT16 len)
{


}


void slipFlushBuf(UINT16 len)
{
	UINT16 i;
	
	for (i=0;i<len;i++) {
		slipbuf[i] = 0;
	}	
}	

void slipFSM(void)
{

	UINT16 len;
	BYTE i;
	IPADDR SrcAddress, DstAddress;
	BYTE serviceID; 

adpFSM_start:

	switch (slipState) {
		
	case SLIP_STATE_IDLE:
	 	
	if (slipReady())  {
		
		len = slipRcv();
		
		if(len != 0){
				
			if (slipIsCmdPkt())  {
				// 专用命令帧
				slipParseCmd(len);
			}	
			
		 	else if (slipIsIpPkt6())  {

				DEBUG_STRING(DBG_INFO,"SLIP IP: Receive IPv6 packet \n");
				// IPv6数据包

				// 检查长度是否正确
				if (  (((((UINT16)(slipbuf[4])) << 8) | slipbuf[5]) + LOWSN_IPH_LEN) != len ) {
					DEBUG_STRING(DBG_INFO,"SLIP IP: Wrong IP length! \n");
					slipFlushBuf(len);
					break;
				}

				DEBUG_STRING(DBG_INFO,"SLIP IP: IPv6 length OK.  \n");

				// 注意: 未考虑分片问题! 要求主机发来的数据包必须小于100个字节，否则程序会崩溃
				// 100个字节是大致估计，即总的数据包不能超过127个字节。待网络层IP分片功能重新加入后再考虑更改
				if ( len > 100) {
					DEBUG_STRING(DBG_INFO,"SLIP IP: Host IP should smaller than 100, current limitation. \n");
					slipFlushBuf(len);
					break;
				}

				
				// 获取地址进行以便进行比对
				for(i=0; i<16; i++) {
	              		SrcAddress.u8[i] = slipbuf[8+i];
					DstAddress.u8[i] = slipbuf[24+i];
	       		}

			
				if(lowsn_is_addr_mcast(&SrcAddress)){
					DEBUG_STRING(DBG_INFO,"SLIP IP: Discard the packet : Muliticast src IP Addr.\n");
					slipFlushBuf(len);
					break;
				}

				if(!lowsn_ds6_is_my_uaddr(&DstAddress)&&
			   	!lowsn_ds6_is_my_maddr(&DstAddress)&&
	                	 !lowsn_ds6_is_my_aaddr(&DstAddress)) {
	                 	//目标节点不是协调器，inject到发送队列中，转往MESH子网.
	                 	// 目前地址识别过于简单，虽然不是转给协调器的，但应判断是否应该转入子网,
	                 	// 若不是转入子网的，应丢弃。以后添加.
	                 	
					DEBUG_STRING(DBG_INFO,"SLIP IP: Dst is a node in the subnet. Inject to nwk tx buf. \n");
					nwk_pib.flags.bits.slipforwardPending = 1;
					slipState = SLIP_STATE_INJECT_NWK_TX_WAIT;
					break;

						
				
				}
				else  {
				// 目标地址就是协调器
					DEBUG_STRING(DBG_INFO,"SLIP IP: Dst is the coord. Inject to nwk RxFSM. \n");
					nwk_pib.flags.bits.slipRxPending = 1;
					slipState = SLIP_STATE_INJECT_NWK_RX_WAIT;
					goto adpFSM_start;


				}	


		
		 	}	
			else  if (slipIsApsPkt())  {
				// 应用层数据包

				// 检查方向是否Host-Coord
				if (slipbuf[2] != SLIP_APS_HOST_TO_COORD) {
					DEBUG_STRING(DBG_INFO,"SLIP APS: Not a valid Host to Coord Packet.\n");
					slipFlushBuf(len);
					break;
				}	

				// 校验数据长度是否正确
				if (  (((((UINT16)(slipbuf[31])) << 8) | slipbuf[32]) + SLIP_APS_TO_HOST_HEADER_LEN) != len) {
					DEBUG_STRING(DBG_INFO,"SLIP APS: Wrong APS packet length! \n");
					slipFlushBuf(len);
					break;
				}
				
				if (slipbuf[3] == SLIP_APS_DATA_FRAME)  {
					
					slipState = SLIP_STATE_INJECT_APS_TX;
					goto adpFSM_start;
				
				}

				else if (slipbuf[3] == SLIP_APS_MP_FRAME)  {

					// 暂时采用Host实现管理进程的方式, 协调器只是透传
					// 将来完善协调器端管理进程+ Host端管理命令的方式
					slipState = SLIP_STATE_INJECT_APS_TX;
					goto adpFSM_start;

				}

				else  {
					
					DEBUG_STRING(DBG_INFO,"SLIP APS: Wrong frame kind. \n");
					slipFlushBuf(len);
					break;

				}	
							
			}	

			else if (slipIsMacPkt())  {
				// 注入的IEEE 802.15.4数据包
				DEBUG_STRING(DBG_INFO,"SLIP: Not support inject MAC packets now. \n");
				slipFlushBuf(len);

			}	

			else  {
				
				DEBUG_STRING(DBG_INFO,"SLIP: Invalid frame kind. \n");
				slipFlushBuf(len);
			}
		}
		
	}	

	break;

	case SLIP_STATE_INJECT_APS_TX:
		
		if (apsBusy())  break;
		
		// 判断是否需自动产生service ID
		
		if (slipbuf[27] == 0xFF) {
			serviceID = apsGenServiceID();
		}
		else  {

			serviceID = slipbuf[27];
		}	

		switch (slipbuf[4]) {
					
	 		case APS_SERVICE_TYPE_PUBLISH:
			aplSendPublishDataNoFSM(*((IPADDR *)(&slipbuf[5])), ((UINT16)(slipbuf[21])) << 8 | ((UINT16)(slipbuf[22])), slipbuf[23], ((UINT16)(slipbuf[24])) << 8 | ((UINT16)(slipbuf[25])), slipbuf[26], 
						                            serviceID, slipbuf[28], &(slipbuf[33]), ((UINT16)(slipbuf[31])) << 8 | ((UINT16)(slipbuf[32])));
			slipState = SLIP_STATE_INJECT_APS_DATA_WAIT;
		        goto adpFSM_start;
 
			case APS_SERVICE_TYPE_REPORT:
			aplSendReportDataNoFSM(*((IPADDR *)(&slipbuf[5])), ((UINT16)(slipbuf[21])) << 8 | ((UINT16)(slipbuf[22])), slipbuf[23], ((UINT16)(slipbuf[24])) << 8 | ((UINT16)(slipbuf[25])), slipbuf[26], 
						                              serviceID, slipbuf[28], &(slipbuf[33]), ((UINT16)(slipbuf[31])) << 8 | ((UINT16)(slipbuf[32])));
		        slipState = SLIP_STATE_INJECT_APS_DATA_WAIT;
		        goto adpFSM_start;

	 		case APS_SERVICE_TYPE_CS:
			aplSendCSDataNoFSM(*((IPADDR *)(&slipbuf[5])), ((UINT16)(slipbuf[21])) << 8 | ((UINT16)(slipbuf[22])), slipbuf[23], ((UINT16)(slipbuf[24])) << 8 | ((UINT16)(slipbuf[25])), slipbuf[26], 
						                           serviceID, slipbuf[28], &(slipbuf[33]), ((UINT16)(slipbuf[31])) << 8 | ((UINT16)(slipbuf[32])), slipbuf[29], slipbuf[30]);
			slipState = SLIP_STATE_INJECT_APS_DATA_WAIT;
		        goto adpFSM_start;

			default:
				DEBUG_STRING(DBG_INFO,"SLIP APS: Not a vailid APS kind.\n");
				slipFlushBuf(len);
			break;
						
		}
	
	break;

	case SLIP_STATE_INJECT_APS_DATA_WAIT:

		if (apsBusy()) break; 
		
		if (aplGetStatus() == LOWSN_STATUS_SUCCESS) {
			slipState = SLIP_STATE_IDLE;
		}
		else {
			DEBUG_STRING(DBG_INFO,"SLIP APS: Send Failed. \n");
			// 重传, 将来完善
			slipState = SLIP_STATE_IDLE;
		}
		
	break;

	case SLIP_STATE_INJECT_NWK_RX_WAIT:
		
		if (nwk_pib.flags.bits.slipRxPending)   break;
		
		if (a_slip_service.status == LOWSN_STATUS_SUCCESS)  {
			// 状态成功仅表明正确放入NWK RX接收流程中了,
			// 无法确保应用层及其以上能够正常收到
			slipState = SLIP_STATE_IDLE;
		}
		else {
			// 进入网络层接收状态机失败，将来考虑失败处理
			slipState = SLIP_STATE_IDLE;
		}	
	 	
	break;

	case SLIP_STATE_INJECT_NWK_TX_WAIT:

		if (nwk_pib.flags.bits.slipforwardPending)   break;
		 //inject packet into TX FSM
		if (a_slip_service.status == LOWSN_STATUS_SUCCESS)  {
			slipState = SLIP_STATE_IDLE;
		}
		else {
			// 重传或告诉Host发不出去     TODO
			slipState = SLIP_STATE_IDLE;
		}	
		 	
	break;
	

	default:  break;
	} 

}



#endif
#endif


