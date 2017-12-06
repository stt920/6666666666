
/******************************************************************************************************
*
* �� �� ����slip.c
*
* �ļ������� SLIP��������Host֮��Ľӿ�
*
* �� �� �ߣ�Wang Heng
*
* ��ǰ�汾��0.50
*
* �� �� �ߣ�
*
* �޸���ʷ��


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

Slip���ͺ���

flag���ڿ����Ƿ�����ʼ��END�ͽ�����END


0  : ����ʼEND���н���END
1  : ����ʼEND���н���END
2  : ����ʼEND���޽���END
3  : ����ʼEND���޽���END
>=4: ����ʼEND���н���END


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

// ���ؽ������ݵĳ���

UINT16 slipRcv(void)
{

	BYTE c;
	int received = 0;

	UINT32 slip_rx_timer = halGetMACTimer();;
	

	// ע��: ���while��ѭ���п���������������������·����������
	// Ӧ�ü��붨ʱ����ʱ���ƲŸ��Ӻ���
	while(1) {

		if( TRUE == slipGetchWithTimeOut(&c) )
		{
			//����δ���ǳ�����һ���ֽڲ�ΪEND �������������
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
		
		// ��ʽ: ��3-10���ֽ�Ϊ64λǰ׺��Ŀǰ����ǰ׺ֻ����64λ����ֻ����ǰ׺������������IPv6��ַ
		for (i=0; i<8; i++)  {
			// ������С��ģʽ
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

// �������������й������յ���SLIPר������֡
// ��Ϊ��ȡǰ׺��Э��������ǰ�ᣬ���Խ�ǰ׺��ȡ��������
// Ŀǰ���������ݿգ���������ǰ׺����
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
				// ר������֡
				slipParseCmd(len);
			}	
			
		 	else if (slipIsIpPkt6())  {

				DEBUG_STRING(DBG_INFO,"SLIP IP: Receive IPv6 packet \n");
				// IPv6���ݰ�

				// ��鳤���Ƿ���ȷ
				if (  (((((UINT16)(slipbuf[4])) << 8) | slipbuf[5]) + LOWSN_IPH_LEN) != len ) {
					DEBUG_STRING(DBG_INFO,"SLIP IP: Wrong IP length! \n");
					slipFlushBuf(len);
					break;
				}

				DEBUG_STRING(DBG_INFO,"SLIP IP: IPv6 length OK.  \n");

				// ע��: δ���Ƿ�Ƭ����! Ҫ���������������ݰ�����С��100���ֽڣ������������
				// 100���ֽ��Ǵ��¹��ƣ����ܵ����ݰ����ܳ���127���ֽڡ��������IP��Ƭ�������¼�����ٿ��Ǹ���
				if ( len > 100) {
					DEBUG_STRING(DBG_INFO,"SLIP IP: Host IP should smaller than 100, current limitation. \n");
					slipFlushBuf(len);
					break;
				}

				
				// ��ȡ��ַ�����Ա���бȶ�
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
	                 	//Ŀ��ڵ㲻��Э������inject�����Ͷ����У�ת��MESH����.
	                 	// Ŀǰ��ַʶ����ڼ򵥣���Ȼ����ת��Э�����ģ���Ӧ�ж��Ƿ�Ӧ��ת������,
	                 	// ������ת�������ģ�Ӧ�������Ժ����.
	                 	
					DEBUG_STRING(DBG_INFO,"SLIP IP: Dst is a node in the subnet. Inject to nwk tx buf. \n");
					nwk_pib.flags.bits.slipforwardPending = 1;
					slipState = SLIP_STATE_INJECT_NWK_TX_WAIT;
					break;

						
				
				}
				else  {
				// Ŀ���ַ����Э����
					DEBUG_STRING(DBG_INFO,"SLIP IP: Dst is the coord. Inject to nwk RxFSM. \n");
					nwk_pib.flags.bits.slipRxPending = 1;
					slipState = SLIP_STATE_INJECT_NWK_RX_WAIT;
					goto adpFSM_start;


				}	


		
		 	}	
			else  if (slipIsApsPkt())  {
				// Ӧ�ò����ݰ�

				// ��鷽���Ƿ�Host-Coord
				if (slipbuf[2] != SLIP_APS_HOST_TO_COORD) {
					DEBUG_STRING(DBG_INFO,"SLIP APS: Not a valid Host to Coord Packet.\n");
					slipFlushBuf(len);
					break;
				}	

				// У�����ݳ����Ƿ���ȷ
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

					// ��ʱ����Hostʵ�ֹ�����̵ķ�ʽ, Э����ֻ��͸��
					// ��������Э�����˹������+ Host�˹�������ķ�ʽ
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
				// ע���IEEE 802.15.4���ݰ�
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
		
		// �ж��Ƿ����Զ�����service ID
		
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
			// �ش�, ��������
			slipState = SLIP_STATE_IDLE;
		}
		
	break;

	case SLIP_STATE_INJECT_NWK_RX_WAIT:
		
		if (nwk_pib.flags.bits.slipRxPending)   break;
		
		if (a_slip_service.status == LOWSN_STATUS_SUCCESS)  {
			// ״̬�ɹ���������ȷ����NWK RX������������,
			// �޷�ȷ��Ӧ�ò㼰�������ܹ������յ�
			slipState = SLIP_STATE_IDLE;
		}
		else {
			// ������������״̬��ʧ�ܣ���������ʧ�ܴ���
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
			// �ش������Host������ȥ     TODO
			slipState = SLIP_STATE_IDLE;
		}	
		 	
	break;
	

	default:  break;
	} 

}



#endif
#endif


