
/******************************************************************************************************
*
* 文 件 名：halStack.c
*
* 文件描述：射频收发与定时器等函数及中断函数
*
* 创 建 者：Wang Heng
*
* 当前版本： 0.50
*
* 修 改 者：
*
* 修改历史：

  2013-05-25  22:38  by Wang Heng
   以WIA-PA CC2530为蓝本，移植到CC2530，上层功能运行成功，
   RF收发和MAC Timer功能进行了大量改进. 


********************************************************************************************************/


#include "compiler.h"
#include "hal.h"
#include "halStack.h"
#include "console.h"
#include "debug.h"
#include "ieee_lrwpan_defs.h"
#include "memalloc.h"
#include "phy.h"
#include "mac.h"



/****************************************************************************************************
* global variables
*/
RADIO_FLAGS local_radio_flags;

BYTE volatile long_address[8];

#ifdef  LOWSN_ASYNC_INTIO
BYTE serio_rxBuff[LOWSN_ASYNC_RX_BUFSIZE];
BYTE serio_rxHead, serio_rxTail;
#endif

UINT8 ReceiveFlag;


/****************************************************************************************************
* contstant
*/
#define T2CMPVAL  64

/****************************************************************************************************
* function
*/
/*********************************************************************************************
@function:

halInit contains both processor specific initialization
**********************************************************************************************/
void halInit(void)
{
  local_radio_flags.val = 0;
  SET_MAIN_CLOCK_SOURCE();
  halInitUart();
  halSetBaud(LOWSN_DEFAULT_BAUDRATE);
  halInitMACTimer();
}


void halInitUart(void) {

  IO_PER_LOC_UART0_AT_PORT0_PIN2345();
  UTX0IF = 1;

#ifdef LOWSN_ASYNC_INTIO
  serio_rxHead = 0;
  serio_rxTail = 0;
  INT_ENABLE_URX0(INT_ON);
#endif
}


void halPutch(char c)
{
  while (!UTX0IF);
  UTX0IF = 0;
  U0DBUF = c;
}

void halRawPut(char c)
{
  while (!UTX0IF);
  UTX0IF = 0;
  U0DBUF = c;
}


#ifdef  LOWSN_ASYNC_INTIO
/*********************************************************************************************
@function:

get a character from serial port, uses interrupt driven IO
**********************************************************************************************/
char halGetch(void){
  char x;
  do {
    x = serio_rxHead;  //use tmp because of volt decl
  }  while(serio_rxTail == x);
  serio_rxTail++;
  if (serio_rxTail == LOWSN_ASYNC_RX_BUFSIZE) serio_rxTail = 0;
  return (serio_rxBuff[serio_rxTail]);
}


/*********************************************************************************************
@function:

**********************************************************************************************/
BOOL halGetchRdy(void){
  char x;
  x = serio_rxHead;
  return(serio_rxTail != x);
}

#else

char halGetch (void ){
  char c;

  while (!URX0IF);
  c = U0DBUF;
  URX0IF = FALSE;

  return c;
}

/*********************************************************************************************
@function:

**********************************************************************************************/
BOOL halGetchRdy(void){

  if (URX0IF) return (1);
  else return(0);

}

#endif

/*********************************************************************************************
@function:

**********************************************************************************************/
void halUtilMemCopy(BYTE *dst, BYTE *src, BYTE len) {
  while (len) {
    *dst = *src;
    dst++;src++;
    len--;
  }
}




// assuming 16us period, have 1/16us = 62500 tics per seocnd
#define T2OVFNUM (62500/SLOWTICKS_PER_SECOND)

/*********************************************************************************************
@function:


**********************************************************************************************/
void halInitMACTimer(void)
{
       //使T2空闲，及确定其它寄存器的状态
	T2CTRL = 0x00;//T2控制寄存器
	T2IRQM = 0x00;//T2中断屏蔽

	//选  择T2M为定时周期 
	MAC_MCU_T2_ACCESS_PERIOD_VALUE();

       //定时周期320 * 31.25ns = 10000 ns = 10us
	//T2M0 = 0x40;
	//T2M1 = 0x01;

	// 总是需先读或写T2M0，然后再T2M1
	T2M0 = 0x00;  // (0x0200) / 32 = 16 u-seconds
	T2M1 = 0x02;  // setting for 16 u-second periods


	#ifdef LOWSN_ENABLE_SLOW_TIMER
	
	// 屏蔽TIMER2_PERIM中断
	T2IRQM &= ~TIMER2_PERM;
	
       //选择T2MOVF溢出中断
	MAC_MCU_T2_ACCESS_OVF_CMP1_VALUE();

	T2MOVF0 = (BYTE) (T2OVFNUM);
	T2MOVF1 = (BYTE) (T2OVFNUM>>8);
	T2MOVF2 = (BYTE) (T2OVFNUM>>16);

	//T2MOVF 比较中断使能
	T2IRQM |= TIMER2_OVF_COMPARE1M;
	   
	 #endif

       //设置LATCH_MODE
      // T2CTRL |= LATCH_MODE;
	T2CTRL &= ~LATCH_MODE;

	//启动T2，开始计时
	MAC_RADIO_TIMER_WAKE_UP();

	#ifdef LOWSN_ENABLE_SLOW_TIMER
       //清除中断标志位
	INT_SETFLAG_T2(INT_CLR);

       //打开T2中断使能
       INT_ENABLE_T2(INT_ON);

	 INT_GLOBAL_ENABLE(INT_ON);

	#endif

}


/*********************************************************************************************
@function:

**********************************************************************************************/
UINT32 halGetMACTimer(void)
{
  //UINT32 overflowCount;
  UINT32 t;
  BOOL gie_status;

  SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);

 /* This T2 access macro allows accessing both T2MOVFx and T2Mx */
  MAC_MCU_T2_ACCESS_OVF_COUNT_VALUE();

  //读T2M0锁定定时器t2tim和溢出计数器t2ovf的值，然后读数值

  t = 0x0FF & T2MOVF0;
  t += (((UINT16)T2MOVF1)<<8);
  t += (((UINT32)T2MOVF2)<<16);

 // ((UINT8 *)&overflowCount)[UINT32_NDX0] = T2MOVF0;
 // ((UINT8 *)&overflowCount)[UINT32_NDX1] = T2MOVF1;
  //((UINT8 *)&overflowCount)[UINT32_NDX2] = T2MOVF2;
 // ((UINT8 *)&overflowCount)[UINT32_NDX3] = 0;

  RESTORE_GLOBAL_INTERRUPT(gie_status);
  //return (overflowCount);
  
  return (t);

  
}



/**************************************************************************************************
 * @fn          macMcuOverflowCapture
 *
 * @brief       Returns the last capture of the overflow counter.  A special hardware feature
 *              captures the overflow counter when the regular hardware timer is captured.
 *
 * @param       none
 *
 * @return      last capture of overflow count
 **************************************************************************************************
 */
UINT32 macMcuOverflowCapture(void)
{
  UINT32  overflowCapture;
  halIntState_t  s;

  /* for efficiency, the 32-bit value is encoded using endian abstracted indexing */
  HAL_ENTER_CRITICAL_SECTION(s);
  MAC_MCU_T2_ACCESS_OVF_CAPTURE_VALUE();
  ((UINT8 *)&overflowCapture)[UINT32_NDX0] = T2MOVF0;
  ((UINT8 *)&overflowCapture)[UINT32_NDX1] = T2MOVF1;
  ((UINT8 *)&overflowCapture)[UINT32_NDX2] = T2MOVF2;
  ((UINT8 *)&overflowCapture)[UINT32_NDX3] = 0;
  HAL_EXIT_CRITICAL_SECTION(s);

  return (overflowCapture);
}

/**************************************************************************************************
 * @fn          halOverflowSetCount
 *
 * @brief       Sets the value of the hardware overflow counter.
 *
 * @param       count - new overflow count value
 *
 * @return      none
 ***************************************************************************************************/
 void halOverflowSetCount(UINT32 count)//ts
{
	  BOOL gie_status;

	  SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
	
 	 MAC_MCU_T2_ACCESS_OVF_COUNT_VALUE();

	  /* for efficiency, the 32-bit value is decoded using endian abstracted indexing */
	  /* T2OF2 must be written last */


	   T2MOVF0 = (UINT8) count;
	   T2MOVF1 = (UINT8)(count>>8);
	   T2MOVF2 = (UINT8)(count>>16);
	   RESTORE_GLOBAL_INTERRUPT(gie_status);
}



/*********************************************************************************************
@function:

**********************************************************************************************/
#ifdef LOWSN_COMPILER_NO_RECURSION
UINT32 halISRGetMACTimer(void){
  UINT32 t;
  BOOL gie_status;

  SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
  t = 0x0FF & T2MOVF0;
  t += (((UINT16)T2MOVF1)<<8);
  t += (((UINT32) T2MOVF2 & 0x0F)<<16);
  RESTORE_GLOBAL_INTERRUPT(gie_status);
  return (t);
}

#endif



/*********************************************************************************************
@function:

only works as long as SYMBOLS_PER_MAC_TICK is not less than 1
**********************************************************************************************/
UINT32 halMacTicksToUs(UINT32 ticks){

  UINT32 rval;

  rval =  (ticks/SYMBOLS_PER_MAC_TICK())* (1000000/LOWSN_SYMBOLS_PER_SECOND);
  return(rval);
}


/*********************************************************************************************
@function:

assumes that Timer2 has been initialized and is running
**********************************************************************************************/
UINT8 halGetRandomByte(void) {
  return(T2MOVF0);
}




/*********************************************************************************************
@function:

set the radio frequency
**********************************************************************************************/
LOWSN_STATUS_ENUM halSetRadioIEEEFrequency(PHY_FREQ_ENUM frequency, BYTE channel)
{
  //UINT16 afreq;
  BOOL gie_status;

  if (frequency != PHY_FREQ_2405M) return(LOWSN_STATUS_PHY_FAILED);
  if ((channel < 11) || (channel > 28)) return(LOWSN_STATUS_PHY_FAILED);  //CC2530 supports channel 27 and 28.

  // enter critical code.

  SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);

  // close RX state. Make sure it is not on the tranmiting state.

  // Rx off
  MAC_RADIO_RXTX_OFF();
  //just in case a receive was about to start, flush the receive FIFO

  MAC_RADIO_FLUSH_RX_FIFO();
  //clear any receive interrupt that happened to squeak through
  MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG();

  // set Frequency register
  // FSCTRLH has no sense, because it always be 01.
  //MAC_RADIO_SET_CHANNEL(channel);
  FREQCTRL = FREQ_2405MHZ + 5 * ((channel) - 11);
  //afreq = 357 + 5*(channel - 11);
  //FSCTRLL = (BYTE) afreq;
  //FSCTRLH = ((FSCTRLH & ~0x03) | (BYTE)((afreq >> 8) & 0x03));

  // RX on
  MAC_RADIO_RX_ON();

  RESTORE_GLOBAL_INTERRUPT(gie_status);


  return(LOWSN_STATUS_SUCCESS);
}

LOWSN_STATUS_ENUM halSetChannel(BYTE channel)
{
 	return(halSetRadioIEEEFrequency(PHY_FREQ_2405M, channel));
}


#if 0
/*********************************************************************************************
@function:

this assumes 2.4GHz frequency
**********************************************************************************************/
#if defined(IAR8051)
//this uses the IEEE address stored in program memory.
//ensure that the flash programmer is configured to retain
//the IEEE address
__near_func void halGetProcessorIEEEAddress(BYTE *buf)
{

  unsigned char memCtr;
  unsigned char bank;

  memCtr = MEMCTR;
  bank = FMAP;

 //switch to bank 7
 MEMCTR |=0x0F;
 FMAP |= 0x07;
  //note that the flash programmer stores these in BIG ENDIAN order for some reason!!!
  buf[0] = aExtendedAddress_B0;
  buf[1] = aExtendedAddress_B1;
  buf[2] = aExtendedAddress_B2;
  buf[3] = aExtendedAddress_B3;
  buf[4] = aExtendedAddress_B4;
  buf[5] = aExtendedAddress_B5;
  buf[6] = aExtendedAddress_B6;
  buf[7] = aExtendedAddress_B7;
/*
  buf[7] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+0);
  buf[6] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+1);
  buf[5] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+2);
  buf[4] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+3);
  buf[3] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+4);
  buf[2] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+5);
  buf[1] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+6);
  buf[0] = *(ROMCHAR *)(IEEE_ADDRESS_ARRAY+7);
*/

  //resore old bank settings
  MEMCTR = memCtr;
  FMAP = bank;


}

#endif

#endif




// 为了调试方便，使用自己设定的EUI64地址
// 务必注意: CC2430存储EUI64地址是采用小端格式的, 而协议栈在设定
// 时都是按照大端格式( 即阅读顺序处理的)
// 这个以后需要专门理顺，否则可能与其他协议栈会有互通问题.



__near_func void halGetProcessorIEEEAddress(BYTE *buf) 
{
	buf[0] = aExtendedAddress_B0;
	buf[1] = aExtendedAddress_B1;
	buf[2] = aExtendedAddress_B2;
	buf[3] = aExtendedAddress_B3;
	buf[4] = aExtendedAddress_B4;
	buf[5] = aExtendedAddress_B5;
	buf[6] = aExtendedAddress_B6;
	buf[7] = aExtendedAddress_B7;

}


// 暂时为了MAC层长地址按反阅读顺序发送而进行设定，以后需要统一处理长地址
__near_func void halGetProcessorIEEEAddress_MAC(BYTE *buf) 
{

	buf[0] = aExtendedAddress_B7;
	buf[1] = aExtendedAddress_B6;
	buf[2] = aExtendedAddress_B5;
	buf[3] = aExtendedAddress_B4;
	buf[4] = aExtendedAddress_B3;
	buf[5] = aExtendedAddress_B2;
	buf[6] = aExtendedAddress_B1;
	buf[7] = aExtendedAddress_B0;
}



// 在编译器参数中配置的aExtendedAddress_B0-B7是按大端格式存储的(自己设定的)
// 这里存储到芯片中时需要反转
// 芯片中存储的是小端格式

void halSetRadioIEEEAddress(void)
{
  BYTE buf[8];
  halGetProcessorIEEEAddress_MAC(buf);
	  EXT_ADDR0 = buf[0];
	  EXT_ADDR1 = buf[1];
	  EXT_ADDR2 = buf[2];
	  EXT_ADDR3 = buf[3];
	  EXT_ADDR4 = buf[4];
	  EXT_ADDR5 = buf[5];
	  EXT_ADDR6 = buf[6];
	  EXT_ADDR7 = buf[7];
}

/*********************************************************************************************
 * @fn          halSetRadioPANID
 *
 * @brief       Set the pan ID on the radio.
 *
 * @param       panID - 16 bit PAN identifier
 *
 * @return      none

**********************************************************************************************/
void halSetRadioPANID(UINT16 panid)
{
    MAC_RADIO_SET_PAN_ID(panid);
}

/*********************************************************************************************
 * @fn          halSetRadioShortAddr
 *
 * @brief       Set the short addrss on the radio.
 *
 * @param       shortAddr - 16 bit short address
 *
 * @return      none

**********************************************************************************************/
void halSetRadioShortAddr(SADDR saddr)
{
   MAC_RADIO_SET_SHORT_ADDR(saddr);
}


/*********************************************************************************************
@function:

**********************************************************************************************/
LOWSN_STATUS_ENUM halInitRadio(PHY_FREQ_ENUM frequency, BYTE channel, RADIO_FLAGS radio_flags)
{
  LOWSN_STATUS_ENUM status;


  // Setting the frequency
  status = halSetRadioIEEEFrequency(frequency, channel);
  if (status != LOWSN_STATUS_SUCCESS)  return(status);


  		  /* This CORR_THR value should be changed to 0x14 before attempting RX. Testing has shown that
	   * too many false frames are received if the reset value is used. Make it more likely to detect
	   * sync by removing the requirement that both symbols in the SFD must have a correlation value
	   * above the correlation threshold, and make sync word detection less likely by raising the
	   * correlation threshold.
	   */
	  MDMCTRL1 = CORR_THR;

	  /* tuning adjustments for optimal radio performance; details available in datasheet */
	  RXCTRL = 0x3F;

	  /* Raises the CCA threshold from about -108dBm to about -80 dBm input level.
	   */
	 CCACTRL0 = CCA_THR;

	  /* Makes sync word detection less likely by requiring two zero symbols before the sync word.
	   * details available in datasheet.
	   */
	  MDMCTRL0 = 0x85;//控制调制调解器

	  /* Adjust current in synthesizer; details available in datasheet. */
	  FSCTRL = 0x5A;//调整频率合成器 全部复位

	  /* Adjust current in VCO; details available in datasheet. */
	  FSCAL1 = 0x00;//调整频率校准

	  /* Adjust target value for AGC control loop; details available in datasheet. */
	 AGCCTRL1 = 0x15;//AGC参考水平

	  /* Disable source address matching an autopend for now */
	  SRCMATCH = 0;//源地址匹配和未决位
/*
	  if (radio_flags.bits.pan_coordinator) {
	    FRMFILT0 |= PAN_COORDINATOR;  //accepts frames with only source addressing modes
	  } else {
	    FRMFILT0 &= ~PAN_COORDINATOR;  //rejects frames with only source addressing modes
	  }
*/
	  /* Sets TX anti-aliasing filter to appropriate bandwidth.
	   * Reduces spurious emissions close to signal.
	   */
	  TXFILTCFG = TXFILTCFG_RESET_VALUE;

	  /* disable the CSPT register compare function */
	 // CSPT = 0xFF;



	  /* set RF interrupts one notch above lowest priority (four levels available) */
	 // IP0 |=  IP_RFERR_RF_DMA_BV;
	 // IP1 &= ~IP_RFERR_RF_DMA_BV;

	//FRMCTRL0_RESET_VALUE: 初始值已经是自动CRC，TX和RX正常模式, 
	// APPEND_DATA_MODE模式为0, AUTO-ACK没有开启

	if (radio_flags.bits.listen_mode) {
    		//corresponds to promiscuous modes
    		//radio accepts all packets, the HUSSY!
		FRMFILT0 &= ~FRAME_FILTER_EN;
			
		FRMCTRL0 = FRMCTRL0_RESET_VALUEL &  (~AUTO_ACK);       //no auto ack
  	} else {
    		// Turning on Address Decoding
    		FRMFILT0 |=  FRAME_FILTER_EN;
   		 
		FRMCTRL0 = FRMCTRL0_RESET_VALUE | AUTOACK;  //enable auto_ack
  	}
	
  	local_radio_flags = radio_flags;  //save this for later

  	//pan
  	if (radio_flags.bits.pan_coordinator) {
   		 FRMFILT0 |= PAN_COORDINATOR;  //accepts frames with only source addressing modes
 	} else {
    		FRMFILT0 &= ~PAN_COORDINATOR;  //rejects frames with only source addressing modes
  	}

	//#ifdef LOWSN_NO_CHECK_PHYMAC
	//FRMFILT0 &= ~FRAME_FILTER_EN;
	//FRMCTRL0 = FRMCTRL0_RESET_VALUE | AUTOACK;
	//#endif
	

	// 所有IEEE 802.15.4定义的帧都接收
	FRMFILT1 = 0x78;
	  /* Initialize SRCEXTPENDEN and SRCSHORTPENDEN to zeros */
	MAC_RADIO_SRC_MATCH_INIT_EXTPENDEN();//扩展地址使能或者禁止未决
	MAC_RADIO_SRC_MATCH_INIT_SHORTPENDEN();//源短地址使能或禁止未决


	FRMCTRL1 |= SET_RXENMASK_ON_TX;
	//set to max value as the FIFOP flag goes high when complete packet received
	MAC_RADIO_SET_RX_THRESHOLD(128);//缓存阈值不能超过127

	MAC_RADIO_RX_ON();//RX使能并校准频率合成器
	//RFST = SRXON;
	MAC_RADIO_FLUSH_TX_FIFO();//立即清除TXFIFO缓冲区
	MAC_RADIO_FLUSH_RX_FIFO();//立即清除RXFIFO缓冲区
	MAC_RADIO_TX_ON();//校准之后使能TX
	CSP_START_PROGRAM();//从写到指令寄存器的第一条指令开始执行CSP程序

	//this controls the frame pending bit in ACK frames.
	//because of auto-ack, we will not have time to determine if data
	//is actually pending or not.
	#if defined(LOWSN_RFD)
  		 MAC_RADIO_SACK();  //RFDs never have data pending for FFDs
	#else
 		 MAC_RADIO_SACKPEND();  //routers/
	#endif

	halSetRadioIEEEAddress();//通过软件写入扩展地址

	/* clear any accidental threshold interrupt that happened as part of power up sequence */
	MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG();

	/* enable threshold interrupts */
	MAC_RADIO_ENABLE_RX_THRESHOLD_INTERRUPT();

	//MAC_MCU_SFD_ENABLE_INTERRUPT();

		  /* enable general RF interrupts */
	  IEN2 |= RFIE;

	  /* enable general RFERR interrupts */
	  IEN0 |= RFERRIE;


	return(LOWSN_STATUS_SUCCESS);

}


/*********************************************************************************************
@function:

regardless of what happens here, we will try TXONCCA after this returns.
**********************************************************************************************/
void  doIEEE_backoff(void)
{
  BYTE be, nb, tmp, rannum;
  UINT32  delay, start_tick;

  be = aMinBE;
  nb = 0;
  do {
    if (be) {
      //do random delay
      tmp = be;
      //compute new delay
      delay = 1;
      while (tmp) {
        delay = delay << 1;  //delay = 2**be;
        tmp--;
      }
      rannum =  halGetRandomByte() & (delay-1); //rannum will be between 0 and delay-1
      delay = 0;
      while (rannum) {
        delay  += SYMBOLS_TO_MACTICKS(aUnitBackoffPeriod);
        rannum--;
      }//delay = aUnitBackoff * rannum
      //now do backoff
      start_tick = halGetMACTimer();
      while (halMACTimerNowDelta(start_tick) < delay);
    }
    //check CCA
    if (MAC_RADIO_CCA_INDICATION())  break;
    nb++;
    be++;
    if (be > aMaxBE) be =aMaxBE;
  }while (nb <= macMaxCSMABackoffs);
  return;
}



/*********************************************************************************************
@function:

transmit packet
hdrlen - header lenth
hdr - pointer to header data
plen - payload length
pload - pointer to payload
**********************************************************************************************/
LOWSN_STATUS_ENUM halSendPacket(BYTE flen, BYTE *frm)
{
  BYTE len;
  LOWSN_STATUS_ENUM res;

  len = flen + PACKET_FOOTER_SIZE;

  DEBUG_STRING(DBG_INFO, "TX PKT Size: ");
  DEBUG_UINT8(DBG_INFO,len);
  DEBUG_STRING(DBG_INFO,"\n");

    if (len > 127) {
    //packet size is too large!
    return(LOWSN_STATUS_PHY_TX_PKT_TOO_BIG);
  }

    // Turn on receiver if its not on
       MAC_RADIO_FLUSH_RX_FIFO();
       MAC_RADIO_RX_ON();

    // Wait until the transceiver is idle   判断TXACTIVE 或可能的SFD中断
    // 发送SFD中断作为条件似乎有问题，但TXACTVIE作为条件没问题.
    // 应尽量不在状态机中用while死循环. 将来改进这个
    while (FSMSTAT1 & (BV(1) | BV(5) ));

    // Turn off RX frame done interrupt to avoid interference on the SPI interface

    RFIRQF1 = ~IRQ_TXDONE;   // Clear the RF TXDONE flag
    INT_SETFLAG_RF(INT_CLR);  //Clear processor interrupt flag
    RFIRQM0 &= ~BV(6);    // disable RXPKTDONE interrupt
   // IEN2 &= ~BV(0);   // disable general RF interrupts

    MAC_RADIO_FLUSH_TX_FIFO();         // Making sure that the TX FIFO is empty.

    RFD = len;
    while (flen) {RFD = *frm; frm++; flen--;}


     // If the RSSI value is not valid, enable receiver
    if(RSSI == 0x80)
   {
       MAC_RADIO_RX_ON();
      // Turning on Rx and waiting 320u-sec to make the RSSI value become valid.
      halWait(1);
   }
	
    doIEEE_backoff();

    MAC_RADIO_TX_ON_CCA();
    //MAC_RADIO_TX_ON() ;

    if(FSMSTAT0 > 30)  //is TX active?
    {
    // Asserting the status flag and enabling ACK reception if expected.
    phyTxStartCallBack();
    res = LOWSN_STATUS_SUCCESS;
	
    RFIRQM0 |= IRQ_RXPKTDONE; // enable RXPKTDONE interrupt
    RFIRQM1 |= IRQ_TXDONE;   //enable IRQ_TXDONE interrupt
    //IEN2 |= BV(0);   // enable general RF interrupts
    
    DEBUG_CHAR( DBG_TX,DBG_CHAR_TXSTART);
  }
  else
  {
    MAC_RADIO_FLUSH_TX_FIFO();           //empty buffer
    res = LOWSN_STATUS_PHY_CHANNEL_BUSY;
    RFIRQM1 &= ~IRQ_TXDONE;   //enable IRQ_TXDONE interrupt
    DEBUG_CHAR( DBG_TX,DBG_CHAR_TXBUSY);
  }
  
  return(res);

}



/*=================================================================================================
 * @fn          HAL_ISR_FUNCTION
 *
 * @brief       UART0 Receive Interrupt
 *
 * @param       none
 *
 * @return      none
 *		
 *=================================================================================================*/
#ifdef  LOWSN_ASYNC_INTIO

HAL_ISR_FUNCTION( hal_Uart0_RxIsr, URX0_VECTOR)//ts
{
  DISABLE_GLOBAL_INTERRUPT();
  BYTE x,y;
  serio_rxHead++;
  if (serio_rxHead == LOWSN_ASYNC_RX_BUFSIZE )
  	{
  	serio_rxHead = 0;
	}
  x = serio_rxHead;  //use tmp variables because of Volatile decl
  y = U0DBUF;
  serio_rxBuff[x] = y;
  ReceiveFlag = 1;
  ENABLE_GLOBAL_INTERRUPT();
}

#endif


/*********************************************************************************************
@function:

**********************************************************************************************/

#ifdef LOWSN_ENABLE_SLOW_TIMER

HAL_ISR_FUNCTION( hal_Timer2_Isr, T2_VECTOR)//ts
{

	UINT8 ActiveInterrupt;
	UINT32 t;
   
	ActiveInterrupt = T2IRQF;
   	ActiveInterrupt &= T2IRQM;

   	T2IRQF = 0;

  	INT_GLOBAL_ENABLE(INT_OFF);
  	INT_SETFLAG_T2(INT_CLR); //clear processor interrupt flag


  	if (ActiveInterrupt & TIMER2_OVF_COMPARE1F)  {

   		//compute next compare value by reading current timer value, adding offset
     		MAC_MCU_T2_ACCESS_OVF_COUNT_VALUE();
  		t = 0x0FF & T2MOVF0;
  		t += (((UINT16)T2MOVF1)<<8);
  		t += (((UINT32)T2MOVF2)<<16);

		t += T2OVFNUM;  //add offset
		
   		MAC_MCU_T2_ACCESS_OVF_CMP1_VALUE();
		T2MOVF0 = (BYTE) (t);
		T2MOVF1 = (BYTE) (t>>8);
		T2MOVF2 = (BYTE) (t>>16);

		MAC_RADIO_TIMER_WAKE_UP();

		evbIntCallback();  //Evaluation board callback
		usrSlowTimerInt();  //user level interrupt callback
	
	}
	
	INT_GLOBAL_ENABLE(INT_ON);

}

#endif

//interrupt for RF error
//this interrupt is same priority as FIFOP interrupt,
//but is polled first, so will occur first.
/**************************************************************************************************
 * @fn          macMcuRfErrIsr
 *
 * @brief       Interrupt service routine that handles all RF Error interrupts.  Only the RX FIFO
 *              overflow condition is handled.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************/

HAL_ISR_FUNCTION( hal_rferr_Isr, RFERR_VECTOR )//ts
{
  INT_GLOBAL_ENABLE(INT_OFF);

  // If Rx overflow occurs, the Rx FiFo is reset.
  // The Rx DMA is reset and reception is started over.
  if(FSMSTAT0 == 17)//RX溢出
  {
    DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_TXBUSY);
    MAC_RADIO_RXTX_OFF();
    MAC_RADIO_FLUSH_RX_FIFO();//ISFLUSHRX;
    //ISFLUSHRX;
    MAC_RADIO_RX_ON();//ISRXON;
  }
  else if(FSMSTAT0 == 56)//TX下溢
  {
    DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_RXOFLOW);
    MAC_RADIO_FLUSH_TX_FIFO();//ISFLUSHTX;
  }

  INT_SETFLAG_RFERR(INT_CLR);

  INT_GLOBAL_ENABLE(INT_ON);
}


//This interrupt used for both TX and RX
/*********************************************************************************************
@function:

**********************************************************************************************/

HAL_ISR_FUNCTION( hal_rf_Isr, RF_VECTOR )//ts
{

  BYTE flen;
  // CC2530对RF中断指示位由CC2430中的一个寄存器扩充到2个寄存器，
  BYTE enabledAndActiveInterrupt0, enabledAndActiveInterrupt1;
  BYTE *ptr, *rx_frame;
  BYTE crc;
  BYTE ack_bytes[5];
    //define alternate names for readability in this function
#define  fcflsb ack_bytes[0]
#define  fcfmsb  ack_bytes[1]
#define  dstmode ack_bytes[2]
#define  srcmode ack_bytes[3]

  INT_GLOBAL_ENABLE(INT_OFF);
  enabledAndActiveInterrupt0 = RFIRQF0;
  enabledAndActiveInterrupt1 = RFIRQF1;
  RFIRQF0 = 0x00;   // Clear all radio interrupt flags
  RFIRQF1 = 0x00;
  INT_SETFLAG_RF(INT_CLR);    // Clear MCU interrupt flag

  enabledAndActiveInterrupt0 &= RFIRQM0;
  enabledAndActiveInterrupt1 &= RFIRQM1;

	if(enabledAndActiveInterrupt0 & IRQ_FIFOP)  {
         DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_RXRCV );
	  ptr = NULL; //temporary pointer
         flen = RFD & 0x7f;  //read the length
         if (flen == LOWSN_ACKFRAME_LENGTH)  {
 	     	 DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_ACKPKT );
 		 ack_bytes[0]= flen;
               ack_bytes[1] =  RFD;  //LSB Frame Control Field
               ack_bytes[2] = RFD;   //MSB Frame Control Field
               ack_bytes[3] = RFD;   //dsn
               ack_bytes[4] = RFD;   //RSSI
               crc = RFD;	
	     if (crc & 0x80)  {
	     	  macRxCallback(ack_bytes, ack_bytes[4]);
	     }
     	  }
		 
	else   {
	//not an ack packet, lets do some more early rejection

		     	   fcflsb = RFD;
                        fcfmsb = RFD;
                        if (!local_radio_flags.bits.listen_mode)  {
                        	   srcmode = LOWSN_GET_SRC_ADDR(fcfmsb);
                               dstmode = LOWSN_GET_DST_ADDR(fcfmsb);
				   if ((srcmode == LOWSN_ADDRMODE_NOADDR) && (dstmode == LOWSN_ADDRMODE_NOADDR))  {
				         //reject this packet, no addressing info
                                     goto do_rxflush;	
                        	     }
		     	     }
         	
		           if (!macRxBuffFull())   {
			        rx_frame = MemAlloc(flen+1);	//开辟空间 放收到的数据
                             ptr = rx_frame;
			    }
		          else  {
			        //MAC RX buffer is full
                            DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_MACFULL );
			    }

		 if (ptr == NULL)  {
		 	  goto do_rxflush;
		 }
		 else   {
		 	       //save packet, including the length
                           *ptr = flen; ptr++;
                            //save the fcflsb, fcfmsb bytes
                           *ptr = fcflsb; ptr++; flen--;
                           *ptr = fcfmsb; ptr++; flen--;
                          //get the rest of the bytes
                          while (flen) { *ptr = RFD;  flen--; ptr++; }
                           //do RX callback
                           //check the CRC
                           if (*(ptr-1) & 0x80)   {
                              //CRC good
                              //change the RSSI byte from 2's complement to unsigned number
                               *(ptr-2) = *(ptr-2) + 0x80;

			           phyRxCallback();
		                  macRxCallback(rx_frame, *(ptr-2));
		 	        }
			        else  {
			        	  // CRC bad. Free the packet
                                     MemFree(rx_frame);
			        }
		      }
	 }

		  //flush any remaining bytes
do_rxflush:

		MAC_RADIO_FLUSH_RX_FIFO();
		MAC_RADIO_FLUSH_RX_FIFO();

		//don't know why, but the RF flags have to be cleared AFTER a read is done.
		RFIRQF0 = 0x00;   // Clear all radio interrupt flags
		RFIRQF1 = 0x00;
		INT_SETFLAG_RF(INT_CLR);    // Clear MCU interrupt flag
		
		MAC_MCU_FIFOP_ENABLE_INTERRUPT();

     }

  if (enabledAndActiveInterrupt1 & IRQ_TXDONE)  {
     //Finished TX, do call back
    DEBUG_CHAR( DBG_ITRACE,DBG_CHAR_TXFIN );
    phyTxEndCallBack();
    macTxCallback();
    // Clearing the tx done interrupt enable
    MAC_MCU_TXDONE_DISABLE_INTERRUPT();//RFIRQM1 &= ~IRQ_TXDONE;
   }

    usrIntCallback();
    INT_GLOBAL_ENABLE(INT_ON);

#undef  fcflsb
#undef  fcfmsb
#undef  dstmode
#undef  srcmode
}


/*********************************************************************************************
@function:

software delay, waits is in milliseconds
**********************************************************************************************/
void halWait(BYTE wait){
  UINT32 largeWait;

  if(wait == 0)
  {return;}
  largeWait = ((UINT16) (wait << 7));
  largeWait += 114*wait;


  largeWait = (largeWait >> CLKSPD);
  while(largeWait--);

  return;
}


/*********************************************************************************************
@function:

**********************************************************************************************/
void halWaitMs(const UINT32 msecs){
  UINT32 towait;

  towait = msecs;
  while (towait > 100)
  {
    halWait(100);
    towait -= 100;
  }
  halWait(towait);
}

/*********************************************************************************************
@function:

**********************************************************************************************/






/* for revision E, this sw workaround require additional code in all
* ISR's that are used to wake up from PM.
*/
#define HAL_SLEEP_SET_POWER_MODE(mode)  do{ SLEEPCMD &= ~0x03;  /* clear mode bits */               \
  SLEEPCMD |= mode;   /* set mode bits   */               \
    asm("NOP");                                          \
      asm("NOP");                                          \
        asm("NOP");                                          \
          if( SLEEPCMD & 0x03 )                                   \
            {                                                    \
              PCON |= 0x01;  /* enable mode */                   \
                asm("NOP");    /* first instruction after sleep*/  \
                                                 }; } while(0)



/*********************************************************************************************
@function:
输入参数:  SleepMode
#define POWER_MODE_0  0x00  // Clock oscillators on, voltage regulator on
#define POWER_MODE_1  0x01  // 32.768 KHz oscillator on, voltage regulator on
#define POWER_MODE_2  0x02  // 32.768 KHz oscillator on, voltage regulator off
#define POWER_MODE_3  0x03  // All clock oscillators off, voltage regulator off


**********************************************************************************************/
__near_func  void halSleep(UINT8 SleepMode, UINT32 msecs )
{
  UINT32 t;
  UINT32 delta;
  BOOL gie_status;

  SAVE_AND_DISABLE_GLOBAL_INTERRUPT(gie_status);
  //read the sleep timer
  delta = (32768 * msecs)/1000;
  t = 0xFF & ST0;
  t += (((UINT16)ST1)<<8);
  t += (((UINT32) ST2 & 0xFF)<<16);

  //compute the compare value  and subtract the processing time spent in function halSleep()
  t = (t + delta - HAL_SLEEP_ADJ_TICKS)&0x00FFFFFF;

  //write the new sleep timer value
  ST2 = (t >> 16)&0xFF;
  ST1 = (t >> 8)&0xFF;
  ST0 = t & 0xFF;
  //clear the sleep flag, enable the interrupt
  IRCON = IRCON & 0x7F; //clear the sleep flag IRCON.STIF = 0;
  IEN0 = IEN0 | (1<<5); //enable the interrupt  IEN0.STIE = 1;

  ENABLE_GLOBAL_INTERRUPT();  //interrupts must be enabled to wakeup!
  //configure the power mode and sleep
  //SET_POWER_MODE(POWER_MODE_2);
  //SET_POWER_MODE(POWER_MODE_1);

  // set CC2430 power mode
  HAL_SLEEP_SET_POWER_MODE(SleepMode);


  //wake up!
  //disable sleep interrupt
  DISABLE_GLOBAL_INTERRUPT();
  IEN0 = IEN0 & ~(1<<5);  // IEN0.STIE = 0;

  //wait for everything to power back up
  while (!(SLEEPSTA & XOSC_STB));         \
    asm("NOP");
    RESTORE_GLOBAL_INTERRUPT(gie_status);
};



/*********************************************************************************************
@function:

**********************************************************************************************/
INT16 halGetAdcValue(){
  INT16 value;
  value = ((INT16)ADCH) << 8;
  value |= (INT16)ADCL;
  return value;
}


// 获取电池电压

UINT16 getVoltageValue(void)
{

  UINT16 value;

  ADCIF = 0;

  ADCCON3 = (HAL_ADC_REF_125V | HAL_ADC_DEC_128 | HAL_ADC_CHN_VDD3);

  while ( !ADCIF );

  value = ADCL;
  value |= ((UINT16) ADCH) << 8;

  
  value = value >> 6;   
  value = (UINT16)(value * 37.5);
  value = value >> 9;  

  // 经处理后，返回的值为VDD实际电压值乘以10

  return value;

}




/*********************************************************************************************
@function:

**********************************************************************************************/
HAL_ISR_FUNCTION( hal_St_Isr, ST_VECTOR)//ts
{
  DRV_SLEEP_TIMER_CLEAR_INT( );//睡眠定时器无中断未决
  DRV_CLEAR_SLEEP_MODE();//供电模式为0模式
}


/*********************************************************************************************
@function:

//functions used by EVboard.

//-----------------------------------------------------------------------------
// See hal.h for a description of this function.
//-----------------------------------------------------------------------------
**********************************************************************************************/
INT16 halAdcSampleSingle(BYTE reference, BYTE resolution, UINT8 input) //不需要改动ADCCFG数据手册上没有但是TI上有
{
  BYTE volatile temp;
  INT16 value;

  //reading out any old conversion value
  temp = ADCH;
  temp = ADCL;


  ADC_ENABLE_CHANNEL(input);
  ADC_STOP();

  ADC_SINGLE_CONVERSION(reference | resolution | input);

  while (!ADC_SAMPLE_READY());

  ADC_DISABLE_CHANNEL(input);

  value = (((INT16)ADCH) << 8);
  value |= ADCL;

  resolution >>= 3;
  return value >> (8 - resolution);
}


/***********************************************************************************
* @fn      halRfTransmit
*
* @brief   Transmit frame with Clear Channel Assessment.
*
* @param   none
*
* @return  uint8 - SUCCESS or FAILED
*/
UINT8 halRfTransmit(void)
{
    UINT8 status;

    MAC_RADIO_TX_ON() ;

    // Waiting for transmission to finish
    ///while(!(RFIRQF1 & IRQ_TXDONE) );

    RFIRQF1 = ~IRQ_TXDONE;
    status= 0;

    return status;
}

BOOL HalUartBufferEmpty(void)
{
 	 return (serio_rxHead == serio_rxTail);
}



void halShutdown(void) {

// 等待添加

}

void halWarmstart(void) {

// 等待添加
  
 }



