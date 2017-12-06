
#ifndef HALSTACK_H
#define HALSTACK_H

/****************************************************************************************************
* includes
*/
#include "compiler.h"
#include "hal.h"
#include "stm32f10x.h"



// assuming 16us period, have 1/16us = 62500 tics per seocnd
#define T2CMPVAL (62500/SLOWTICKS_PER_SECOND)
//In stm32, max slow timer peroid is: 65536*16us=1.04s. CCR1_VAL can not exceed 0xFFFF
#define CCR1_VAL (62500/SLOWTICKS_PER_SECOND)   

#define SMARTRF_SETTING_FSCTRL1    0x12
#define SMARTRF_SETTING_FSCTRL0    0x00
#define SMARTRF_SETTING_FREQ2      0x10
#define SMARTRF_SETTING_FREQ1      0xB1
#define SMARTRF_SETTING_FREQ0      0x3B
#define SMARTRF_SETTING_MDMCFG4    0x2D
#define SMARTRF_SETTING_MDMCFG3    0x3B
#define SMARTRF_SETTING_MDMCFG2    0x13
#define SMARTRF_SETTING_MDMCFG1    0x22
#define SMARTRF_SETTING_MDMCFG0    0xF8
#define SMARTRF_SETTING_CHANNR     0x14
#define SMARTRF_SETTING_DEVIATN    0x47
#define SMARTRF_SETTING_FREND1     0xB6
#define SMARTRF_SETTING_FREND0     0x10
#define SMARTRF_SETTING_FOCCFG     0x1D
#define SMARTRF_SETTING_BSCFG      0x1C
#define SMARTRF_SETTING_AGCCTRL2   0xC7
#define SMARTRF_SETTING_AGCCTRL1   0x00
#define SMARTRF_SETTING_AGCCTRL0   0xB0
#define SMARTRF_SETTING_MCSM0      0x18
#define SMARTRF_SETTING_FSCAL3     0xEA
#define SMARTRF_SETTING_FSCAL2     0x2A
#define SMARTRF_SETTING_FSCAL1     0x00
#define SMARTRF_SETTING_FSCAL0     0x1F
#define SMARTRF_SETTING_FSTEST     0x59
#define SMARTRF_SETTING_TEST2      0x88
#define SMARTRF_SETTING_TEST1      0x31
#define SMARTRF_SETTING_TEST0      0x09
#define SMARTRF_SETTING_FIFOTHR    0x07
#define SMARTRF_SETTING_IOCFG2     0x29
#define SMARTRF_SETTING_IOCFG0D    0x06
#define SMARTRF_SETTING_ADDR       0x00


#define MRFI_CCA_RETRIES        4

#define MRFI_NUM_LOGICAL_CHANS           4  //pu 
#define MRFI_NUM_POWER_SETTINGS          3
#define MRFI_LENGTH_FIELD_SIZE              1
#define MRFI_BACKOFF_PERIOD_USECS           250


/* Max time we can be in a critical section within the delay function.
 * This could be fine-tuned by observing the overhead is calling the bsp delay
 * function. The overhead should be very small compared to this value.
 * Note that the max value for this must be less than 19 usec with the
 * default CLKCON.TICKSPD and CLKCON.CLOCKSPD settings and external 26 MHz
 * crystal as a clock source (which we use).
 *
 * Be careful of direct calls to Mrfi_DelayUsec().
 */
#define MRFI_MAX_DELAY_US 16 /* usec */


/* frame header size definition 
// CC1101 data sheet中关于帧长的定义

In variable packet length mode, PKTCTRL0.LENGTH_CONFIG=1, the packet length is configured by the first byte after the sync word. 
The packet length is defined as the payload data, excluding the length byte and the optional CRC. The PKTLEN register is used to set the maximum packet length
allowed in RX.
TX和RX FIFO最大64个字节，第一个字节用于指示长度，写入这个字节的值是该字节以后到CRC之前的字节数，
不包括该字节本身，也不包括CRC字节。CRC不占用TX FIFO，这样在TX FIFO中，最多还可以一次装入63个字节.
但考虑接收FIFO也是64个字节，而这64个字节要包含接收的CRC，所以实际上真实的负载长度最多为61个字节
*/


#define MRFI_MAX_FRAME_SIZE        127   //最多装入的不包括长度字节和CRC两个字节的负载长度
#define MRFI_RX_METRICS_SIZE                  2  //CRC长度
#define MRFI_RX_METRICS_RSSI_OFS          0  // 接收处理后，将收到的CRC的第一个字节变为RSSI指示域
#define MRFI_RX_METRICS_CRC_LQI_OFS    1  // 将收到的CRC第二个字节变为LQI指示域，同时最高位指示CRC是否通过。

/* Radio States */
#define MRFI_RADIO_STATE_UNKNOWN  0
#define MRFI_RADIO_STATE_OFF      1
#define MRFI_RADIO_STATE_IDLE     2
#define MRFI_RADIO_STATE_RX       3

#define MAX_HOPS  3
/* Platform constant used to calculate worst-case for an application
 * acknowledgment delay. Used in the NWK_REPLY_DELAY() macro.
 *

                                      processing time on peer
                                      |   round trip
                                      |   |      max number of replays
                                      |   |      |             number of backoff opportunities
                                      |   |      |             |         average number of backoffs
                                      |   |      |             |         |                                    */
#define   PLATFORM_FACTOR_CONSTANT   (2 + 2*(MAX_HOPS*(MRFI_CCA_RETRIES*(8*MRFI_BACKOFF_PERIOD_USECS)/1000)))


/* the TimeoutTerminator_t type is the prototype used for functions that test
 * semaphores and are passed to the MRFI_WaitTimeoutUsec and MRFI_WaitTimeoutMS
 * functions.
 */
  typedef  uint8_t  (*TimeoutTerminator_t)( void );

extern uint8_t mac_timer_upper_byte;

void halInit(void);  //processor, board specific initializations


BOOL halGetchRdy(void);  //is a character available from the serial port?

void halRawPut(char c);  //write a byte to serial port, no character interpretation
extern void halInitMACTimer(void); //init timer used for Radio timeouts
UINT32 halGetMACTimer(void); //return timer value
void halOverflowSetCount(UINT32 count);
UINT32 macMcuOverflowCapture(void);
#ifdef LOWSN_COMPILER_NO_RECURSION
UINT32 halISRGetMACTimer(void); //return timer value
#else
#define halISRGetMACTimer() halGetMACTimer()
#endif

void halInitRFChip(void);
LOWSN_STATUS_ENUM halInitRadio(PHY_FREQ_ENUM frequency, BYTE channel, RADIO_FLAGS radio_flags);
void halGetProcessorIEEEAddress(BYTE *buf);
void halGetProcessorIEEEAddress_MAC(BYTE *buf);
void halSetRadioIEEEAddress(void );
LOWSN_STATUS_ENUM halSetRadioIEEEFrequency(PHY_FREQ_ENUM frequency, BYTE channel);
void halSetRadioPANID(UINT16 panid);
void halSetRadioShortAddr(SADDR saddr);
LOWSN_STATUS_ENUM halSendPacket(BYTE flen, BYTE *frm);
void spp_rf_IRQ(void);
LOWSN_STATUS_ENUM halSetChannel(BYTE channel);
void    halSetPowerLevel(uint8_t);
UINT32 halMacTicksToUs(UINT32 x);
UINT8 halGetRandomByte(void);
void    MRFI_RxOn(void);
void    MRFI_RxIdle(void);
void    MRFI_Sleep(void);
void    MRFI_WakeUp(void);
BOOL Mrfi_DelayUsecLong( uint32_t, uint16_t, TimeoutTerminator_t );



void halSleep(UINT8 SleepMode, UINT32 msecs);    //put processor to sleep
void halUtilMemCopy(BYTE *dst, BYTE *src, BYTE len);
void halWaitMs(UINT32 msecs);

void halInitUart(void);
char halGetch (void );
void halPutch(char c);


//call backs to PHY, MAC from HAL
void phyRxCallback(void);
void phyTxStartCallBack(void);
void phyTxEndCallBack(void);
void macRxCallback(BYTE *ptr, BYTE rssi);
void macTxCallback(void);

void evbIntCallback(void);  //Evaluation board slow timer interrupt callback
void usrIntCallback(void);   //general interrupt callback , when this is called depends on the HAL layer.
#ifdef LOWSN_ENABLE_SLOW_TIMER
void usrSlowTimerInt(void); //user interrupt slow timer interrupt callback
#endif


void halShutdown(void); 
void halWarmstart(void);
UINT16 getVoltageValue(void);

void halInitLed(void);
void halLedToggle( uint8_t led );
#endif

