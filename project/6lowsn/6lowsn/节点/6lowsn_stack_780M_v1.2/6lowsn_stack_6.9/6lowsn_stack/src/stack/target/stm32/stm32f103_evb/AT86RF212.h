/*Define to prevent recursive inclusion----------------*/
#ifndef _AT86RF212_H
#define _AT86RF212_H

#include "stm32f10x.h"
#include "stdbool.h"

#define RST         GPIO_Pin_8       
#define SLP_TR      GPIO_Pin_0 

#define GET_CHINA_FREQ(x)           (11 + (2 * x))
//定义AT86RF212端口使能
#define AT86RF212_CSN_High GPIO->BSRR = GPIO_Pin_4
#define AT86RF212_CSN_Low  GPIO->BRR  = GPIO_Pin_4

#define READ_REGISTER      0x80
#define WRITE_REGISTER     0xC0
#define READ_FRAME         0x20
#define WRITE_FRAME        0x60
#define READ_SRAM          0x00
#define WRITE_SRAM         0x40

#define DOG                1
#define PIG                0 

//-----offset of the AT86RF212 registers
#define 	RG_ANT_DIV         (0x0d)
#define 	RG_BATMON          (0x11)
#define 	RG_CC_CTRL_0       (0x13)
#define 	RG_CC_CTRL_1       (0x14)
#define 	RG_CCA_THRES       (0x09)
#define 	RG_CSMA_BE         (0x2f)
#define 	RG_CSMA_SEED_0     (0x2d)
#define 	RG_CSMA_SEED_1     (0x2e)
#define 	RG_FTN_CTRL        (0x18)
#define 	RG_IEEE_ADDR_0     (0x24)
#define 	RG_IEEE_ADDR_1     (0x25)
#define 	RG_IEEE_ADDR_2     (0x26)
#define 	RG_IEEE_ADDR_3     (0x27)
#define 	RG_IEEE_ADDR_4     (0x28)
#define 	RG_IEEE_ADDR_5     (0x29)
#define 	RG_IEEE_ADDR_6     (0x2a)
#define 	RG_IEEE_ADDR_7     (0x2b)
#define 	RG_IRQ_MASK        (0x0e)
#define 	RG_IRQ_STATUS      (0x0f)
#define 	RG_MAN_ID_0        (0x1e)
#define 	RG_MAN_ID_1        (0x1f)
#define 	RG_PAN_ID_0        (0x22)
#define 	RG_PAN_ID_1        (0x23)
#define 	RG_PART_NUM        (0x1c)
#define 	RG_PHY_CC_CCA      (0x08)
#define 	RG_PHY_ED_LEVEL    (0x07)
#define 	RG_PHY_RSSI        (0x06)
#define 	RG_PHY_TX_PWR      (0x05)
#define 	RG_PLL_CF          (0x1a)
#define 	RG_PLL_DCU         (0x1b)
#define 	RG_RF_CTRL_0       (0x16)
#define 	RG_RF_CTRL_1       (0x19)
#define 	RG_RX_CTRL         (0x0a)
#define 	RG_RX_SYN          (0x15)
#define 	RG_SFD_VALUE       (0x0b)
#define 	RG_SHORT_ADDR_0    (0x20)
#define 	RG_SHORT_ADDR_1    (0x21)
#define 	RG_TRX_CTRL_0      (0x03)
#define 	RG_TRX_CTRL_1      (0x04)
#define 	RG_TRX_CTRL_2      (0x0c)
#define 	RG_TRX_STATE       (0x02)
#define 	RG_TRX_STATUS      (0x01)
#define 	RG_VERSION_NUM     (0x1d)
#define 	RG_VREG_CTRL       (0x10)
#define 	RG_XAH_CTRL_0      (0x2c)
#define 	RG_XAH_CTRL_1      (0x17)
#define 	RG_XOSC_CTRL       (0x12)
#define   RG_TST_CTRL_DIGI   (0x36)
#define   RG_PART_NUM        (0x1c) 


/*exist predefined constants which assign symbolic names to numeric values*/
#define 	ACK_DISABLE                   1
#define 	ACK_ENABLE                    0
#define 	ACK_TIME_12_SYMBOLS           0
#define 	ACK_TIME_2_SYMBOLS            1
#define 	AES_CTRL                     (0x83)
#define 	AES_CTRL_MIRROR              (0x94)
#define 	AES_DIR_DECRYPT              (0x08)
#define 	AES_DIR_ENCRYPT              (0)
#define 	AES_MODE_CBC                 (0x20)
#define 	AES_MODE_ECB                 (0x0)
#define 	AES_MODE_KEY                 (0x10)
#define 	AES_REQUEST                  (0x80)
#define 	AES_RY_DONE                  (1)
#define 	AES_RY_NOT_DONE              (0)
#define 	AES_STATE_KEY                (0x84)
#define 	AES_STATUS                   (0x82)
#define 	ALTRATE_100_KBPS_OR_250_KBPS     0
#define 	ALTRATE_200_KBPS_OR_500_KBPS     1
#define 	ALTRATE_400_KBPS_OR_1_MBPS       2
#define 	ALTRATE_ALSO400_KBPS_OR_1_MBPS   3
#define 	ANT_EXT_SW_DISABLE_PINS       0
#define 	ANT_EXT_SW_ENABLE_PINS        1
#define 	ANT_SEL_ANTENNA_0             0
#define 	ANT_SEL_ANTENNA_1             1
#define 	BATMON_NOT_VALID              0
#define 	BATMON_VALID                  1
#define 	BLM_AVAILABLE                (1)
#define 	BUSY_RX                       1
#define 	BUSY_RX_AACK                  17
#define 	BUSY_RX_AACK_NOCLK            30
#define 	BUSY_TX                       2
#define 	BUSY_TX_ARET                  18
#define 	CCA_CH_BUSY                   0
#define 	CCA_CH_IDLE                   1
#define 	CCA_COMPLETED                 1
#define 	CCA_MODE_0                    0
#define 	CCA_MODE_1                    1
#define 	CCA_MODE_2                    2
#define 	CCA_MODE_3                    3
#define 	CCA_ONGOING                   0
#define 	CCA_START                     1
#define 	CLEAR_PD                      0
#define 	CLKM_16MHZ                    5
#define 	CLKM_1_4MHZ                   6
#define 	CLKM_1MHZ                     1
#define 	CLKM_2MHZ                     2
#define 	CLKM_4MHZ                     3
#define 	CLKM_8MHZ                     4
#define 	CLKM_NO_CLOCK                 0
#define 	CLKM_SHA_DISABLE              0
#define 	CLKM_SHA_ENABLE               1
#define 	CLKM_SYMBOL_RATE              7
#define 	CMD_FORCE_PLL_ON              4
#define 	CMD_FORCE_TRX_OFF             3
#define 	CMD_NOP                       0
#define 	CMD_PLL_ON                    9
#define 	CMD_RX_AACK_ON                22
#define 	CMD_RX_ON                     6
#define 	CMD_TRX_OFF                   8
#define 	CMD_TX_ARET_ON                25
#define 	CMD_TX_START                  2
#define 	CRC16_NOT_VALID               0
#define 	CRC16_VALID                   1
#define 	FRAME_VERSION_00              0
#define 	FRAME_VERSION_01              1
#define 	FRAME_VERSION_012             2
#define 	FRAME_VERSION_IGNORED         3
#define 	IRQ_HIGH_ACTIVE               0
#define 	IRQ_LOW_ACTIVE                1
#define 	IRQ_MASK_MODE_OFF             0
#define 	IRQ_MASK_MODE_ON              1
#define 	P_ON                          0
#define 	PAD_CLKM_2MA                  0
#define 	PAD_CLKM_4MA                  1
#define 	PAD_CLKM_6MA                  2
#define 	PAD_CLKM_8MA                  3
#define 	PAD_IO_2MA                    0
#define 	PAD_IO_4MA                    1
#define 	PAD_IO_6MA                    2
#define 	PAD_IO_8MA                    3
#define 	PART_NUM_AT86RF212            7
#define 	PLL_ON                        9
#define 	PROM_MODE_DISABLE             0
#define 	PROM_MODE_ENABLE              1
#define 	RF212_RAM_SIZE               (0x80)
#define 	RG_AES_CTRL_MIRROR           (0x94)
#define 	RSSI_BASE_VAL_BPSK_20        (-100)
#define 	RSSI_BASE_VAL_BPSK_40        (-99)
#define 	RSSI_BASE_VAL_OQPSK_100      (-98)
#define 	RSSI_BASE_VAL_OQPSK_250      (-97)
#define 	RX_AACK_ON                    22
#define 	RX_AACK_ON_NOCLK              29
#define 	RX_DISABLE                    1
#define 	RX_ENABLE                     0
#define 	RX_ON                         6
#define 	RX_ON_NOCLK                   28
#define 	RX_SAFE_MODE_DISABLE          0
#define 	RX_SAFE_MODE_ENABLE           1
#define 	SET_PD                        1
#define 	SLEEP                         15
#define 	SPI_CMD_MODE_DEFAULT          0
#define 	SPI_CMD_MODE_IRQ_STATUS       3
#define 	SPI_CMD_MODE_PHY_RSSI         2
#define 	SPI_CMD_MODE_TRX_STATUS       1
#define 	STATE_TRANSITION_IN_PROGRESS  31
#define 	T_OCT                         32
#define 	T_SYM                         16
#define 	TIMESTAMPING_DISABLE          0
#define 	TIMESTAMPING_ENABLE           1
#define 	TRAC_CHANNEL_ACCESS_FAILURE   3
//#define 	TRAC_CHANNEL_ACCESS_FAILURE  (3)
#define 	TRAC_INVALID                  7
//#define 	TRAC_INVALID                 (7)
#define 	TRAC_NO_ACK                   5
//#define 	TRAC_NO_ACK                  (5)
#define 	TRAC_SUCCESS                  0
//#define 	TRAC_SUCCESS                 (0)
#define 	TRAC_SUCCESS_DATA_PENDING     1
//#define 	TRAC_SUCCESS_DATA_PENDING    (1)
#define 	TRAC_SUCCESS_WAIT_FOR_ACK     2
//#define 	TRAC_SUCCESS_WAIT_FOR_ACK    (2)
#define 	TRX_AES_BLOCK_SIZE           (16)
#define 	TRX_IRQ_AMI                  (0x20)
#define 	TRX_IRQ_AWAKE_END            (0x10)
#define 	TRX_IRQ_BAT_LOW              (0x80)
#define 	TRX_IRQ_CCA_ED_DONE          (0x10)
#define 	TRX_IRQ_CCA_ED_READY         (TRX_IRQ_CCA_ED_DONE)
#define 	TRX_IRQ_PLL_LOCK             (0x01)
#define 	TRX_IRQ_PLL_UNLOCK           (0x02)
#define 	TRX_IRQ_RX_START             (0x04)
#define 	TRX_IRQ_TRX_END              (0x08)
#define 	TRX_IRQ_TRX_UR               (0x40)
#define 	TRX_OFF                       8
#define 	TX_ARET_ON                    25
#define 	TX_AUTO_CRC_DISABLE           0
#define 	TX_AUTO_CRC_ENABLE            1
#define 	UPLD_RES_FT_DISABLE           0
#define 	UPLD_RES_FT_ENABLE            1
#define 	VERSION_NUM_AT86RF212         1


/*offset values, masks and shift values for accessing sub registers in the radio transceiver*/
#define 	SR_AACK_ACK_TIME      0x17, 0x04, 2
#define 	SR_AACK_DIS_ACK       0x2e, 0x10, 4
#define 	SR_AACK_FLTR_RES_FT   0x17, 0x20, 5
#define 	SR_AACK_FVN_MODE      0x2e, 0xc0, 6
#define 	SR_AACK_I_AM_COORD    0x2e, 0x08, 3
#define 	SR_AACK_PROM_MODE     0x17, 0x02, 1
#define 	SR_AACK_SET_PD        0x2e, 0x20, 5
#define 	SR_AACK_UPLD_RES_FT   0x17, 0x10, 4
#define 	SR_ANT_CTRL           0x0d, 0x03, 0
#define 	SR_ANT_EXT_SW_EN      0x0d, 0x04, 2
#define 	SR_ANT_SEL            0x0d, 0x80, 7
#define 	SR_AVDD_OK            0x10, 0x40, 6
#define 	SR_AVREG_EXT          0x10, 0x80, 7
#define 	SR_BATMON_HR          0x11, 0x10, 4
#define 	SR_BATMON_OK          0x11, 0x20, 5
#define 	SR_BATMON_VTH         0x11, 0x0f, 0
#define 	SR_BPSK_OQPSK         0x0c, 0x08, 3
#define 	SR_CC_BAND            0x14, 0x07, 0
#define 	SR_CC_NUMBER          0x13, 0xff, 0
#define 	SR_CCA_DONE           0x01, 0x80, 7
#define 	SR_CCA_ED_THRES       0x09, 0x0f, 0
#define 	SR_CCA_MODE           0x08, 0x60, 5
#define 	SR_CCA_REQUEST        0x08, 0x80, 7
#define 	SR_CCA_STATUS         0x01, 0x40, 6
#define 	SR_CHANNEL            0x08, 0x1f, 0
#define 	SR_CLKM_CTRL          0x03, 0x07, 0
#define 	SR_CLKM_SHA_SEL       0x03, 0x08, 3
#define 	SR_CSMA_LBT_MODE      0x17, 0x40, 6
#define 	SR_CSMA_SEED_0        0x2d, 0xff, 0
#define 	SR_CSMA_SEED_1        0x2e, 0x07, 0
#define 	SR_DVDD_OK            0x10, 0x04, 2
#define 	SR_DVREG_EXT          0x10, 0x08, 3
#define 	SR_ED_LEVEL           0x07, 0xff, 0
#define 	SR_FTN_START          0x18, 0x80, 7
#define 	SR_GC_PA              0x05, 0x60, 5
#define 	SR_GC_TX_OFFS         0x16, 0x03, 0
#define 	SR_IEEE_ADDR_0        0x24, 0xff, 0
#define 	SR_IEEE_ADDR_1        0x25, 0xff, 0
#define 	SR_IEEE_ADDR_2        0x26, 0xff, 0
#define 	SR_IEEE_ADDR_3        0x27, 0xff, 0
#define 	SR_IEEE_ADDR_4        0x28, 0xff, 0
#define 	SR_IEEE_ADDR_5        0x29, 0xff, 0
#define 	SR_IEEE_ADDR_6        0x2a, 0xff, 0
#define 	SR_IEEE_ADDR_7        0x2b, 0xff, 0
#define 	SR_IRQ_0_PLL_LOCK     0x0f, 0x01, 0
#define 	SR_IRQ_1_PLL_UNLOCK   0x0f, 0x02, 1
#define 	SR_IRQ_2_EXT_EN       0x04, 0x40, 6
#define 	SR_IRQ_2_RX_START     0x0f, 0x04, 2
#define 	SR_IRQ_3_TRX_END      0x0f, 0x08, 3
#define 	SR_IRQ_4_CCA_ED_DONE  0x0f, 0x10, 4
#define 	SR_IRQ_5_AMI          0x0f, 0x20, 5
#define 	SR_IRQ_6_TRX_UR       0x0f, 0x40, 6
#define 	SR_IRQ_7_BAT_LOW      0x0f, 0x80, 7
#define 	SR_IRQ_MASK           0x0e, 0xff, 0
#define 	SR_IRQ_MASK_MODE      0x04, 0x02, 1
#define 	SR_IRQ_POLARITY       0x04, 0x01, 0
#define 	SR_JCM_EN             0x0a, 0x20, 5
#define 	SR_MAN_ID_0           0x1e, 0xff, 0
#define 	SR_MAN_ID_1           0x1f, 0xff, 0
#define 	SR_MAX_BE             0x2f, 0xf0, 4
#define 	SR_MAX_CSMA_RETRIES   0x2c, 0x0e, 1
#define 	SR_MAX_FRAME_RETRIES  0x2c, 0xf0, 4
#define 	SR_MIN_BE             0x2f, 0x0f, 0
#define 	SR_OQPSK_DATA_RATE    0x0c, 0x03, 0
#define 	SR_OQPSK_SCRAM_EN     0x0c, 0x20, 5
#define 	SR_OQPSK_SUB1_RC_EN   0x0c, 0x10, 4
#define 	SR_PA_BOOST           0x05, 0x80, 7
#define 	SR_PA_EXT_EN          0x04, 0x80, 7
#define 	SR_PA_LT              0x16, 0xc0, 6
#define 	SR_PAD_IO             0x03, 0xc0, 6
#define 	SR_PAD_IO_CLKM        0x03, 0x30, 4
#define 	SR_PAN_ID_0           0x22, 0xff, 0
#define 	SR_PAN_ID_1           0x23, 0xff, 0
#define 	SR_PART_NUM           0x1c, 0xff, 0
#define 	SR_PDT_THRES          0x0a, 0x0f, 0
#define 	SR_PLL_CF             0x1a, 0x1f, 0
#define 	SR_PLL_CF_START       0x1a, 0x80, 7
#define 	SR_PLL_DCU_START      0x1b, 0x80, 7
#define 	SR_PLL_LOCK_CP        0x11, 0x80, 7
#define 	SR_RESERVED_09_1      0x09, 0xf0, 4
#define 	SR_RESERVED_0a_1      0x0a, 0xc0, 6
#define 	SR_RESERVED_0a_3      0x0a, 0x10, 4
#define 	SR_RESERVED_0d_2      0x0d, 0x78, 3
#define 	SR_RESERVED_10_3      0x10, 0x30, 4
#define 	SR_RESERVED_10_6      0x10, 0x03, 0
#define 	SR_RESERVED_11_2      0x11, 0x40, 6
#define 	SR_RESERVED_14_1      0x14, 0xf8, 3
#define 	SR_RESERVED_15_2      0x15, 0x70, 4
#define 	SR_RESERVED_16_2      0x16, 0x30, 4
#define 	SR_RESERVED_16_3      0x16, 0x0c, 2
#define 	SR_RESERVED_17_1      0x17, 0x80, 7
#define 	SR_RESERVED_17_5      0x17, 0x08, 3
#define 	SR_RESERVED_17_8      0x17, 0x01, 0
#define 	SR_RESERVED_18_2      0x18, 0x7f, 0
#define 	SR_RESERVED_19_2      0x19, 0x0c, 2
#define 	SR_RESERVED_19_3      0x19, 0x03, 0
#define 	SR_RESERVED_1a_2      0x1a, 0x40, 6
#define 	SR_RESERVED_1a_3      0x1a, 0x20, 5
#define 	SR_RESERVED_1b_2      0x1b, 0x7f, 0
#define 	SR_RF_MC              0x19, 0xf0, 4
#define 	SR_RND_VALUE          0x06, 0x60, 5
#define 	SR_RSSI               0x06, 0x1f, 0
#define 	SR_RX_BL_CTRL         0x04, 0x10, 4
#define 	SR_RX_CRC_VALID       0x06, 0x80, 7
#define 	SR_RX_PDT_DIS         0x15, 0x80, 7
#define 	SR_RX_PDT_LEVEL       0x15, 0x0f, 0
#define 	SR_RX_SAFE_MODE       0x0c, 0x80, 7
#define 	SR_SFD_VALUE          0x0b, 0xff, 0
#define 	SR_SHORT_ADDR_0       0x20, 0xff, 0
#define 	SR_SHORT_ADDR_1       0x21, 0xff, 0
#define 	SR_SLOTTED_OPERATION  0x2c, 0x01, 0
#define 	SR_SPI_CMD_MODE       0x04, 0x0c, 2
#define 	SR_SUB_MODE           0x0c, 0x04, 2
#define 	SR_TRAC_STATUS        0x02, 0xe0, 5
#define 	SR_TRX_CMD            0x02, 0x1f, 0
#define 	SR_TRX_OFF_AVDD_EN    0x0c, 0x40, 6
#define 	SR_TRX_STATUS         0x01, 0x1f, 0
#define 	SR_TX_AUTO_CRC_ON     0x04, 0x20, 5
#define 	SR_TX_PWR             0x05, 0x1f, 0
#define 	SR_VERSION_NUM        0x1d, 0xff, 0
#define 	SR_XTAL_MODE          0x12, 0xf0, 4
#define 	SR_XTAL_TRIM          0x12, 0x0f, 0

typedef         unsigned char uint_8;
typedef         unsigned char radio_addr_t;
typedef         unsigned char radio_reg_t;
typedef         unsigned char radio_ram_t;
typedef         unsigned char aes_access_t;


enum aes_access_t{ECB,KEY,CBC};

void 	        trx_aes_io (aes_access_t mode, uint8_t cmd, bool start, uint8_t *idata, uint8_t *odata);
void 	        trx_aes_read (uint8_t cmd, uint8_t *odata);
void 	        trx_aes_write (uint8_t cmd, bool start, uint8_t *idata);
void        	trx_aes_wrrd (uint8_t cmd, bool start, uint8_t *idata, uint8_t *odata);
uint_8          trx_bit_read (radio_addr_t addr, radio_reg_t mask, uint8_t pos);
void        	trx_bit_write (radio_addr_t addr, radio_reg_t mask, uint8_t pos, radio_reg_t val);
uint8_t 	    trx_frame_length_read (void);
uint8_t 	    trx_frame_read ();
uint8_t 	    trx_frame_read_blm (uint8_t *d);
void 	        trx_frame_write (uint8_t length, uint8_t *frame);
void 	        trx_io_init (void);
void 	        trx_irq_init (void);
void 	        trx_pinset_reset ();
void 	        trx_pinset_set ();
void            trx_pinset_slp_tr();
void            trx_pinreset_slp_tr();
void 	        trx_pinset_slptr (bool val);
void 	        trx_pinset_tst (bool val);
uint_8  	    trx_reg_read (uint_8 addr);
void 	        trx_reg_write (uint_8 addr, uint_8 value);
void 	        trx_require_irq_callback (void);
void 	        trx_sram_read (radio_ram_t addr, uint8_t length, uint8_t *data);
void 	        trx_sram_write (radio_ram_t addr, uint8_t length, uint8_t *data);
void 	        usr_trx_main_irq (uint8_t irqstatus);
void            delay(unsigned long delaytime);
void            continuous_transmission_mode();
void            Control_GPIO_Configuration(void);
void            Rf231_SendPacket(void);
void            AT86RF212_Init(void);
void            RF212_Rx_Mode(void);
void            RF212_Tx_Mode(void);  
bool            RF212_CCA_Perform(void);
void            RF212_PortInit(void);
void            SPI1_GPIO_Configuration(void);
void            SPI1_Configuration_Configuration(void);
void            RF212_EXTI_Init(void);
#endif   /*_AT86RF212_H*/
