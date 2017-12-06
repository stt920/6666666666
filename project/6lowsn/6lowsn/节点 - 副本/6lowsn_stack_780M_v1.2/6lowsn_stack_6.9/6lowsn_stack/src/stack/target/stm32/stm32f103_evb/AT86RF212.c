
#include "AT86RF212.h"
#include "stm32f10x.h"
#include "6lowsn_config.h"
#include "String.h"
#include "_STM32F107_Console.h"
#include "console.h"
#include "mac.h"
#define CHANEL 8
#define POWER 
#define RATE_10000BPS

uint16_t RXBYTES;
uint16_t PacketLen;
uint16_t RSSI_value;
uint16_t LQI_value;
uint16_t CRC_state;

UINT8 g_initRadioFlag = 0;
//extern LOWSN_STATUS_ENUM macInitRadio(void);
void Delayms(uint16_t ms)//ms��
{
  u8  i;
  u16 j;
  for(i=ms;i>0;i--)
    for(j=0;j<800;j++);  //2800 //50000
}
//extern uint8_t subtype;
uint8_t frame_test[17] = 
{
  0x12,0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,0x00,0x0d,0xBE,0xEF
};
extern 
uint8_t buffer[128] = 
{
  0x7f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
  0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
  0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
  0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
  0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
  0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
};

void RF212_PortInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);	 
}
void SPI1_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable NSS/GDO0/GDO2 clocks */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
  
  /* Enable SPI2 GPIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  /* Configure SPI1 pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*Enable SPI1.NSS as a GPIO*/
  SPI_SSOutputCmd(SPI1, ENABLE);
  
  /* Configure SPI1 pins: NSS */
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void SPI1_Configuration_Configuration(void)
{
  SPI_InitTypeDef SPI_InitStructure;
  
  /* Enable SPI1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  SPI_Cmd(SPI1, DISABLE);
  
  /* SPI1 configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  
  SPI_Init(SPI1, &SPI_InitStructure);
  
  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}


void RF212_EXTI_Init(void)
{
  //-------------------------config GPIO interface--------------------------//
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //����GPIOA�˿�
  
  //GPIO_SetBits(GPIOB, GPIO_Pin_GD0);                       //Ԥ��Ϊ��
  
  //PB10��11����Ϊ����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   //��������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //50Mʱ���ٶ�
  GPIO_Init(GPIOA, &GPIO_InitStructure);			       //GPIO���ú���
  
  //--------------------config EXIT interface------------------------------//
  EXTI_InitTypeDef  EXTI_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);       //����AFIO�˿�
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
  //GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);
  
  /*�����ж�Ϊ�½��ش���*/
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;                     //ѡ���ж���·0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //����Ϊ�ж����󣬷��¼�����
  //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    //�½��ش���
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;      //�����ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                         //�ⲿ�ж�ʹ��
  EXTI_Init(&EXTI_InitStructure);
#if 0
  /*�����ж�Ϊ�½��ش���*/
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;                     //ѡ���ж���·6
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      //����Ϊ�ж����󣬷��¼�����
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    //�½��ش���
  //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;      //�����ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;                         //�ⲿ�ж�ʹ��
  EXTI_Init(&EXTI_InitStructure);
#endif
  //�ⲿ�ж�NVIC����
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;		//ѡ���ж�ͨ��0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;      //��ռʽ�ж����ȼ�����Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧʽ�ж����ȼ�����Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //ʹ���ж�
  NVIC_Init(&NVIC_InitStructure);
}
/*Read the value from a register.

Parameters:
addr 	a value or variable of uint8_t, the offset of the register

Returns:
register value 
*/ 
uint_8 trx_reg_read (uint_8 addr)
{
  uint8_t value;
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);           //ʹ��CSN
  while(GPIOA->BSRR & GPIO_Pin_6);
  addr|=READ_REGISTER;
  SPI2_SendByte(addr);
  value=SPI2_SendByte(0xff);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);
  return value;
}

/*Write a value to a register.

Parameters:
addr 	a value or variable of uint8_t, the offset of the register
val 	a variable name of radio_reg_t

Returns:
void 
*/
void trx_reg_write (uint_8 addr, uint_8 value)
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);           //ʹ��CSN
  while(GPIOA->BSRR & GPIO_Pin_6);
  addr|=WRITE_REGISTER;
  SPI2_SendByte(addr);
  SPI2_SendByte(value);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);            //����CSN
}

/*Read a value from a subregister.

Parameters:
addr 	value or variable of uint8_t, the offset of the register
mask 	mask value of the bit group (at original position)
pos 	shift value of the bit group

Returns:
value which is read from the sub register.

Note:
For any meaningful combination of the parameters (addr,mask,pos) there exist SR_* macros, 
which are described in the radio-specific registermap. The appropriate function call is
then with the macros: stat = trx_bit_read(SR_TRX_STATUS); 
*/
uint_8 trx_bit_read (radio_addr_t addr, radio_reg_t mask, uint8_t pos)
{
  uint_8 value;
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);           //ʹ��CSN
  while(GPIOA->BSRR & GPIO_Pin_6);
  addr |= READ_REGISTER;
  value = trx_reg_read(addr);
  value &= mask;
  value >>=pos;
  GPIO_SetBits(GPIOA,GPIO_Pin_4);            //����CSN
  return value;
}

/*Write a value to a subregister.

Parameters:
addr 	value or variable of uint8_t, the offset of the register
mask 	mask value of the bit group (at original position)
pos 	shift value of the bit group
val 	value which is written to the sub register

Note:
For any meaningful combination of the parameters (addr,mask,pos) there exist SR_* macros,
which are described in the radio-specific registermap. The appropriate function call is then 
with the macros: trx_bit_write(SR_TRX_CMD,CMD_TRX_OFF);
*/
void  trx_bit_write (radio_addr_t addr, radio_reg_t mask, uint8_t pos, radio_reg_t val)
{
  uint_8 value;
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);           //ʹ��CSN
  while(GPIOA->BSRR & GPIO_Pin_6);
  value = trx_reg_read(addr);
  value &= ~mask;
  val <<= pos;
  val &= mask;
  val |= value;
  trx_reg_write(addr,val);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);            //����CSN
}

/*Read the length of a received frame.

Returns:
length of the received frame 
*/
uint8_t trx_frame_length_read (void)
{
  uint8_t value;
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);           //ʹ��CSN
  while(GPIOA->BSRR & GPIO_Pin_6);
  SPI2_SendByte(READ_FRAME);
  value=SPI2_SendByte(0xff);
  return value;
}

/*Read a frame from the radio transceiver.

Return values:
frame 	Pointer to an array that takes the received bytes. 
The array must be able to store (aMaxPHYPacketSize + 2) bytes. 
The array will receive the frame length byte at index 0, 
followed by the received frame data, and finally an LQI byte. 
The length byte accounts for the frame length and the LQI byte, 
but not for its own size.

Returns:
length of the downloaded frame (including the LQI byte, if radio is AT86RF230), 
which is stored as the first element of the array frame. 
*/ 
uint8_t trx_frame_read ()
{
  uint8_t  i;
  uint8_t  length;
  static uint16_t receivenum = 0;		// RF���յ���֡��
  static uint16_t errorrate = 0;
  static uint8_t LastSN = 0;
  static uint8_t buf[32]={0};
  uint8_t CRC_buf[2] = {0};    
  length = trx_frame_length_read();
  conPrintUINT8(length);
  for(i=0;i<length+3;i++)
  {
    buf[i] = SPI2_SendByte(0xff);
    // conPrintUINT8(buf[i]);  //pu
    
  }
  conPrintROMString("\n");  //pu
  GPIO_SetBits(GPIOB,GPIO_Pin_12);            //����CSN
  
  if(trx_bit_read(SR_RX_CRC_VALID) ==0x01)	    //��FCS��Ч
  {
    //STM32F107_Led_Toggle(2);
    
    /*  if(subtype == 0x25 )
    {
    if(receivenum == 0)
    {
    errorrate = 0;
    receivenum = 1;
  }
           else
    {
    if(buf[13] != ((LastSN+1)%16))
    {
    if(buf[13] > LastSN)
    {
    errorrate = errorrate + buf[13] - LastSN;
  }
                 else
    {
    errorrate = 16 + errorrate + buf[13] - LastSN;
  }
  }
    receivenum ++;
    
    if((receivenum + errorrate) >= 1000)
    {
    CRC_buf[0] = errorrate / 10;
    CRC_buf[1] = errorrate % 10;
    //����ҪУ���ֵ
    STM32F107_Uart_Send(CRC_buf, 2);
    receivenum = 0;
  }
  }
    LastSN = buf[13];
  }
                else if(subtype == 0x24 )
    {
    STM32F107_Uart_Send(buf, length);                 // Ĭ�Ϸ����յ���֡
  }
    */
    
  }// if(trx_bit_read(SR_RX_CRC_VALID) ==0x01)
  
  return length; 
}

/*Read frame in buffer level mode.

This function does a frame read in buffer level mode. 
The frame should be read out during reception of the frame and 
should be finished after the end of the frame was received. 
The buffer empty signal should never occur. For it, the interrupt pin 
is overloaded in the buffer level mode to indicate an empty buffer. 
A buffer empty is signalled by overwriting the frame length byte with zero. 
Thus, a check of the frame length byte==0 is sufficient.

Parameters:
d 	Pointer to an array of Payload bytes that should be read

Returns:
length octet of the downloaded frame 
*/
uint8_t trx_frame_read_blm (uint8_t *d);

/*Write a frame to the radio transceiver.

Parameters:
length 	Number of the bytes in the data array.
frame 	Pointer to an array of bytes that will be sent. 
*/
void trx_frame_write (uint8_t length, uint8_t *frame)
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);           //ʹ��CSN
  while(GPIOA->BSRR & GPIO_Pin_6);
  SPI2_SendByte(WRITE_FRAME);
  SPI2_SendByte(length);
  for(int i=0;i<length;i++)
  {
    SPI2_SendByte(frame[i]);
  }
  GPIO_SetBits(GPIOA,GPIO_Pin_4);            //����CSN
}

/*Transceiver IO initialization function.

This function must be called before any other function in the API. 
It configures the radio IO pins. 
*/
void trx_io_init (void);

/*Transceiver IRQ initialization function.

This function have to be called in order to enable the transceiver interrupts in the AVR. 
*/
void trx_irq_init (void)
{
  trx_reg_write(RG_IRQ_MASK,0xff);
}

/*Set the level of the RESET pin.

Parameters:
val 	0 for LOW or 1 for HIGH level of the pin

Returns:
void 
*/
void trx_pinset_set ()
{
  GPIO_SetBits(GPIOA,GPIO_Pin_8);
}

void trx_pinset_reset ()
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}

/*Set the level of the SLP_TR pin.

Parameters:
val 	0 for LOW or 1 for HIGH level of the pin

Returns:
void 
*/
void trx_pinset_slp_tr ()
{
  GPIO_SetBits(GPIOB,GPIO_Pin_0);
}

void trx_pinreset_slp_tr()
{
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}

/*Set the level of the TST pin.

Parameters:
val 	0 for LOW or 1 for HIGH level of the pin

Note:
Not all plattforms support this pin. 
*/
void trx_pinset_tst (bool val)
{  
  if(val)
  {
    GPIO_SetBits(GPIOA,GPIO_Pin_11);
  } 
  else
  {
    GPIO_ResetBits(GPIOA,GPIO_Pin_11);
  }
}
/*Linker hook to the library provided ISR.

A call to this dummy function causes the linker to link the predefined interrupt service routine.

See also Interrupt Configuration and Handling. 
*/
void trx_require_irq_callback (void);

/*Read data from SRAM.

Parameters:
addr 	start address in the SRAM
length 	number of bytes to be read

Return values:
data 	pointer to an array of bytes, where the read from the SRAM are stored. 
*/
void trx_sram_read (radio_ram_t addr, uint8_t length, uint8_t *data);

/*Write data to SRAM.

Parameters:
addr 	start address in the SRAM
length 	number of bytes to be written
data 	pointer to an array of bytes 
*/
void trx_sram_write (radio_ram_t addr, uint8_t length, uint8_t *data)
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);           //ʹ��CSN
  while(GPIOA->BSRR & GPIO_Pin_6);
  SPI2_SendByte(WRITE_SRAM);
  SPI2_SendByte(addr);
  for(int i=0;i<length;i++)
  {
    SPI2_SendByte(data[i]);
  }
  GPIO_SetBits(GPIOA,GPIO_Pin_4);            //����CSN
}

/*Callback for library defined IRQ service routine.

If the library defined ISR is linked, this function will be called, if a radio transceiver interrupt occurs.

See also Interrupt Configuration and Handling .
*/
void usr_trx_main_irq (uint8_t irqstatus)
{
  trx_reg_read(RG_IRQ_MASK);
}

/*Continuous transmission mode
*/
uint8_t test1=0;
/***************************************************************************
*�������� ��AT86RF212_Init(void)
*����ʱ�� ��2011.03.28
*������   ��Felix Zhang
*�������� ����ʼ��RF231
*������� ����
*������� ����
*������Ϣ ����
*�޸����� ��
*�޸��� ����
*�޸����� ��
***************************************************************************/
void AT86RF212_Init(void)
{
  delay_1ms(1);
  GPIO_ResetBits(GPIOA, RST);//��λ231ģ��
  while(GPIO_ReadOutputDataBit(GPIOA, RST)!= Bit_RESET)
  { 
    GPIO_ResetBits(GPIOA, RST);//��λ231ģ��
  }
  GPIO_ResetBits(GPIOB,SLP_TR);//����SLP_TR���� 
  delay_1ms(2);
  GPIO_SetBits(GPIOA,RST);
  delay_1ms(2);
  while(!(GPIOA->ODR & RST))
  {  
    GPIO_SetBits(GPIOA,RST);
  }
  
  trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);	// Set to TRX_OFF
  while(trx_reg_read(RG_TRX_STATUS) != TRX_OFF);	// Check if it is TRX_OFF
  
  trx_reg_write(RG_IRQ_MASK, 0x0C);		// Enable TRX_END, RX_START Interrupt
  // trx_reg_write(RG_IRQ_MASK, 0x18);		// Enable TRX_END, XXXXX  Interrupt
  trx_bit_write(SR_IRQ_MASK_MODE, 1);	// Interrupt polling enabled
  trx_bit_write(SR_IRQ_POLARITY, 0);		// pin IRQ high active
  
  //    // Default data rate 250k
  //    trx_bit_write(SR_CHANNEL, 0x13);		// Channel 19, 2445 MHz
  trx_bit_write(SR_PA_EXT_EN, 0x01);				// Enable PA
  trx_bit_write(SR_TX_AUTO_CRC_ON, 1);	// Enable auto CRC
  
  trx_bit_write(SR_RX_SAFE_MODE,1);       //֡����
  //
  //    trx_bit_write(SR_TX_PWR, 0x0E);			// Tx output power = 3 dB
  
  
  trx_reg_write(RG_CC_CTRL_0,0x0d);            //�ŵ�ѡ��CC_NUMBER��������Ƶ��=782MHz
  trx_reg_write(RG_CC_CTRL_1,0x04);            //�ŵ�ѡ��CC_NUMBER��������Ƶ��=782MHz
  trx_reg_write(RG_PHY_TX_PWR,0xe7);             //����������� = 5dBm (780MHzƵ������书��)
  
  trx_reg_write(RG_RF_CTRL_0,0x03);               //PA RF LEAD TIME = 2US TX POWER OFFSET = 2db
  while(trx_reg_read(RG_RF_CTRL_0) != 0x03);              //�ȴ��������
  trx_reg_write(RG_TRX_CTRL_2,0x7C);             //����ΪO-QPSK-250��IEEE P802.15��4C�й���   1000kchip/s
  test1 =trx_reg_read(RG_TRX_CTRL_2);
  while(trx_reg_read(RG_TRX_CTRL_2) != 0x7C);              //�ȴ��������
  
  trx_bit_write(SR_PAD_IO_CLKM, PAD_IO_2MA);			// CLKM pin to lowest output driver strength
  trx_bit_write(SR_CLKM_CTRL, CLKM_NO_CLOCK);			// No clock at pin 17 (CLKM), pin set to logic low
  trx_bit_write(SR_CLKM_SHA_SEL, CLKM_SHA_DISABLE);	// CLKM clock rate change appears immediately
  
  trx_bit_write(SR_TRX_CMD, CMD_PLL_ON);
  while(trx_reg_read(RG_IRQ_STATUS) != TRX_IRQ_PLL_LOCK);	// Wait for IRQ_0 (PLL_LOCK)
}





/***************************************************************************
*�������� ��Rf_SendPacket(void)
*����ʱ�� ��2011.08.21
*������   ��Alize wu
*�������� ��Rf231��������
*������� ����
*������� ����
*������Ϣ ����
*�޸����� ��
*�޸��� ����
*�޸����� ��
***************************************************************************/
void Rf231_SendPacket(void)
{
  
  // trx_frame_write (frame_test[0],&frame_test[0] );                 //֡������д��
  
  trx_reg_write(RG_TRX_STATE,0x02);               //��ʼ���ͣ�����BUSY_TX״̬
  trx_reg_write(RG_IRQ_MASK,0x09);              //����IRQ����Ĵ���������IRQ_3��TRX_END��
  while(trx_reg_read(0x01) != 0x09);
  while(trx_bit_read(SR_IRQ_3_TRX_END) == 0x00);
  
}

/***************************************************************************
*�������� ��Rf_SendPacket(Uint8 *txBuffer, Uint8 size)
*����ʱ�� ��2011.08.21
*������   ��Alize wu
*�������� ��Rf212��������
*������� ����
*������� ����
*������Ϣ ����
*�޸����� ��
*�޸��� ����
*�޸����� ��
***************************************************************************/
void Rf212_SendPacket(void)
{
  
  trx_frame_write (frame_test[0],&frame_test[0] );                 //֡������д��
  
  trx_reg_write(RG_TRX_STATE,0x02);               //��ʼ���ͣ�����BUSY_TX״̬
  trx_reg_write(RG_IRQ_MASK,0x09);              //����IRQ����Ĵ���������IRQ_3��TRX_END��
  while(trx_reg_read(0x01) != 0x09);
  while(trx_bit_read(SR_IRQ_3_TRX_END) == 0x00);
  
}
/***************************************************************************
*�������� ��RF212_Sleep_Mode(void)

***************************************************************************/
void RF212_Sleep_Mode( void )
{
  //       Hal_RF212_PortInit();
  
  
  //  GPIO_ResetBits(GPIOE, RST);//��λ212ģ��
  //  delay_1ms(2);;
  //    GPIO_SetBits(GPIOE,RST);
  //    delay_1ms(2);;  
  GPIO_ResetBits(GPIOE,SLP_TR);//����SLP_TR����
  //     trx_reg_write(RG_TRX_CTRL_0,0x18); //��������������̵�����������CLKM��ʱ������ 4mA,����17��ʱ��
  //     test1 =trx_reg_read(RG_TRX_CTRL_0);
  delay_1ms(1);;
  trx_reg_write(RG_TRX_STATE,0x03);             //�����ߵ��շ�״̬����ΪTRX_OFF
  while( ((trx_reg_read(RG_TRX_STATUS))&0x0f) != 0x08 )
  {
    trx_reg_write(RG_TRX_STATE,0x03);
  }    //����TRX_OFF״̬
  delay_1ms(1);;
  GPIO_SetBits(GPIOE,SLP_TR);//����SLP_TR����;  //SLP_TR L-H����SLEEPģʽ
}
void RF212_Weak_Up(void)
{
  GPIO_ResetBits(GPIOE,SLP_TR);//����SLP_TR����
  delay_1ms(1);;
  /*while( ((trx_reg_read(RG_TRX_STATUS))&0x0f) != 0x08 )    //����TRX_OFF״̬
  {
  trx_reg_write(RG_TRX_STATE,0x03);             //�����ߵ��շ�״̬����ΪTRX_OFF
}*/
  RF212_Rx_Mode();
  
}
void RF212_IRQHandler( void )
{
  uint_8 res_status,i,rxBytes;
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line0);
    res_status =  trx_reg_read(RG_IRQ_STATUS) ;        
    //conPrintUINT8(res_status);
    // slipSend(&res_status,1,1);
    //halPutch(res_status);
//    Delayms(1);
    //if(res_status&0x08||(res_status == 0x09)||(res_status == 0x00))
    if(res_status == 0xc2||res_status == 0xff||((res_status&0x0f) == 0x0f)||((res_status&0xf0) == 0xf0)||res_status == 0xC9)
    {
      //conPrintROMString("something wrong!...\n");
      //conPrintROMString("Repairing!...\n");
      g_initRadioFlag = 1;
      //return;
      
    }
    if(res_status==0x08||res_status==0xc0||res_status==0x28||res_status==0x8f||res_status==0x19||res_status==0x0c)
    {  
      //halPutch(res_status);
      Delayms(1);
   //   while((GPIOE->IDR & GPIO_Pin_9) == GPIO_Pin_9)
    //  {
    //    trx_reg_read(RG_IRQ_STATUS);
   //   }
      //trx_frame_read ();
//      Delayms(1);
      spp_rf_IRQ1(res_status);
      //STM32F107_Led_Toggle(1);
    }
    
  } 
}
//pu 2.23
void RF212_Rx_Mode( void )
{
  trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);	// Set to TRX_OFF
  while((trx_reg_read(RG_TRX_STATUS))&0x0f != TRX_OFF);	// Check if it is TRX_OFF
  //	trx_bit_write(SR_IRQ_MASK_MODE,0x01);           //IRQ_MASK_MODE
  //	trx_reg_write(0x0E, 0x08);                       //MASK_RX_END
  trx_reg_write(RG_IRQ_MASK, 0x0C);		// Enable TRX_END, RX_START Interrupt
  trx_bit_write(SR_IRQ_MASK_MODE, 1);	// Interrupt polling enabled
  trx_bit_write(SR_IRQ_POLARITY, 0);		// pin IRQ high active
  trx_bit_write(SR_TRX_CMD, CMD_RX_ON);			// Set to RX_ON
  while((trx_reg_read(RG_TRX_STATUS))&0x0f != RX_ON);	// Check if it is RX_ON	
}

void RF212_Tx_Mode( void )
{
  UINT8 tem;
  while(trx_reg_read(RG_TRX_STATUS)&0x0f != TRX_OFF)	// Check if it is TRX_OFF
  {
    trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);	
  }
  trx_bit_write(SR_TRX_CMD, CMD_PLL_ON);			// TRX_OFF to PLL_ON
  while((tem&0x0f) != 0x09)	    // wait until PLL_ON
  {
    trx_bit_write(SR_TRX_CMD, CMD_PLL_ON);
    tem =trx_reg_read(RG_TRX_STATUS);
  }
}


bool RF212_CCA_Perform(void)
{
  UINT8 trx_status;
  UINT8 cca_status;
  UINT8 cca_done;
  
  /* Ensure that trx is not in SLEEP for register access */
  /* do
  {
  trx_status = set_trx_state(CMD_TRX_OFF);
}
  while (trx_status != TRX_OFF);*/
  while(trx_reg_read(RG_TRX_STATUS)&0x0f != 0x08)	// Check if it is TRX_OFF
  {
    trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);	
    conPrintUINT8(trx_reg_read(RG_TRX_STATUS));
  }
  
  /* no interest in receiving frames while doing CCA */
  // trx_bit_write(SR_RX_PDT_DIS, RX_DISABLE); // disable frame reception indication
  
  /* Set trx to rx mode. */
  /*do
  {
  trx_status = set_trx_state(CMD_RX_ON);
}
  while (trx_status != RX_ON);*/
  trx_bit_write(SR_TRX_CMD, CMD_RX_ON);			// Set to RX_ON
  while((trx_reg_read(RG_TRX_STATUS)&0x0f) != RX_ON);	// Check if it is RX_ON	
  
  /* Start CCA */
  trx_bit_write(SR_CCA_REQUEST, CCA_START);
  
  /* wait until CCA is done */
  //here we got no delay __pu
  // pal_timer_delay(TAL_CONVERT_SYMBOLS_TO_US(CCA_DURATION_SYM));
  do
  {
    /* poll until CCA is really done */
    cca_done = trx_bit_read(SR_CCA_DONE);
  }
  while (cca_done != CCA_COMPLETED);
  
  // set_trx_state(CMD_TRX_OFF);
  //trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
  // while(trx_reg_read(RG_TRX_STATUS) != TRX_OFF);	// Check if it is TRX_OFF
  while(trx_reg_read(RG_TRX_STATUS)&0x0f != 0x88)	// Check if it is TRX_OFF
  {
    trx_bit_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
    //conPrintUINT8(trx_reg_read(RG_TRX_STATUS));
  }
  
  /* Check if channel was idle or busy. */
  if (trx_bit_read(SR_CCA_STATUS) == CCA_CH_IDLE)
  {
    cca_status =1; //PHY_IDLE;
    conPrintROMString("Now, doing CCA  ...  OK  \n");
  }
  else
  {
    cca_status = 0;//PHY_BUSY;
    conPrintROMString("Now, doing CCA  ...  failed  \n");
  }
  
  /* Enable frame reception again. */
  // trx_bit_write(SR_RX_PDT_DIS, RX_ENABLE);
  
  //return (phy_enum_t)cca_status;
  return cca_status;
}
