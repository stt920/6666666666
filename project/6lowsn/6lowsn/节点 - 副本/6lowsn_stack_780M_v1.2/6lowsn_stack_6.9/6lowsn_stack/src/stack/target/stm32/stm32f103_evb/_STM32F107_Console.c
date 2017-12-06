#include "_STM32F107_Console.h"


uint8_t _wiredatabuf[MAXWIREBUFSIZE];		// UART buffer
uint8_t _wirebufhead;			       // UART buffer head pointer
uint8_t _wirebuftail;			     // UART buffer tail pointer
extern volatile uint8_t UART_flag;
extern uint8_t LMP_flag;
uint16_t CCR1_Val = 49152;         // 定时时间
ErrorStatus HSEStartUpStatus;
/*******************************************************************************
* Function Name  : spi2_Configuration
* Description    : Configures the spi2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_Configuration_Configuration(void)
{
  SPI_InitTypeDef SPI_InitStructure;
  
  /* Enable SPI2 clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  
  
  SPI_Cmd(SPI2, DISABLE); 
  
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
  
  SPI_Init(SPI2, &SPI_InitStructure);
  
  /* Enable SPI2  */
  SPI_Cmd(SPI2, ENABLE);
}

/*******************************************************************************
* Function Name  : SPI2_SendData
* Description    : 发送数据
* Input          : data
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t SPI2_SendByte(uint16_t data)
{
    uint16_t temp;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);   
    SPI_I2S_SendData(SPI1, data);    
    SPI_I2S_ClearFlag(SPI1,SPI_I2S_FLAG_TXE);     
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    temp = SPI_I2S_ReceiveData(SPI1);
    return temp;
}

/*******************************************************************************
* Function Name  : SPI_GPIO_Configuration
* Description    : Configures the uart1 GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
     /* Enable NSS/GDO0/GDO2 clocks */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
  
  /* Enable SPI2 GPIO clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  
  /* Configure SPI1 pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
   /*Enable SPI2.NSS as a GPIO*/
  SPI_SSOutputCmd(SPI2, ENABLE);
  
    /* Configure SPI2 pins: NSS */
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*******************************************************************************
* Function Name  : delay
* Description    : delay a short time
* Input          : 延时时间
* Output         : None
* Return         : None
*******************************************************************************/
void delay_1ms(unsigned long delaytime)
{
  unsigned long i,j;
    for (i=0; i<delaytime; i++)
    {
      for (j=0;j<12000;j++); 
    }
    
}

/*******************************************************************************
* Function Name  : Control_GPIO_Configuration
* Description    : Configures the uart2 GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Control_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable Control_GPIO clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  /* Configure Control_GPIO pins: GPIO_Pin_5, GPIO_Pin_6,GPIO_Pin_7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : Control_GPIO_Configuration
* Description    : Configures the sleep mode GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Sleep_Control( void )
{
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE );
  	GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOE, &GPIO_InitStructure);
  	GPIO_ResetBits(GPIOE,GPIO_Pin_1);
}

/*******************************************************************************
* Function Name  : Control_GPIO_Configuration
* Description    : Configures the max3221 GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MAX3221_Control( void )
{
  	GPIO_InitTypeDef GPIO_InitStructure;
        
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE );
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
  	GPIO_ResetBits(GPIOD,GPIO_Pin_2);
        
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 ;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_10);
	GPIO_SetBits(GPIOC,GPIO_Pin_11);
}

/*******************************************************************************
* Function Name  : STM32F107_Led_Init
* Description    : init the led port
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STM32F107_Led_Init( void )
{
  	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                // led tx 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;                // led rx 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                // led err 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );
        
        GPIO_SetBits(GPIOD,GPIO_Pin_3 |  GPIO_Pin_7 | GPIO_Pin_13);
        
}

void NVIC_TIM2_Configuration( void )
{
  	NVIC_InitTypeDef NVIC_InitStructure;
	
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
        
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}

void STM_TIM_BaseInit( void )
{
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 16;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
	TIM_PrescalerConfig( TIM2, 4, TIM_PSCReloadMode_Immediate );
}
/*******************************************************************************
* Function Name  : RF_Inter_Configuration
* Description    : Configures the RF_irq0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RF_Inter_Confg( void )
{
  	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE );
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );
        GPIO_PinRemapConfig( GPIO_FullRemap_TIM1, ENABLE );
        
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOE, &GPIO_InitStructure );
        
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
        TIM_ICInitStructure.TIM_ICPolarity =  TIM_ICPolarity_Rising;
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.TIM_ICFilter = 0x0;
        TIM_ICInit(TIM1, &TIM_ICInitStructure);
        
        
        TIM_ClearITPendingBit( TIM1, TIM_IT_CC1|TIM_IT_Update|TIM_IT_CC2 |TIM_IT_CC3 | TIM_IT_CC4 );
         /* TIM enable counter */
        TIM_Cmd(TIM1, ENABLE);
        TIM_ITConfig( TIM1, TIM_IT_CC1, ENABLE);
}

void TIM2_OCInit( void )
{
  	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init( TIM2, &TIM_OCInitStructure );
	TIM_OC1PreloadConfig( TIM2, TIM_OCPreload_Disable );
        // Clear update interrupt bit
        TIM_ClearITPendingBit( TIM2, TIM_IT_CC1 );
        // Enable update interrupt
	TIM_ITConfig( TIM2, TIM_IT_CC1, ENABLE );
	TIM_Cmd( TIM2, ENABLE );
}

/***************************************************************************
*函数名称 ：STM32F107_RF212_PortInit(void)
*创建时间 ：2010.11.25
*创建人   ：Felix Zhang
*函数功能 ：初始化与RF212的RSTn、SLPRT相连的端口
			SLPRT	PE8
			RSTn	PE7
*输入参数 ：无
*输出参数 ：无
*返回信息 ：无
***************************************************************************/
void STM32F107_RF212_PortInit()
 {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE );
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOE, &GPIO_InitStructure);
 }
/*******************************************************************************
* Function Name  : STM32F107_Led_Toggle
* Description    : init the led port
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STM32F107_Led_Toggle( uint8_t led )
{
  switch (led){
	  case 0:
		GPIOD->ODR ^= GPIO_Pin_7;
		break;
	  case 1:
		GPIOD->ODR ^= GPIO_Pin_13;
		break;
	   case 2:
		GPIOD->ODR ^= GPIO_Pin_3;
		break;
  default:break;
  }
}

/*******************************************************************************
* Function Name  : STM32F107_Uart_Send
* Description    : 发送数据
* Input          : the data address and the number 
* Output         : None
* Return         : None
*******************************************************************************/
void STM32F107_Uart_Send(uint8_t *pData, uint8_t num)
{   
    uint8_t i;
    for(i = 0; i < num; i ++ )
    {
        USART_SendData(USART2, pData[i]);
       while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    }
}

/*******************************************************************************
* Function Name  : Enter_Standby_Mode
* Description    : 发送数据
* Input          : number of the seconds to weak up
* Output         : None
* Return         : None
*******************************************************************************/
void Enter_Standby_Mode( void )
{  
      GPIO_InitTypeDef GPIO_InitStructure;
//      
//      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//      GPIO_Init(GPIOE, &GPIO_InitStructure);      
//      
//      //射频进入休眠状态
//      RF212_Sleep_Mode();     
      delay_1ms(2);
        
      /* IO配置*/
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
      GPIO_Init(GPIOA, &GPIO_InitStructure);  
   
  
      //MAX3221进入低功耗
      GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_Init(GPIOD, &GPIO_InitStructure);
      GPIO_SetBits(GPIOD,GPIO_Pin_2);
      GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_10;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_Init(GPIOC, &GPIO_InitStructure); 
      GPIO_ResetBits(GPIOC,GPIO_Pin_11 | GPIO_Pin_10);
       
      //LED进入低功耗
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                // led tx 
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init( GPIOD, &GPIO_InitStructure );
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;                // led rx 
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init( GPIOD, &GPIO_InitStructure );
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                // led err 
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init( GPIOD, &GPIO_InitStructure ); 
      
      GPIO_SetBits(GPIOD,GPIO_Pin_3 |  GPIO_Pin_7 | GPIO_Pin_13);
      
      TIM_Cmd( TIM2, DISABLE );
      TIM_Cmd( TIM1, DISABLE );
      
        /* Enable PWR and BKP clock */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);   
      /* Configure EXTI Line to generate an interrupt on falling edge */
      EXTI_Configuration();   
      /* Configure RTC clock source and prescaler */
      RTC_Configuration();   
      /* NVIC configuration */
      NVIC_Configuration();      
        /* Configure the SysTick to generate an interrupt each 1 millisecond */
      //SysTick_Configuration();
    
      /* Wait till RTC Second event occurs */
      RTC_ClearFlag(RTC_FLAG_SEC);
      //while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET); 
      /* Alarm in 3 second */
      RTC_SetAlarm(RTC_GetCounter()+ 15);
      /* Wait until last write operation on RTC registers has finished */
      RTC_WaitForLastTask();  
      /* Request to enter STOP mode with regulator in low power mode*/
      PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
      /* Configures system clock after wake-up from STOP: enable HSE, PLL and select 
         PLL as system clock source (HSE and PLL are disabled in STOP mode) */   
}



/***************************************************************************
*函数名称 ：USART2_Configuration
*创建时间 ：2011.11.23
*创建人   ：Alize wu
*函数功能 ：USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  
*输入参数 ：无
*输出参数 ：无
*返回信息 ：无
*修改日期 ：
*修改人 　：
*修改内容 ：
***************************************************************************/
void USART2_Configuration(void)
{
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
 //   RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE );
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef   NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
   
     /*配置Tx-------------------*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
     /*配置Rx-------------------*/
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    /* Enable the USART2 Pins Software Remapping */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* USART_InitStruct members default value */
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No ;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    
    USART_Init(USART2, &USART_InitStruct);
    /* Disable USARTy Receive and Transmit interrupts */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_Cmd(USART2,ENABLE);
}

/**
  * @brief  Configures EXTI Lines.
  * @param  None
  * @retval None
  */
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
  /* RTC clock source configuration ------------------------------------------*/
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset Backup Domain */
  BKP_DeInit();
  
  /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* RTC configuration -------------------------------------------------------*/
  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Set the RTC time base to 1s */
  RTC_SetPrescaler(32767);  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Enable the RTC Alarm interrupt */
  RTC_ITConfig(RTC_IT_ALR, ENABLE);
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
}

/**
  * @brief  Configures NVIC and Vector Table base location.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bits for Preemption Priority and 2 bits for Sub Priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the SysTick to generate an interrupt each 1 millisecond.
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void)
{
 
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *   and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SYSCLKConfig_STOP(void)
{
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

#ifdef STM32F10X_CL
    /* Enable PLL2 */ 
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {
    }
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

/***************************************************************************
*函数名称 ：STM32F107_UART_GetWireByte(uint8_t* rWireByte)
*创建时间 ：2011.03.17
*创建人   ：Felix Zhang
*函数功能 ：从串口缓存中读出一个字节
*输入参数 ：无
*输出参数 ：读出的字节
*返回信息 ：操作是否成功
*修改日期 ：
*修改人 　：
*修改内容 ：
***************************************************************************/
uint8_t STM32F107_UART_GetWireByte(uint8_t* rWireByte)
{
	if(_wirebufhead == _wirebuftail)
	{
		return 0;
	}
	
	*rWireByte = _wiredatabuf[_wirebufhead];
	_wirebufhead ++;
	
	if(_wirebufhead == MAXWIREBUFSIZE)
	{
		_wirebufhead = 0;
	}
	return 1;
}

/***************************************************************************
*函数名称 ：STM32F107_UART_GetOneWireByte(uint8_t* rOneWireByte)
*创建时间 ：2011.03.17
*创建人   ：Felix Zhang
*函数功能 ：从串口缓存中读出一个字节，如果读取失败，则延时1ms后再读，如果连续3次读取失败，则不再读取
*输入参数 ：无
*输出参数 ：读出的字节
*返回信息 ：操作是否成功
*修改日期 ：
*修改人 　：
*修改内容 ：
***************************************************************************/
uint8_t STM32F107_UART_GetOneWireByte(uint8_t* rOneWireByte)
{
	char delayTimes = 0;
	
	while(!STM32F107_UART_GetWireByte(rOneWireByte))
		{
			delay_1ms(1);
			if(delayTimes++ >3)
				return 0;
		}
	return 1;
}

/***************************************************************************
*函数名称 ：STM32F107_LED_Open(Uint8 num)
*创建时间 ：2011.12.5
*创建人   ：wujierong
*函数功能 ：点亮LED
*输入参数 ：LED编号
*输出参数 ：无
*返回信息 ：无
*修改日期 ：
*修改人 　：
*修改内容 ：
***************************************************************************/
void STM32F107_LED_Open(uint8_t  num)
{
	switch(num)
	{
		case 1:
			GPIO_ResetBits(GPIOD, GPIO_Pin_3 );//电源灯
			break;
		case 2:
			GPIO_ResetBits(GPIOD, GPIO_Pin_7);//点亮TX/RX
			break;
                case 3:
                       GPIO_ResetBits(GPIOD, GPIO_Pin_13);//警报
			break;		
               default:
			break;
 	}
}

/***************************************************************************
*函数名称 ：STM32F107_LED_Close(Uint8 num)
*创建时间 ：2011.12.5
*创建人   ：Felix Zhang
*函数功能 ：熄灭LED
*输入参数 ：LED编号
*输出参数 ：无
*返回信息 ：无
*修改日期 ：
*修改人 　：
*修改内容 ：
***************************************************************************/
void STM32F107_LED_Close(uint8_t  num)
{
	switch(num)
	{
		case 1:
			GPIO_SetBits(GPIOD, GPIO_Pin_3);
			break;
		case 2:
			GPIO_SetBits(GPIOD, GPIO_Pin_7);
			break;
                case 3:
                       GPIO_SetBits(GPIOD, GPIO_Pin_13);
		      break;
		default:
			break;
 	}
}


/***************************************************************************
*函数名称 ：STM32F107_UART_SendByte(uint8_t c)
*创建时间 ：2010.11.24
*创建人   ：Felix Zhang
*函数功能 ：
*输入参数 ：发送的字节
*输出参数 ：无
*返回信息 ：无
*修改日期 ：
*修改人 　：
*修改内容 ：
***************************************************************************/
void STM32F107_UART_SendByte(uint8_t c)
{
  USART_SendData(USART2, c);
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}