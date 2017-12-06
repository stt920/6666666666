/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "halStack.h"
#include "6lowsn_config.h"
#include "console.h"
#include "cc1101_spi.h"
#include "AT86RF212.h"
#include "debug.h"

#ifdef  LOWSN_ASYNC_INTIO
extern uint8_t serio_rxBuff[LOWSN_ASYNC_RX_BUFSIZE];
extern uint8_t serio_rxHead, serio_rxTail;
#endif

#ifdef LOWSN_ENABLE_SLOW_TIMER
uint16_t capture = 0;
#endif

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST   
    conPrintROMString_new("\nHardFault infinite loop!");
#endif
#endif    
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST    
    conPrintROMString_new("\nMemManage infinite loop!");
#endif
#endif    
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST    
    conPrintROMString_new("\nBusFault infinite loop!");
#endif
#endif    
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
#ifdef LOWSN_COORDINATOR
#ifdef LOWSN_SLIP_TO_HOST    
    conPrintROMString_new("\nUsageFault infinite loop!");
#endif
#endif    
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}



// CC1101 to SPI interrupt



#ifdef  GELANRUI_BOARD

void EXTI9_5_IRQHandler(void)
{

  // 由于CC1101有时候禁用EXT中断，用于PAPD，所以为了保险，
  // 最好还判断一下当前是否应该有外部中断使能, 这是暂时没有
  // 例子: if (MRFI_SYNC_PIN_INT_IS_ENABLED() && MRFI_SYNC_PIN_INT_FLAG_IS_SET())
  if(EXTI_GetITStatus(CC1101_SPI_EXT_LINE) != RESET)
  {

   //conPrintROMString("#");
   spp_rf_IRQ();
   
   EXTI_ClearITPendingBit(CC1101_SPI_EXT_LINE);


	
  }
}

#else

void EXTI15_10_IRQHandler(void)
{

  // 由于CC1101有时候禁用EXT中断，用于PAPD，所以为了保险，
  // 最好还判断一下当前是否应该有外部中断使能, 这是暂时没有
  // 例子: if (MRFI_SYNC_PIN_INT_IS_ENABLED() && MRFI_SYNC_PIN_INT_FLAG_IS_SET())
  if(EXTI_GetITStatus(CC1101_SPI_EXT_LINE) != RESET)
  {

    spp_rf_IRQ();
   
    EXTI_ClearITPendingBit(CC1101_SPI_EXT_LINE);
  }
}

#endif


#ifdef  LOWSN_ASYNC_INTIO

void USART1_IRQHandler(void)
{
  uint8_t x,y;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
   	

   	serio_rxHead++;
  	 if (serio_rxHead == LOWSN_ASYNC_RX_BUFSIZE ) serio_rxHead = 0;
  	 x = serio_rxHead;  //use tmp variables because of Volatile decl
   	y = USART_ReceiveData(USART1);
   	serio_rxBuff[x] = y;
        
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
  }
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)	    //溢出中断
  {
          USART_ClearFlag(USART1,USART_FLAG_ORE);				//清溢出位	
          y = USART_ReceiveData(USART1);
  }
  
}
void USART2_IRQHandler(void)
{	
	static UINT8 sleepFlag = 0;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)==SET)	    //接到数据中断
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE); 			
	} 	
	
	if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)==SET)	    //溢出中断
	{
		USART_ClearFlag(USART2,USART_FLAG_ORE);				//清溢出位		
	}
}
#endif

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    mac_timer_upper_byte++;

    if (mac_timer_upper_byte >= 0xff)
    {
			mac_timer_upper_byte = 0;
    }

  }

  #ifdef LOWSN_ENABLE_SLOW_TIMER
  
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    capture = TIM_GetCapture1(TIM2);
    TIM_SetCompare1(TIM2, capture + CCR1_VAL);

    //evbIntCallback();  //Evaluation board callback
    usrSlowTimerInt();  //user level interrupt callback
    
  }
  #endif

 }


/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External interrupt Line 0 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI0_IRQHandler(void)
{
	//conPrintROMString("x");
	//halPutch('x');
	//halLedToggle(1); 
	RF212_IRQHandler();
    
}
/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles TIM1 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
   conPrintROMString("x_x");
  
    RF212_IRQHandler();
    
    
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
