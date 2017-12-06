
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_GPIO_H
#define __APP_GPIO_H


//#define RST         GPIO_Pin_7       
//#define SLP_TR      GPIO_Pin_8   

#define MAXWIREBUFSIZE	32				// Size of UART buffer
#define LENGTH   16

#define CRC16_POLY		0x8005
#define CRC_INIT		0xFFFF

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @defgroup STM32_EVAL_Exported_Functions
  * @{
  */ 

extern uint16_t CCR1_Val;
extern uint8_t _flag;
extern uint8_t _trigger;

#define LED1_GPIO_PORT          GPIOD                           //led tx
#define LED1_GPIO_CLK           RCC_APB2Periph_GPIOD
#define LED1_GPIO_PIN           GPIO_Pin_7

#define LED2_GPIO_PORT          GPIOD                           //led rx
#define LED2_GPIO_CLK           RCC_APB2Periph_GPIOD
#define LED2_GPIO_PIN           GPIO_Pin_13

#define LED3_GPIO_PORT          GPIOD                           //led err
#define LED3_GPIO_CLK           RCC_APB2Periph_GPIOD
#define LED3_GPIO_PIN           GPIO_Pin_3  

void SPI2_GPIO_Configuration(void);
void SPI2_Configuration_Configuration(void);
uint16_t SPI2_SendByte(uint16_t data);
void Sleep_Control( void );
void MAX3221_Control( void );
void delay_1ms(unsigned long delaytime);
void RF_Inter_Confg( void );
void STM32F107_Led_Toggle( uint8_t led );
void STM_TIM_BaseInit( void );
void NVIC_TIM2_Configuration( void );
void TIM2_OCInit( void );
void STM32F107_Led_Init( void );
void STM32F107_Uart_Send(uint8_t *pData, uint8_t num);
void Enter_Standby_Mode( void );
void USART2_Configuration(void);
void RF212_Sleep_Mode(void); 
void RTC_Configuration(void);
void NVIC_Configuration(void);
void SYSCLKConfig_STOP(void);
void EXTI_Configuration(void);
void SysTick_Configuration(void);
void STM32F107_RF212_PortInit(void);
void UART_DATA(void);
void STM32F107_LED_Open(uint8_t  num);
void STM32F107_LED_Close(uint8_t  num);
#ifdef __cplusplus
}
#endif


#endif /* __STM32_EVAL_H */

