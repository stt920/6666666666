

#ifndef EVBCONFIG_H
#define EVBCONFIG_H

//macros taken from Chipcon EVB header file

#define FOSC 32000000    //CPU Frequency

#define LED_OFF 1
#define LED_ON  0

//I could not get LED2, LED4 to work on my CC2430EB
#define LED1          P1_0 //this is LED1 on the board
#define LED2          P1_3 //this is LED3 on the board

#define INIT_LED1()   do { LED1 = LED_OFF; IO_DIR_PORT_PIN(1, 0, IO_OUT); P1SEL &= ~0x01;} while (0)
#define INIT_LED2()   do { LED2 = LED_OFF; IO_DIR_PORT_PIN(1, 3, IO_OUT); P2SEL &= ~0x08;} while (0)


#define LED1_ON()  (LED1 = LED_ON)
#define LED2_ON()  (LED2 = LED_ON)



#define LED1_OFF()  (LED1 = LED_OFF)
#define LED2_OFF()  (LED2 = LED_OFF)

#define LED1_STATE() (LED1 == LED_ON)
#define LED2_STATE() (LED2 == LED_ON)




#endif



