
#include "hal.h"
#include "halStack.h"
#include "evboard.h"
#include "evbConfig.h"


/******************************************************************************
* Joystick
*
******************************************************************************/
#define JOYSTICK_PUSH         P2_0
#define JOYSTICK_PRESSED()    JOYSTICK_PUSH
#define INIT_JOYSTICK_PUSH() \
    do {                     \
        P2DIR &= ~0x01;      \
        P2INP |= 0x01;       \
    } while (0)

BOOL joystickPushed( void );

typedef enum {CENTRED, LEFT, RIGHT, UP, DOWN} JOYSTICK_DIRECTION;

#define JOYSTICK              P0_6
#define INIT_JOYSTICK()       IO_DIR_PORT_PIN(0, 6, IO_IN)
#define ADC_INPUT_JOYSTICK    0x06

JOYSTICK_DIRECTION getJoystickDirection( void );


EVB_SW_STATE sw_state;



JOYSTICK_DIRECTION getJoystickDirection( void ) {
    INT8 adcValue, i;
    JOYSTICK_DIRECTION direction[2];


    for(i = 0; i < 2; i++){
       adcValue = halAdcSampleSingle(ADC_REF_AVDD, ADC_8_BIT, ADC_INPUT_JOYSTICK);

       if (adcValue < 0x29) {
          direction[i] = DOWN;  // Measured 0x01
       } else if (adcValue < 0x50) {
          direction[i] = LEFT;  // Measured 0x30

          //Not supporting 45 degrees positions (UP-RIGTH)
       } else if (adcValue < 0x45) {
          direction[i] = CENTRED;  // Measured 0x40
       } else if (adcValue < 0x35) {
          direction[i] = RIGHT; // Measured 0x4D
       } else if (adcValue < 0x20) {
          direction[i] = UP;    // Measured 0x5C
       } else {
          direction[i] = CENTRED; // Measured 0x69
       }
    }

    if(direction[0] == direction[1]){
       return direction[0];
    }
    else{
       return CENTRED;
    }
}

#define SW_POLL_TIME   MSECS_TO_MACTICKS(100)

UINT32 last_switch_poll;
//poll the switches
void evbPoll(void){

//only do this if the slow timer not enabled as reading
//the joystick takes a while. If the slowtimer is enabled,
//then that interrupt is handing polling
#ifndef LOWSN_ENABLE_SLOW_TIMER
  if ( halMACTimerNowDelta(last_switch_poll) > SW_POLL_TIME) {
   evbIntCallback();
   last_switch_poll = halGetMACTimer();
  }
#endif

}

//init the board
void evbInit(void){
  halInit();
  INIT_JOYSTICK();
  sw_state.val = 0;
  INIT_LED1();
  INIT_LED2();
}

void evbLedSet(BYTE lednum, BOOL state) {
    switch(lednum) {
       case 1:    if (state) LED1_ON(); else LED1_OFF(); break;
       case 2:    if (state) LED2_ON(); else LED2_OFF(); break;
    }
}

BOOL evbLedGet(BYTE lednum){
  switch(lednum) {
       case 1:    return(LED1_STATE());
       case 2:    return(LED2_STATE());
    }
  return(FALSE);
}


//if joystick pushed up, consider this a S1 button press
//if joystick pushed down, consider this a S2 button press
//does not allow for both buttons to be pressed at once
//tgl bits are set if the state bits become different

void evbIntCallback(void){

  JOYSTICK_DIRECTION x;
  x = getJoystickDirection();
  if (x == CENTRED) {
    sw_state.bits.s1_val = 0;
    sw_state.bits.s2_val = 0;
  }
  else  if (x == UP) sw_state.bits.s1_val = 1;
  else if (x == DOWN) sw_state.bits.s2_val = 1;
  if (sw_state.bits.s1_val != sw_state.bits.s1_last_val) sw_state.bits.s1_tgl = 1;
  if (sw_state.bits.s2_val != sw_state.bits.s2_last_val) sw_state.bits.s2_tgl = 1;
  sw_state.bits.s1_last_val = sw_state.bits.s1_val;
  sw_state.bits.s2_last_val = sw_state.bits.s2_val;
}

