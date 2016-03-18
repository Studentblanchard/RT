/* main.c
 * Trevor Blanchard
 * the scheduler and tasks 
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <lcd_functions.h>
#include <lcd_driver.h>

#include <stdio.h>
#include <inttypes.h>
#include "uFreezer.h"
#include "hardware.h"
#include "stack.h"

volatile uint16_t systicks = 0;

ISR(TIMER0_OVF_vect, ISR_NAKED) {
  TCB[currentTask].stackPtr = SP;
  PushState();

  if(currentTask < MAX_TASKS){
    currentTask = (currentTask + 1) % MAX_TASKS;
  }

  uint8_t tmp = currentTask;
  uint8_t pri = HIGH_PRIORITY;

  while( TCB[currentTask].state != READY && TCB[currentTask].priority != pri ){
    currentTask = (currentTask + 1) % MAX_TASKS;
    if(currentTask == tmp)
      pri = LOW_PRIORITY;
  }

  SP = TCB[currentTask].stackPtr;
  PopState();

  systicks++;
  asm volatile( "\t  reti\n"::);
}

void task1( void ) {
    uint8_t count = 0;
    char buffer[8];
    for(;;) {
        sprintf(buffer,"T1: %2d", count % 99 );
        LCD_puts( buffer, 1 );
        count = count + 1;
        
        _delay_ms(200);
    }
}

void task2( void ) {
    uint8_t i;
    OCR1A = 300;
    for( i = 0;; i++ ) {
        ICR1 = ( i % 2 ) == 0 ? 2000 : 3000;
        _delay_ms(500);
    }
}

int main(void) {
    
    char buffer[8];

    cli();
    LCD_Init();
    InitTimer0( ( ( 1 << CS02 ) | ( 0 << CS01 ) | ( 1 << CS00 ) ) );
    InitSound();
    InitKernel();
    CreateTask( & task1 );
    CreateTask( & task2 );
    sei();
    
    for(;;) {
        sprintf(buffer,"M: %3d", systicks % 999);
        LCD_puts(buffer,1);
        _delay_ms(100);
    }
    return 0;   /* never reached */
}


