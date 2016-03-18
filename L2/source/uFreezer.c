/* uFreezer.c
 * 
 * A Pre-emptive OS kernel
 *
 */

#include <inttypes.h>
#include "uFreezer.h"
#include "stack.h"

uint8_t stacks[MAX_TASKS][STACK_SIZE];
struct TaskControlBlock TCB[MAX_TASKS];
uint8_t currentTask = -1;

void 
initKernel( void ){
  currentTask = -1;
  int i;
  for( i = 0 ; i < MAX_TASKS ; i++ )
    TCB[i].state = INVALID;
}

uint8_t 
startTask( void (*func) ( void * data ), uint16_t stackSize ){
  uint16_t oldstack;
  uint8_t i;

  for( i = 0 ; i < MAX_TASKS ; i++ ) {
    if( TCB[i].state == INVALID ) {
      
      oldstack = SP;
      SP = (uint16_t) (&stacks[i]) + STACK_SIZE;
      
      TCB[i].state = READY;
      TCB[i].stackPtr = SP;
      asm volatile( "\t push %0\n"
		    "\t push %1\n" ::
		    "r" ((uint8_t)(((uint16_t) task))),
		    "r" ((uint8_t)(((uint16_t) task) >> 8 )));
      PushState();
      SP = oldstack;
      return 0;
    }
  }

  return -1;
}

void
stopTask( void ){

}

void 
sleep( uint16_t sysTicks){

}
