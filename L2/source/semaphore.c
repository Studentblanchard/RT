/* semaphore.c
 * Trevor Blanchard
 */

#include "semaphore.h"
#include <inttypes.h>

void InitSemaphore(struct Semaphore * sem, int8_t n) {
  sem->count = n;
}

uint8_t
waitSemaphore(struct Semaphore * sem) {
 
  cli();
  
  if(sem->count > 0)
    return --(sem->count);
  else if (sem->count < 0)
    return sem->count;

  sei();
  
  sem->waiting = &TCB[currentTask];
  sem->waiting->state = WAIT_SEMAPHORE;
 
  while(sem->count == 0)
    _delay_ms(1);

  return 0;
}

void 
releaseSemaphore(struct Semaphore * sem) {
  cli();
  
  sem->count = 1;
  if(sem->waiting != NULL)
    sem->waiting->state = READY;

  sem->waiting = NULL;
  
  sei();
}
