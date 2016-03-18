/* semaphore.h
 * Trevor Blanchard
 */

#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <inttypes.h>
#include "uFreezer.h"

struct Semaphore {
  int8_t count;
  struct TaskControlBlock * waiting;
};

void
initSemaphore( struct Semaphore * sem );

uint8_t 
waitSemaphore( struct Semaphore * sem );

void 
releaseSemaphore( struct Semaphore * sem );

#endif
