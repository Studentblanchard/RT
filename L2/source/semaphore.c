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

  //base case, the semaphore is available, decrement and return
  if(sem->count-- > 0) return 0;

  //return 0 if avail
  //
  //
  //otherwise put this thread on the wait queue
  //it will be signaled when the semaphore is avail
  //in this case we return -1
  //
  //any other value signals an error
}

void 
releaseSemaphore(struct Semaphore * sem) {
  //add the the sem
  //????signal the next task?????
}
