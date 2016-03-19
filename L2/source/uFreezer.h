/*
 * uFreezer.h
 * Trevor Blanchard
 *
 * The uFreezer is a pre-emptive OS kernel
 */

#ifndef UFREEZER_H
#define UFREEZER_H

#include <inttypes.h>

/* Enums/Constants
 */

#define MAX_TASKS (8)
#define STACK_SIZE (96)

enum TASK_STATE {
  INVALID = -1,
  READY = 0,
  RUNNING = 1,
  WAIT_TICKS = 2,
  WAIT_SEMAPHORE = 3
};

enum TASK_PRIORITY {
  LOW_PRIORITY = 0,
  HIGH_PRIORITY = 1
};

/* Function prototypes
 */

void initKernel( void );
uint8_t startTask( void ( *func )( void ), uint16_t stackSize);
void stopTask( void );
void sleep( uint16_t systicks );

/* Structs
 */

struct TaskControlBlock {
  uint8_t state;
  uint16_t stackPtr;
  uint8_t priority;
};

#endif
