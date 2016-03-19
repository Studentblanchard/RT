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

/**********************
*  STRUCTS
*  - Wait
*  - Semaphore
*  - TaskControlBlock
*  
**********************/

typedef struct Wait {
  uint16_t ticks;
  uint16_t rollover;
} wait;

typedef struct TaskControlBlock {
  uint8_t state;
  uint16_t stackPtr;
  uint8_t priority;
  wait * wait;
} tcb;

typedef struct Semaphore {
  int8_t count;
  tcb * waiting;
} semaphore;

/**********************
*  ENUMS
*  - TASK_STATE
*  - TASK_PRIORITY
*  
**********************/

enum TASK_STATE {
  INVALID = 4,
  READY = 0,
  RUNNING = 1,
  WAIT_TICKS = 2,
  WAIT_SEMAPHORE = 3
};

enum TASK_PRIORITY {
  LOW_PRIORITY = 0,
  HIGH_PRIORITY = 1,
  IDLE_PRIORITY = 2
};

enum BUTTON_CLICK {
  SINGLE = 0,
  DOUBLE = 1
};

/**********************
*  DEFINES
*  - MAX_TASKS
*  - STACK_SIZE
*  
**********************/

#define MAX_TASKS (8)
#define STACK_SIZE (96)

/**********************
*  PROTOTYPES
*  - InitTimer0()
*  - InitSound()
*  - InitSemaphore()
*  - WaitSemaphore()
*  - ReleaseSemaphore()
*  - PushState()
*  - PopState()
*  - InitKernel()
*  - StartTask()
*  - StopTask()
*  - Sleep()
*  - FlashColon()
*  - Melody()
*  - Button()
*  - Idle()
*
**********************/

int InitTimer0( uint8_t preScalar );
int InitSound( void );
void InitSemaphore( semaphore * sem, int8_t n );
uint8_t WaitSemaphore(semaphore * sem );
void ReleaseSemaphore( semaphore * sem );
static inline void PushState( void ) __attribute__ ( ( always_inline ) );
static inline void PopState( void ) __attribute__ ( ( always_inline ) );
void InitKernel( void );
uint8_t StartTask( void ( *func )( void ), uint16_t stackSize, uint8_t priority );
void StopTask( void );
void Sleep( uint16_t systicks );
void lcdTask( void );
void melodyTask( void );
void buttonTask( void );
void idleTask( void );
uint16_t GetSysTicks( void );
void GetSysTicksWait( void );
uint8_t getClick( void );
int InitButtons( void );

void dummy1( void );
void dummy2( void );

/**********************
*  VARIABLES
*  - stacks
*  - TCB
*  - currentTask
*  - systicks
*
**********************/

uint8_t stacks[MAX_TASKS][STACK_SIZE];
volatile tcb TCB[MAX_TASKS];
uint8_t currentTask = -1;
volatile uint16_t systicks = 0;
volatile uint16_t systicks_rollover = 0;
volatile uint16_t systicks_50hz = 0;
volatile uint16_t systicks_50hz_rollover = 0;
volatile wait systicks_wait;
semaphore lcd_sem;

/**********************
*  IMPLEMENTATIONS (in sections)
*  - System Ticks
*  - Stack
*  - Hardware
*  - Semaphores
*  - ISR
*  - Task Control
*  - Tasks
*  - Main
*
**********************/

/**********************
* -------BUTTON--------
**********************/

uint8_t 
GetClick( void ){
  if( PINB & ( 1 << PB4 ) )
    return SINGLE;
  return 2;
}

/**********************
* ----SYSTEM TICKS-----
**********************/

uint16_t 
GetSysTicks( void ) {
  uint16_t tmp;
  cli();
  tmp = systicks;
  sei();
  return tmp;
}

uint16_t 
GetSysTicksRollover( void ) {
  uint16_t tmp;
  cli();
  tmp = systicks_rollover;
  sei();
  return tmp;
}

void 
GetSysTicksWait( void ) {
  cli();
  systicks_wait.ticks = systicks_50hz;
  systicks_wait.rollover = systicks_50hz_rollover;
  sei();
}

/**********************
* -------STACK---------
**********************/

static inline void 
PushState( void )  {
  asm volatile(
                 "\t push __zero_reg__\n"
                 "\t push __tmp_reg__\n"
                 "\t in __tmp_reg__, __SREG__\n"
                 "\t push __tmp_reg__\n"
                 "\t push r2\n"
                 "\t push r3\n"
                 "\t push r4\n"
                 "\t push r5\n"
                 "\t push r6\n"
                 "\t push r7\n"
                 "\t push r8\n"
                 "\t push r9\n"
                 "\t push r10\n"
                 "\t push r11\n"
                 "\t push r12\n"
                 "\t push r13\n"
                 "\t push r14\n"
                 "\t push r15\n"
                 "\t push r16\n"
                 "\t push r17\n"
                 "\t push r18\n"
                 "\t push r19\n"
                 "\t push r20\n"
                 "\t push r21\n"
                 "\t push r22\n"
                 "\t push r23\n"
                 "\t push r24\n"
                 "\t push r25\n"
                 "\t push r26\n"
                 "\t push r27\n"
                 "\t push r28\n"
                 "\t push r29\n"
                 "\t push r30\n"
                 "\t push r31\n"
                 :
                 :
	       );
}

static inline void 
PopState( void ) {
  asm volatile(
                 "\t pop r31\n"
                 "\t pop r30\n"
                 "\t pop r29\n"
                 "\t pop r28\n"
                 "\t pop r27\n"
                 "\t pop r26\n"
                 "\t pop r25\n"
                 "\t pop r24\n"
                 "\t pop r23\n"
                 "\t pop r22\n"
                 "\t pop r21\n"
                 "\t pop r20\n"
                 "\t pop r19\n"
                 "\t pop r18\n"
                 "\t pop r17\n"
                 "\t pop r16\n"
                 "\t pop r15\n"
                 "\t pop r14\n"
                 "\t pop r13\n"
                 "\t pop r12\n"
                 "\t pop r11\n"
                 "\t pop r10\n"
                 "\t pop r9\n"
                 "\t pop r8\n"
                 "\t pop r7\n"
                 "\t pop r6\n"
                 "\t pop r5\n"
                 "\t pop r4\n"
                 "\t pop r3\n"
                 "\t pop r2\n"
                 "\t pop __tmp_reg__\n"
                 "\t out __SREG__, __tmp_reg__\n"
                 "\t pop __tmp_reg__\n"
                 "\t pop __zero_reg__\n"
                 :
                 :
	       );

}


/**********************
* -----HARDWARE--------
**********************/

int 
InitTimer0( uint8_t preScalar ) {
  // TCCR0A register masks
  uint8_t preScalarMask = ~( ( 1 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 ) );
  uint8_t wgmMask = ~( ( 1 << WGM01 ) | ( 1 << WGM00 ) );
  uint8_t ocMask = ~ ( ( 1 << COM0A1 ) | ( 1 << COM0A0 ) );
    
  // TIMSK0 register masks
  uint8_t ocieMask = ~ ( 1 << OCIE0A);
  uint8_t toieMask = ~ ( 1 << TOIE0 );
    
  //TIFR0 register masks
  uint8_t ocf0aMask = ~ ( 1 << OCF0A);
  uint8_t tov0Mask = ~ ( 1 << TOV0);
    
  TCCR0A = ( TCCR0A & ( preScalarMask & wgmMask & ocMask ) ) | ( preScalar | ( 0 << FOC0A) | ( 0 << WGM01) | ( 0 << WGM00 ) | ( 0 << COM0A1 ) | ( 0 << COM0A0) );
    

  TCNT0 = 0;
  TIMSK0 = ( TIMSK0 & ( ocieMask & toieMask ) ) | ( ( 0 << OCIE0A ) | ( 1 << TOIE0 ) );
    
  TIFR0 = ( TIFR0 & ( ocf0aMask & tov0Mask ) ) | ( ( 0 << OCF0A ) | ( 0 << TOV0 ) );
    
  return 0;
}

int 
InitSound( void ) {
  DDRB = DDRB | ( 1 << PB5);
  PORTB = PORTB & ( ~ ( 1 << PB5 ) );
    
  uint8_t const preScalarMask = ~( ( 1 << CS12 ) | ( 1 << CS11 ) | ( 1 << CS10 ) );
  uint8_t const wgm1Mask = ~( ( 1 << WGM11 ) | ( 1 << WGM10 ) );
  uint8_t const wgm2Mask = ~( ( 1 << WGM13 ) | ( 1 << WGM12 ) );
  uint8_t const ocMask = (uint8_t) ~ ( ( 1 << COM1A1 ) | ( 1 << COM1A0 ) );
    
  uint8_t const wgm1Mode = ( ( 0 << WGM11 ) | ( 0<<WGM10 ) );
  uint8_t const wgm2Mode = ( ( 1 << WGM13 ) | ( 0<<WGM12 ) );
  uint8_t const ocMode = ( ( 1 << COM1A1 ) | ( 1 << COM1A0 ) );
    
  TCCR1A = ( TCCR1A & ( wgm1Mask & ocMask ) ) | ( wgm1Mode | ocMode );
  TCCR1B = ( TCCR1B & ( wgm2Mask & preScalarMask) ) | ( (1 << CS10) | wgm2Mode );
    
  TCNT1 = 0;
  ICR1 = 0;
  OCR1A = 0;
    
  return 0;
}

int
InitButtons( void ) {
  DDRB = DDRB & (~ ( 1 << PB4 ) | ( 1 << PB6 ) | ( 1 << PB7 ) );
  PORTB = PORTB | ( ( 1 << PB4 ) | ( 1 << PB6 ) | ( 1 << PB7 ) );

  PCMSK0 = 0;
  PCMSK1 = 0;

  return 0;
}

/**********************
* -----SEMAPHORE-------
**********************/

void 
InitSemaphore( semaphore * sem, int8_t n ) {
  sem->count = n;
  sem->waiting = 0;
}

uint8_t
WaitSemaphore( semaphore * sem ) {
 
  cli();
  if( sem->count == 1 ) {
    sem->count = 0;
    sei();
    return 0;
  } else if ( sem->count == -1 ) {
    sei();
    return -1;
  }
  sem->count--;
  sem->waiting = &TCB[currentTask];
  sem->waiting->state = WAIT_SEMAPHORE;
  sei();
 
  while( sem->count < 1 )
    _delay_ms( 1 );

  return 0;
}

void 
ReleaseSemaphore( semaphore * sem ) {
  cli();
  sem->count++;
  if( sem->waiting != 0 ) {
    sem->waiting->state = READY;
    sem->waiting = 0;
  }
  sei();

}

/**********************
* --------ISR----------
**********************/

ISR(TIMER0_OVF_vect/*, ISR_NAKED*/) {
  /*if(currentTask < MAX_TASKS){
    PushState();
    TCB[currentTask].stackPtr = SP;
  }

  currentTask = (currentTask + 1) % MAX_TASKS;

  while(TCB[currentTask].state != READY){

    if(TCB[currentTask].state == WAIT_TICKS) {
      if(systicks_50hz == TCB[currentTask].wait->ticks && 
         systicks_50hz_rollover == TCB[currentTask].wait->rollover ) 
			TCB[currentTask].state = READY;
    }

    currentTask = (currentTask + 1) % MAX_TASKS;
  }  

  SP = TCB[currentTask].stackPtr;
  PopState();
*/
  systicks++;
  if(systicks == 0) systicks_rollover++;
  if(systicks % 5 == 0) {
    systicks_50hz++;
    if(systicks_50hz == 0) systicks_50hz_rollover++;
  }
  
  //asm volatile( "\t  reti\n"::);
}

/**********************
* ----TASK CONTROL-----
**********************/

void 
InitKernel( void ) {
  currentTask = -1;
  int i;
  for( i = 0 ; i < MAX_TASKS ; i++ ){
    TCB[i].state = INVALID;
    TCB[i].wait = (wait *)malloc(sizeof(wait));
  } 
}

uint8_t 
StartTask( void ( * task ) ( void ), uint16_t stackSize, uint8_t priority ) {
  uint16_t oldstack;
  uint8_t i;

  for( i = 0 ; i < MAX_TASKS ; i++ ) {
    if( TCB[i].state == INVALID ) {
      
      TCB[i].state = READY;
      TCB[i].priority = priority;

      oldstack = SP;
      SP = ( uint16_t ) ( &stacks[i] ) + STACK_SIZE;
      
      asm volatile( "\t push %0\n"
		    "\t push %1\n" 
		    :
		    : "r" ( ( uint8_t ) ( ( ( uint16_t ) task ) ) ),
		    "r" ( ( uint8_t ) ( ( ( uint16_t ) task ) >> 8 ) ) );
      
      PushState();
      TCB[i].stackPtr = SP;
      SP = oldstack;
      break;
    }
  }

  return 0;
}

void
StopTask( void ) {

}

void 
Sleep( uint16_t sysTicks ) {
  GetSysTicksWait();
  
  TCB[currentTask].wait->ticks = (sysTicks % 255) + systicks_wait.ticks;
  TCB[currentTask].wait->rollover = (sysTicks / 255) + systicks_wait.rollover;

  TCB[currentTask].state = WAIT_TICKS;

  while(TCB[currentTask].state == WAIT_TICKS)
	asm volatile("\t  nop\n"::);
}

/**********************
* -------TASKS---------
**********************/

void 
dummy1( void ) {
    uint8_t count = 0;
    char buffer[8];
    for(;;) {
        if( WaitSemaphore( &lcd_sem ) == 0 ){
          sprintf( buffer,"T1: %2d", count % 99 );
          LCD_puts( buffer, 1 );
          count = count + 1;
          Sleep(50);
          ReleaseSemaphore( &lcd_sem );
        }
    }
}

void 
dummy2( void ) {
    uint8_t count = 0;
    char buffer[8];
    for(;;) {
        if( WaitSemaphore( &lcd_sem ) == 0 ) {
          sprintf( buffer,"T2: %2d", count % 99 );
          LCD_puts( buffer, 1 );
          count = count + 1;
          Sleep(5);
          ReleaseSemaphore( &lcd_sem );
        }
    }
}

void 
dummy3( void ) {
    uint8_t i;
    OCR1A = 300;
    for( i = 0;; i++ ) {
        ICR1 = ( i % 2 ) == 0 ? 2000 : 3000;
        Sleep(2);
    }
}

void 
lcdTask( void ){
  char buffer[8];
  sprintf( buffer, "T" );
  for(;;){
    if( WaitSemaphore( &lcd_sem ) == 0 ) {
      LCD_puts( buffer, 1 );
      ReleaseSemaphore( &lcd_sem );
    }
    _delay_ms(1000);//ms
  } 
}

void 
melodyTask( void ){
  uint8_t i;
  OCR1A = 300;
  for( i = 0;; i++ ) {
    ICR1 = ( i % 2 ) == 0 ? 2000 : 3000;
    Sleep( 500 );//ms
  }
}

void 
buttonTask( void ){
  uint8_t click;
  for(;;){
    click = GetClick();
    if( click == SINGLE ){
      if(OCR1A) OCR1A = 0;
      else OCR1A = 300;
    }else if( click == DOUBLE ){
      asm volatile( "\t jmp 0\n" ::);
    }
    Sleep( 1 );
  }
}

void 
idleTask( void ){
  for(;;){
    //_delay_ms(1);
    asm volatile("\t  nop\n"::);
  }
}

/**********************
* -------MAIN----------
**********************/

int main(void) {
    
    char buffer[8];

    cli();
    LCD_Init();
    InitTimer0( ( ( 1 << CS02 ) | ( 0 << CS01 ) | ( 1 << CS00 ) ) );
    InitSound();
    //InitButtons();
    InitKernel();
    //StartTask( & dummy1, STACK_SIZE, HIGH_PRIORITY );
    //StartTask( & dummy2, STACK_SIZE, HIGH_PRIORITY );
    //StartTask( & dummy3, STACK_SIZE, HIGH_PRIORITY );
    //StartTask( & idleTask, STACK_SIZE, IDLE_PRIORITY );
    //StartTask( & buttonTask, STACK_SIZE, LOW_PRIORITY );
    //StartTask( & lcdTask, STACK_SIZE, LOW_PRIORITY );
	//StartTask( & melodyTask, STACK_SIZE, HIGH_PRIORITY );
    InitSemaphore( &lcd_sem, 1 );
    sei();
    uint16_t tmp1;
    uint16_t tmp2;
    for(;;) {
        cli();
        tmp1 = systicks;
        tmp2 = systicks_50hz;
        sei();
        sprintf(buffer,"%2d-%2d", tmp1 % 999, tmp2 % 999);
        LCD_puts(buffer,1);
        _delay_ms(1000);
    }
    return 0;   /* never reached */
}


