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
*  - Semaphore
*  - TaskControlBlock
*  
**********************/

typedef struct TaskControlBlock {
  uint8_t state;
  uint16_t stackPtr;
  uint8_t priority;
  uint16_t wait_ticks;
} tcb;

typedef struct Semaphore {
  volatile int8_t count;
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

#define E_329 ( (uint16_t) ( 2000000UL / (329UL*2) ) )
#define F_349 ( (uint16_t) ( 2000000UL / (349UL*2) ) )
#define G_392 ( (uint16_t) ( 2000000UL / (392UL*2) ) )
#define A_440 ( (uint16_t) ( 2000000UL / (440UL*2) ) )
#define B_494 ( (uint16_t) ( 2000000UL / (494UL*2) ) )
#define C_523 ( (uint16_t) ( 2000000UL / (523UL*2) ) )
#define D_587 ( (uint16_t) ( 2000000UL / (587UL*2) ) )
#define E_659 ( (uint16_t) ( 2000000UL / (659UL*2) ) )

#define MAX_TASKS (8)
#define STACK_SIZE (96)

typedef enum { E_LOW, F, G, A, B, C, D, E_HIGH, NA } NOTE;
NOTE twinkle[48] = { C, C, G, G, A, A, G, NA, F, F, E_LOW, E_LOW, D, D, C, NA, G, G, F, F, E_LOW, E_LOW, D, NA, G, G, F, F, E_LOW, E_LOW, D, NA, C, C, G, G, A, A, G, NA, F, F, E_LOW, E_LOW, D, D, C, NA };
uint16_t notes[9] = { E_329, F_349, G_392, A_440, B_494, C_523, D_587, E_659, 0 };

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
volatile uint16_t systicks_50hz = 0;
semaphore lcd_sem;
int hyst = 0;
int currentNote = 0;

/**********************
*  IMPLEMENTATIONS (in sections)
*  - Button
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

int click = 0, checkfordouble = 0, release = 0;

uint8_t 
GetClick( void ){

  if(checkfordouble > 0)
    checkfordouble --;

  if(checkfordouble == 1)
    return SINGLE;

  if( ! ( PINB & ( 1 << PB4 ) ) && release ) {
    if( checkfordouble > 0 ) return DOUBLE;
    release = 0;
    checkfordouble = 5;
  } else 
    release = 1;

  return -1;
}

/**********************
* ----SYSTEM TICKS-----
**********************/

uint16_t 
GetSysTicks50hz( void ) {
  uint16_t tmp;
  cli();
  tmp = systicks_50hz;
  sei();
  return tmp;
}

uint16_t 
GetSysTicks( void ) {
  uint16_t tmp;
  cli();
  tmp = systicks;
  sei();
  return tmp;
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
  } else if ( sem->waiting != 0 || sem->count == -1 ) {
    sei();
    return -1;
  }
  sem->count--;
  sem->waiting = (tcb*)&(TCB[currentTask]);
  sem->waiting->state = WAIT_SEMAPHORE;
  sei();
 
  while( sem->count < 0 )
    asm volatile("\t  nop\n"::);

  sem->waiting = 0;

  return 0;
}

void 
ReleaseSemaphore( semaphore * sem ) {
  cli();
  sem->count++;
  if( sem->waiting != 0 ) {
    sem->waiting->state = READY;
    //sem->waiting = 0;
  }
  sei();
}

/**********************
* --------ISR----------
**********************/

ISR(TIMER0_OVF_vect,ISR_NAKED) {

  PushState();

  systicks++;

  if(currentTask < MAX_TASKS){
    TCB[currentTask].stackPtr = SP;
  }

  currentTask = (currentTask + 1) % MAX_TASKS;

  int tmp = currentTask;
  int pri = HIGH_PRIORITY;

  while(TCB[currentTask].state != READY && TCB[currentTask].state != pri){
    if(TCB[currentTask].state == WAIT_TICKS) {
      if( systicks >= TCB[currentTask].wait_ticks ) 
	TCB[currentTask].state = READY;
    }

    currentTask = (currentTask + 1) % MAX_TASKS;

    if(tmp == currentTask) 
    {
      if(pri == LOW_PRIORITY) pri = IDLE_PRIORITY;
      else pri = LOW_PRIORITY;
    }
  }  

  SP = TCB[currentTask].stackPtr;

  PopState();

  asm volatile( "\t  reti\n"::);
}

/**********************
* ----TASK CONTROL-----
**********************/

void 
InitKernel( void ) {
  currentTask = -1;
  int i;
  for( i = 0 ; i < MAX_TASKS ; i++ )
    TCB[i].state = INVALID;
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
  TCB[currentTask].wait_ticks = sysTicks + GetSysTicks();

  TCB[currentTask].state = WAIT_TICKS;

  while(TCB[currentTask].state == WAIT_TICKS)
	asm volatile("\t  nop\n"::);
}

/**********************
* -------TASKS---------
**********************/

void 
lcdTask( void ){
  int x = 0;
  for(;;){
    if( WaitSemaphore( &lcd_sem ) == 0 ) {
      x = !x;
      LCD_Colon(x);
      ReleaseSemaphore( &lcd_sem );
    }
    Sleep( 25 );
  } 
}

void 
melodyTask( void ){
  char buffer[8];
  uint8_t i;
  OCR1A = 300;
  for( i = 0;; i++ ) {
    if( WaitSemaphore( &lcd_sem ) == 0 ){
      currentNote = (currentNote+1) %48;
      ICR1 = notes[twinkle[currentNote]];
      sprintf( buffer, "%4d", ICR1 );
      LCD_puts( buffer, 1 );
      ReleaseSemaphore( &lcd_sem );
    }
    Sleep( 25 );
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
    Sleep( 2 );
  }
}

void 
idleTask( void ){
  for(;;)
    asm volatile("\t  nop\n"::);
}

/**********************
* -------MAIN----------
**********************/

int main(void) {
    
    char buffer[8];

    cli();
    InitButtons();
    LCD_Init();
    InitTimer0( ( ( 1 << CS02 ) | ( 0 << CS01 ) | ( 1 << CS00 ) ) );
    InitSound();
    CLKPR = 0x00 | (1 << CLKPCE);
    CLKPR = 0x00;
    InitKernel();
    StartTask( & idleTask, STACK_SIZE, IDLE_PRIORITY );
    StartTask( & buttonTask, STACK_SIZE, LOW_PRIORITY );
    StartTask( & lcdTask, STACK_SIZE, LOW_PRIORITY );
    StartTask( & melodyTask, STACK_SIZE, HIGH_PRIORITY );
    InitSemaphore( &lcd_sem, 1 );
    sei();
    
    for(;;) {
        sprintf(buffer,"%2d", systicks);
        LCD_puts(buffer,1);
        _delay_ms(8000);
    }
    return 0;   /* never reached */
}


