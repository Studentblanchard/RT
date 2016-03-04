/* Name: main.c
 * Author: Trevor Blanchard
 * Course: Real time systems COMP 4550
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <lcd_functions.h>
#include <lcd_driver.h>

#include <stdio.h>
#include <inttypes.h>
#include <setjmp.h>

typedef enum { E_LOW, F, G, A, B, C, D, E_HIGH, NA } NOTE;
typedef enum { RUNNING, KILLED, WAITING, DONE, NEW } TASK_STATE;
typedef enum { CD, PN, CB } TASK;

NOTE twinkle[48] = { C, C, G, G, A, A, G, NA, F, F, E_LOW, E_LOW, D, D, C, NA, G, G, F, F, E_LOW, E_LOW, D, NA, G, G, F, F, E_LOW, E_LOW, D, NA, C, C, G, G, A, A, G, NA, F, F, E_LOW, E_LOW, D, D, C, NA };

int InitButterfly( void );
int InitSound( void );

void yeild(void);

void countdown(void);
void play_note_adsr(void);
void check_buttons(void);

#define volume 400 //default 500
#define attack 100 //milliseconds
#define decay 50 //milliseconds
#define release 50 //milliseconds
#define duration 1000
#define HYST_MAX 12
#define HYST_HIGH 10
#define HYST_LOW 2
#define HYST_MIN 0
#define peak 600 //volume that we reach between attack and decay

typedef struct TCB {
  TASK_STATE state;
  void (*task)(void);
  jmp_buf jb;
} TaskControlBlock;

void (*tasks[])(void) = { countdown, play_note_adsr, check_buttons };
int currentTask;
int currentNote;
int song_length = 48;
TaskControlBlock tcbs[4];

jmp_buf task_controller_jb;

int
InitButterfly( void )
{
  PCMSK0 = 0x00;
  PCMSK1 = 0x00;

  DIDR0 = 0x00;
  DIDR1 = 0x00;
  return 0;
}

int
InitSound( void )
{
  // Enable PortB5 output
  DDRB = DDRB | ( 1 << PB5 );
  // Set output pin Port B5 to 0
  PORTB = ( PORTB & ( ~ ( 1 << PB5 ) ) ) | ( 0 << PB5 );
  // Enable output compare toggle mode
  TCCR1A = ( TCCR1A & (~ ( ( 1 << COM1A1 ) | (1 << COM1A0) ) ) ) | ( ( 1 << COM1A1 ) | ( 0 << COM1A0 ) );
  // Enable phase and frequency correct mode, WGM=1000
  TCCR1B = ( TCCR1B & (~ ( ( 1 << WGM13 ) | (1 << WGM12) ) ) ) | ( ( 1 << WGM13 ) | ( 0 << WGM12 ) );
  TCCR1A = ( TCCR1A & (~ ( ( 1 << WGM11 ) | (1 << WGM10) ) ) ) | ( ( 0 << WGM11 ) | ( 0 << WGM10 ) );

  // Set clock to prescalar of 1
  TCCR1B = ( TCCR1B & (~ ( ( 1 << CS12 ) | ( 1 << CS11 ) | ( 1 << CS10 ) ) ) ) | ( ( 0 << CS12 ) | ( 0 << CS11 ) | ( 1 << CS10 ) );
  return 0;
}

void my_yeild_ms(int ms)
{
  while (0 < ms)
  {
    _delay_ms(1);
    yeild();
    --ms;
  }
}

void my_delay_ms(int ms)
{
  while (0 < ms)
  {
    _delay_ms(10);
    --ms;
  }
}

void
play_note_adsr()
{
  currentNote = 0;
  for(;;){
	ICR1 = twinkle[currentNote];
	OCR1A = 250;
	//int i = 0;
        my_yeild_ms(duration);
	/*
	while(i++ < attack){
		OCR1A += 6;
		yeild();
	}

	i = 0;
	while(i++ < decay){
		OCR1A -= 4;
		yeild();
	}

	my_yeild_ms(duration - (attack + decay + release));

	i = 0;
	while(i++ < release){
		OCR1A -= 8;
		yeild();
	}*/
        currentNote = (currentNote + 1) % song_length;
  }
  return;
}

int hys[5] = { 0, 0, 0, 0, 0 }; 
int j, i, y;

void adjust_hysterisis(int btn){
   if(btn == -1)
   {
     for(j = 0 ; j < 5 ; j++)
       if(hys[j] > 0) hys[j]--;
     return;
   }

   if(hys[btn] < HYST_MAX) hys[btn]++;

   for(i = 1; i < 5; i++)
     if(hys[btn + i] > HYST_MIN) hys[btn + i]--;
}

int btn;

void check_buttons(){
  for(;;){
    btn = -1;
    if(!(PINB & (1 << PB4))){//center
      btn = 0;
    }else if(!(PINB & (1 << PB6))){//up
      btn = 1;
    }else if(!(PINB & (1 << PB7))){//down
      btn = 2;
    }else if(!(PINE & (1 << PE2))){//left
      btn = 3;
    }else if(!(PINE & (1 << PE3))){//right
      btn = 4;
    }
    adjust_hysterisis(btn);
    yeild();  
  }
}

char buffer[16];

void countdown(){
  for(;;){
    if(hys[0] > HYST_HIGH)
    	sprintf(buffer, "%s\n", "center" );
    if(hys[1] > HYST_HIGH)
    	sprintf(buffer, "%s\n", "up" );
    if(hys[2] > HYST_HIGH)
    	sprintf(buffer, "%s\n", "down" );
    if(hys[3] > HYST_HIGH)
    	sprintf(buffer, "%s\n", "left" );
    if(hys[4] > HYST_HIGH)	
    	sprintf(buffer, "%s\n", "right" );
    LCD_puts(buffer, 1);
    yeild();
  }
}

void yeild(){
  if(setjmp(tcbs[currentTask].jb) == 0){
    longjmp(task_controller_jb, 1);
  }
}

uint8_t entry(void (*func)(void)){
  uint8_t stack[100];
  stack[100-1] = 12;
  func();
  return stack;
}

void schedule(void (*task)(void), int taskno){
  tcbs[taskno].state = NEW;
  tcbs[taskno].task = task;
}

void task_controller(){
  currentTask = 0;

  if(setjmp(task_controller_jb) == 0)
  {
    tcbs[currentTask].state = RUNNING;
    entry(tcbs[currentTask].task);
  }else{
    if(tcbs[currentTask].state == DONE){
      schedule(tasks[currentTask], currentTask);
    }

    tcbs[currentTask].state = WAITING;

    currentTask = (currentTask + 1) % 3;

    if(tcbs[currentTask].state == NEW){
       tcbs[currentTask].state = RUNNING;
       entry(tcbs[currentTask].task);
    }else{
       tcbs[currentTask].state = RUNNING;
       longjmp(tcbs[currentTask].jb, 1);
    }
  }
}

#define E_329 ( (uint16_t) ( 2000000UL / (329UL*2) ) )
#define F_349 ( (uint16_t) ( 2000000UL / (349UL*2) ) )
#define G_392 ( (uint16_t) ( 2000000UL / (392UL*2) ) )
#define A_440 ( (uint16_t) ( 2000000UL / (440UL*2) ) )
#define B_494 ( (uint16_t) ( 2000000UL / (494UL*2) ) )
#define C_523 ( (uint16_t) ( 2000000UL / (523UL*2) ) )
#define D_587 ( (uint16_t) ( 2000000UL / (587UL*2) ) )
#define E_659 ( (uint16_t) ( 2000000UL / (659UL*2) ) )

uint16_t notes[9] = { E_329, F_349, G_392, A_440, B_494, C_523, D_587, E_659, 0 };

int main(void)
{
  cli();
  InitButterfly();
  LCD_Init();
  InitSound();
  sei();

  schedule(play_note_adsr,0);
  schedule(countdown,1);
  schedule(check_buttons,2);

  task_controller();

  return 0;   /* never reached */
}
