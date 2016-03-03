/* Name: main.c
 * Author: Kiral Poon
 * Copyright: All copyright belongs to author
 * License:  normal public license
 * Freq_ref: http://www.phy.mtu.edu/~suits/notefreqs.html
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <lcd_functions.h>
#include <lcd_driver.h>

#include <stdio.h>
#include <inttypes.h>

typedef enum { E_LOW, F, G, A, B, C, D, E_HIGH, NA } NOTE;

typedef enum {ATTACK, DECAY, SUSTAIN, RELEASE} ADSR_STATE;

ADSR_STATE cur_state;

NOTE twinkle[48] = { C, C, G, G, A, A, G, NA, F, F, E_LOW, E_LOW, D, D, C, NA, G, G, F, F, E_LOW, E_LOW, D, NA, G, G, F, F, E_LOW, E_LOW, D, NA, C, C, G, G, A, A, G, NA, F, F, E_LOW, E_LOW, D, D, C, NA };


int InitButterfly( void );
int InitSound( void );

#define TOP 255

int duration[4] = { 150, 50, 1500, 50 };
int step[4] = { 4, -4, 0 , 8 };

float tickspeed = 0.512f;

int num_rollovers = 0;
int final_set = 0;

volatile uint16_t rollover;

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
stay(){
  uint16_t tmp;
  cli();
  tmp = rollover;
  sei();

  if(tmp >= num_rollovers+1)
    return 0;

  return 1;
}

ISR( TIMER0_COMP_vect ) {
    rollover++;
    if(rollover == num_rollovers)
      OCR0A = final_set;
}

int InitCTCTimer0( uint8_t preScalar, uint8_t duration ) {
    uint8_t const preScalarMask = ~( ( 1 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 ) );
    uint8_t const wgmMask = ~( ( 1 << WGM01 ) | ( 1 << WGM00 ) );
    uint8_t const ocMask = ~ ( ( 1 << COM0A1 ) | ( 1 << COM0A0 ) );
    uint8_t const ieMask = ~ ( ( 1 << TOIE0 ) | ( 1 << OCIE0A ) );
    uint8_t const ifMask = ~ ( ( 1 << TOV0 ) | ( 1 << OCF0A ) );

    uint8_t const wgMode = ( ( 1<<WGM01)|(0<<WGM00) );

    TCCR0A = ( TCCR0A & ( preScalarMask & wgmMask & ocMask ) ) | ( preScalar | wgMode );

    TCNT0 = 0;
    OCR0A = duration;
    TIFR0 = ( TIFR0 & ifMask ) | ( ( 1 << OCF0A ) ) ;
    TIMSK0 = ( TIMSK0 & ieMask ) | ( ( 1 << OCIE0A ) );
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

void calculate_timer(int ms){
  OCR0A = TOP;
  cli();
  rollover = 0;
  sei();
  int tmp = (float)ms / tickspeed;
  num_rollovers = tmp / TOP;
  final_set = tmp % TOP;
  if(final_set == 0)
  {
    final_set = TOP;
    num_rollovers -= 1;
  }
  if(num_rollovers == 0)
    OCR0A = final_set;
}

void my_delay_ms(int ms)
{
  while (0 < ms)
  {
    _delay_ms(1);
    --ms;
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

uint16_t notes[8] = { E_329, F_349, G_392, A_440, B_494, C_523, D_587, E_659, 0 };

void play_adsr_part(int step){
  while(stay())
  {
    OCR1A += step;
    _delay_ms(1);
  }
}

void play_mute(){
  OCR1A = 0;
  while(stay())
    _delay_ms(1);
}

int
playsong(NOTE *melody){
  cur_state = ATTACK;
  int noteindex = 0;
  NOTE cur_note = melody[0];

  for(;;){
    calculate_timer(duration[cur_state]);
    ICR1 = notes[cur_note];

    if(cur_note == NA)
      play_mute();
    else
      play_adsr_part(step[cur_state]);

    //cur_state = (cur_state + 1) % 4;
    if((cur_state = (cur_state + 1) % 4) == ATTACK)//
    {
      OCR1A = 0;
      cur_note = melody[++noteindex % 48];
    }
  }
}

int main(void)
{
  cli();
  InitButterfly();
  InitCTCTimer0( ( ( 1 << CS02 ) | ( 0 << CS02 ) | ( 1 << CS02 ) ) /* Prescalar 1024 */, 0 );
  LCD_Init();
  InitSound();
  sei();

  /* insert your hardware initialization here */
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "Twinkle Twinkle Little Star");
  LCD_puts(buffer, 1);
  int i;

  playsong(twinkle);
  return 0;   /* never reached */
}
