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

typedef enum { E_LOW, F, G, A, B, C, D, E_HIGH } NOTE;

NOTE twinkle[42] = { C, C, G, G, A, A, G, F, F, E_LOW, E_LOW, D, D, C, G, G, F, F, E_LOW, E_LOW, D, G, G, F, F, E_LOW, E_LOW, D, C, C, G, G, A, A, G, F, F, E_LOW, E_LOW, D, D, C };

int InitButterfly( void );
int InitSound( void );
#define beat 160 //default 500
#define volume 400 //default 500

#define attack 100 //milliseconds
#define decay 50 //milliseconds
#define release 50 //milliseconds
#define peak 600 //volume that we reach between attack and decay

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

int
noSound(void)
{
  OCR1A = 0;
  _delay_ms(50);
  return 0;
}
void my_delay_ms(int ms)
{
  while (0 < ms)
  {
    _delay_ms(1);
    --ms;
  }
}
int
mute(int duration)
{
  OCR1A = 0;
  duration= duration * beat;
  my_delay_ms(duration);
  return 0;
}
int
not_mute(int duration)
{
  OCR1A = volume;
  duration= duration * beat;
  my_delay_ms(duration);
  noSound();
  return 0;
}

int
tone(uint16_t key)
{
  ICR1 = key;
  OCR1A = volume;
  _delay_ms(beat);
  noSound();
  return 0;
}

int
play_note(uint16_t key, uint16_t duration, uint16_t uvolume)
{
  ICR1 = key;
  OCR1A = uvolume;
  _delay_ms(duration);
  return 0;
}

int
play_note_adsr(uint16_t key, uint16_t duration, uint16_t uvolume)
{
  ICR1 = key;
  OCR1A = 0;
  int i = 0;

  while(i < attack){
    OCR1A += 6;
    _delay_ms(1);
    i++;
  }

  i = 0;
  while(i < decay){
    OCR1A -= 4;
    _delay_ms(1);
    i++;
  }

  i = 0;
  _delay_ms(duration - (attack + decay + release));

  while(i < release){
    OCR1A -= 8;
    _delay_ms(1);
    i++;
  }
  return 0;
}

#define E_329 ( (uint16_t) ( 2000000UL / (329UL*2) ) )
#define F_349 ( (uint16_t) ( 2000000UL / (349UL*2) ) )
#define G_392 ( (uint16_t) ( 2000000UL / (392UL*2) ) )
#define A_440 ( (uint16_t) ( 2000000UL / (440UL*2) ) )
#define B_494 ( (uint16_t) ( 2000000UL / (494UL*2) ) )
#define C_523 ( (uint16_t) ( 2000000UL / (523UL*2) ) )
#define D_587 ( (uint16_t) ( 2000000UL / (587UL*2) ) )
#define E_659 ( (uint16_t) ( 2000000UL / (659UL*2) ) )

uint16_t notes[8] = { E_329, F_349, G_392, A_440, B_494, C_523, D_587, E_659 };

int main(void)
{
  cli();
  InitButterfly();
  LCD_Init();
  InitSound();
  sei();

  /* insert your hardware initialization here */
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "ICR1: %d", A_440);
  LCD_puts(buffer, 0);
  int i;

  for(i = 0; i < 42; i++){
    play_note_adsr(notes[twinkle[i]], 1000, volume);
  }
  return 0;   /* never reached */
}
