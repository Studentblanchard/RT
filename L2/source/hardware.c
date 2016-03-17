/* hardware.c
 * Trevor Blanchard
 * an implementation of the hardware init functions
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include "hardware.h"


int InitTimer0( uint8_t preScalar ) {
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

int InitSound( void ) {
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

