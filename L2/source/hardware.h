/* hardware.h 
 * Trevor Blanchard
 * Prototypes for the hardware init functions
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#include <inttypes.h>

int InitTimer0( uint8_t preScalar );
int InitSound( void );

#endif
