/* TODO: This header. */


#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Right now, PWM output only works on the pins with
   hardware support.  These are defined in the appropriate
   pins_*.c file.  For the rest of the pins, we default
   to digital output.

   This was copied from wiring_analog.c and modified to
   1) remove code not relevant to the Seeeduino XIAO, and
   2) set the PWM frequency to 20kHz.

   Args:
     value - Range is [0, 255], like analogWrite(), but the actual resolution
             is only [0, 149], so the value will be scaled down.
*/
void analogWrite20kHz(const uint32_t pin, uint32_t value);


#ifdef __cplusplus
}
#endif

