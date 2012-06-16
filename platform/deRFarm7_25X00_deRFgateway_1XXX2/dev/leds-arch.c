/*
 * Copyright (c) 2005, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Configurable Sensor Network Application
 * Architecture for sensor nodes running the Contiki operating system.
 *
 * Implementation of the LEDs API for deRFgateway by Dresden Elektronik
 *
 * -----------------------------------------------------------------
 *
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 *
 */


#include "contiki-conf.h"
#include "dev/leds.h"
//#include "leds-arch.h"

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

//#include "led.h"
#include <board.h>
#include <pio.h>

//------------------------------------------------------------------------------
//         Local Variables
//------------------------------------------------------------------------------

#ifdef PINS_LEDS
static const Pin pinsLeds[] = {PINS_LEDS};
static const unsigned int numLeds = PIO_LISTSIZE(pinsLeds);
#endif

//------------------------------------------------------------------------------
/// Turns the given LED on if it exists; otherwise does nothing.
/// \param led  Number of the LED to turn on.
/// \return 1 if the LED has been turned on; 0 otherwise.
//------------------------------------------------------------------------------
static unsigned char LED_Set(unsigned int led)
{
#ifdef PINS_LEDS
    // Check if LED exists
    if (led >= numLeds) {

        return 0;
    }

    // Turn LED on
    if (pinsLeds[led].type == PIO_OUTPUT_0) {

        PIO_Set(&pinsLeds[led]);
    }
    else {

        PIO_Clear(&pinsLeds[led]);
    }

    return 1;
#else
    return 0;
#endif
}

//------------------------------------------------------------------------------
/// Turns a LED off.
/// \param led  Number of the LED to turn off.
/// \param 1 if the LED has been turned off; 0 otherwise.
//------------------------------------------------------------------------------
static unsigned char LED_Clear(unsigned int led)
{
#ifdef PINS_LEDS
    // Check if LED exists
    if (led >= numLeds) {

        return 0;
    }

    // Turn LED off
    if (pinsLeds[led].type == PIO_OUTPUT_0) {

        PIO_Clear(&pinsLeds[led]);
    }
    else {

        PIO_Set(&pinsLeds[led]);
    }

    return 1;
#else
    return 0;
#endif
}

/*---------------------------------------------------------------------------*/
void
leds_arch_init( void )
{
#ifdef PINS_LEDS
	unsigned int led;

	for ( led = 0; led < numLeds; ++ led ) {
		// Configure LED
    	(PIO_Configure(&pinsLeds[led], 1));
	}
#endif
}

/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
	// TODO @@@jwg: implement me!
    return 0;
}

/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
	if( leds & LEDS_GREEN ) {
		LED_Set(0);
	}
	else {
		LED_Clear(0);
	}

	if( leds & LEDS_YELLOW ) {
		LED_Set(1);
	}
	else {
		LED_Clear(1);
	}

	if( leds & LEDS_RED ) {
		LED_Set(2);
	}
	else {
		LED_Clear(2);
	}
}
/*---------------------------------------------------------------------------*/
