/*
 * Copyright (c) 2012, Swedish Institute of Computer Science and other contributors.
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
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *			Hardware specific clock routines for deRFgateway boards with
 *			at91sam7x512 modules made by Dresden Elektronik
 *			<http://www.dresden-elektronik.de>
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 */
#include <board.h>
#include <aic/aic.h>
#include <pit/pit.h>
#include <sys/clock.h>
#include <sys/cc.h>
#include <sys/etimer.h>

#include <AT91SAM7X512.h>

// PIT period value in µseconds.
#define PIT_PERIOD          1000

/* seconds is the number of seconds since startup, returned by clock_seconds() */
volatile unsigned long seconds;

/* scount is the 8 bit counter that counts ticks modulo CLOCK_SECONDS */
static volatile uint16_t 	  scount = 0;
static volatile unsigned long ticks  = 0;

//------------------------------------------------------------------------------
/// Handler for PIT interrupt. Increments the ticks counter.
//------------------------------------------------------------------------------
void ISR_Pit( void ) {
	unsigned int status;

	// Read the PIT status register
	status = PIT_GetStatus() & AT91C_PITC_PITS;
	if ( status != 0 ) {
		// Read the PIVR to acknowledge interrupt and get number of ticks
		//        ticks += (PIT_GetPIVR() >> 20);
		status = PIT_GetPIVR();
		++ticks;
		if ( ++scount == CLOCK_SECOND ) {
			scount = 0;
			seconds++;
		}
		if ( etimer_pending() ) {
			etimer_request_poll();
		}
	}
}

//------------------------------------------------------------------------------
/// Configure the periodic interval timer to generate an interrupt every
/// 10 milliseconds.
//------------------------------------------------------------------------------
void clock_init( void ) {

    // Initialize the PIT to the desired frequency
    PIT_Init(PIT_PERIOD, BOARD_MCK / 1000000);

    // Configure interrupt on PIT
    AIC_DisableIT(AT91C_ID_SYS);
    AIC_ConfigureIT(AT91C_ID_SYS, AT91C_AIC_PRIOR_LOWEST, ISR_Pit);
    AIC_EnableIT(AT91C_ID_SYS);
    PIT_EnableIT();

    // Enable the pit
    PIT_Enable();

    seconds = ticks = scount = 0;
}

/*
 *
 */
clock_time_t clock_time( void ) {
	clock_time_t tmp;
	do {
		tmp = ticks;
	} while ( tmp != ticks );
	return(tmp);
}

//------------------------------------------------------------------------------
/// Waits for the given number of milliseconds (using the timestamp generated
/// by the PIT).
/// \param delay  Delay to wait for, in milliseconds.
//------------------------------------------------------------------------------
void clock_delay( unsigned int delay ) {
	volatile unsigned int start = ticks;
	unsigned int elapsed;
	do {
		elapsed = ticks;
		elapsed -= start;
	} while ( elapsed < delay );
}

/*
 *
 */
unsigned long clock_seconds( void ) {
	return(seconds);
}

/* eof */
