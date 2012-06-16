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
 *			Hardware specific rtimer code for deRFgateway boards
 *			made by Dresden Elektronik <http://www.dresden-elektronik.de>
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 */
#include <AT91SAM7X512.h>

#include "contiki-conf.h"

#include "rtimer.h"
#include "rtimer-arch.h"

#include <aic/aic.h>
#include <tc/tc.h>

//#include "rtimer-arch-interrupt.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static rtimer_clock_t offset;

/**
 * @brief Handles interrupts coming from Timer0
 */
static void ISR_TC0(void)
{
	unsigned int status;

    // Clear status bit to acknowledge interrupt
    status = RTIMER_ARCH_TIMER_BASE->TC_SR;
//    if ( status & AT91C_TC_CPAS ) {
       rtimer_run_next();
//     }
}


/*
 *
 */
void rtimer_arch_init( void ) {

	// enable timer peripheral
    AT91C_BASE_PMC->PMC_PCER |= (1 << AT91C_ID_TC0);

// From ATMEL getting started document
//    // Configure TC for a 4Hz frequency and trigger on RC compare
//      TC_FindMckDivisor(4, BOARD_MCK, &div, &tcclks);
//      TC_Configure(AT91C_BASE_TC0, tcclks | AT91C_TC_CPCTRG);
//      AT91C_BASE_TC0->TC_RC = (BOARD_MCK / div) / 4; // timerFreq / desiredFreq

    // timer clock is BOARD_MCK/1024, compare to Register C interrupt
    TC_Configure(RTIMER_ARCH_TIMER_BASE, AT91C_TC_CLKS_TIMER_DIV5_CLOCK | AT91C_TC_WAVESEL_UP_AUTO | AT91C_TC_WAVE);
    // set compare value
    RTIMER_ARCH_TIMER_BASE->TC_RC = (BOARD_MCK/(1024*2));

    // enable interrupt
    RTIMER_ARCH_TIMER_BASE->TC_IER = AT91C_TC_CPCS;
    AIC_ConfigureIT(RTIMER_ARCH_TIMER_ID, AT91C_AIC_PRIOR_LOWEST, ISR_TC0);
    AIC_EnableIT(RTIMER_ARCH_TIMER_ID);

    // finally start timer
    TC_Start(RTIMER_ARCH_TIMER_BASE);

	PRINTF("rtimer_arch_init: Done\n");
}
/*
 *
 */
void rtimer_arch_schedule( rtimer_clock_t t ) {
	RTIMER_ARCH_TIMER_BASE->TC_RC = t + offset;
	PRINTF("rtimer_arch_schedule: %d\n", t + offset);
}

/*
 *
 */
void
rtimer_arch_set( rtimer_clock_t t )
{
  offset = t -  RTIMER_ARCH_TIMER_BASE->TC_CV;
}

/*
 *
 */
rtimer_clock_t
rtimer_arch_now(void)
{
  return(RTIMER_ARCH_TIMER_BASE->TC_CV + offset);
}

/* eof */
