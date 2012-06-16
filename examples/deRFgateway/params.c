/*
 * Copyright (c) 2012, Swedish Institute of Computer Science.
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
 *
 */
/**
 * \file		params.c
 *
 * \brief		Parameter settings for the deRFgateway platform.
 *
 * \author		Joerg Wolf <gwynpen@googlemail.com>
 */

#define DEBUG 1
#if DEBUG
#define PRINTF printf
#define PRINTD printf
#else
#define PRINTD(...)
#define PRINTF(...)
#endif

#include "contiki.h"
#include "contiki-conf.h"

#include <stdio.h>
#include <string.h>

#include "contiki-net.h"
#include "params.h"

/*
 *
 */
uint8_t
params_get_eth_eui64( uint8_t *eui64 ) {
//	FIXME @@@jwg: read from eeprom!
// Hardcoded MAC for ethernet port: 12-13-ff-fe-ed-be-ef
	eui64[0] = 0x02;
	eui64[1] = 0x12;
	eui64[2] = 0x13;
	eui64[3] = 0xff;
	eui64[4] = 0xfe;
	eui64[5] = 0xed;
	eui64[6] = 0xbe;
	eui64[7] = 0xef;

	PRINTF("eth MAC:  %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x%s", ((uint8_t *)eui64)[0], ((uint8_t *)eui64)[1], ((uint8_t *)eui64)[2], ((uint8_t *)eui64)[3], ((uint8_t *)eui64)[4], ((uint8_t *)eui64)[5], ((uint8_t *)eui64)[6], ((uint8_t *)eui64)[7], "\n");

	return(0);
}

/*
 *
 */
uint8_t
params_get_eth_addr( uint8_t *addr ) {
//	FIXME @@@jwg: read from eeprom!
// Hardcoded ethernet address: 12-13-ff-fe-ed
	addr[0] = 0x02;
	addr[1] = 0x12;
	addr[2] = 0x13;
	addr[3] = 0xff;
	addr[4] = 0xfe;
	addr[5] = 0xed;

	PRINTF("eth addr: %02x-%02x-%02x-%02x-%02x-%02x%s", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], "\n");

	return(0);
}

/*
 *
 */
uint8_t
params_get_radio_eui64( uint8_t *eui64 ) {
//	FIXME @@@jwg: read from eeprom!
// Hardcoded MAC address: 11-22-ff-fe-77-88-77
	eui64[0] = 0x02;
	eui64[1] = 0x11;
	eui64[2] = 0x22;
	eui64[3] = 0xFF;
	eui64[4] = 0xFE;
	eui64[5] = 0x77;
	eui64[6] = 0x88;
	eui64[7] = 0x77;
    return(0);
}

/*
 *
 */
uint8_t
params_get_channel( void ) {
  return(25);
}

/*
 *
 */
uint16_t
params_get_panid( void ) {
  return(0xABCD);
}

/*
 *
 */
uint16_t
params_get_panaddr(void) {
  return(0);
}

/*
 *
 */
uint8_t
params_get_txpower( void )
{
  return(0);
}

/* eof */
