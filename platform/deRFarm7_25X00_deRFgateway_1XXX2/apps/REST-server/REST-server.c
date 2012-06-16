/*
 * Copyright (c) 2012, Matthias Kovatsch and other contributors.
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
 *			RESTful interface for ND proxy gateway
 *			Code derived from example for the CoAP REST Engine by M.Kovatsch
 * \author
 * 			Matthias Kovatsch <kovatsch@inf.ethz.ch>
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h> 	/* for size_t */
#include <string.h>

#include "REST-server.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((u8_t *)addr)[0], ((u8_t *)addr)[1], ((u8_t *)addr)[2], ((u8_t *)addr)[3], ((u8_t *)addr)[4], ((u8_t *)addr)[5], ((u8_t *)addr)[6], ((u8_t *)addr)[7], ((u8_t *)addr)[8], ((u8_t *)addr)[9], ((u8_t *)addr)[10], ((u8_t *)addr)[11], ((u8_t *)addr)[12], ((u8_t *)addr)[13], ((u8_t *)addr)[14], ((u8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]",(lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3],(lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#include "contiki.h"
#include "contiki-net.h"

#include "dev/leds.h"

#include "erbium.h"
#include "er-coap-07.h"

/*
 * Resources
 *
 * Resources are defined by the RESOURCE macro.
 * Signature: resource name, the RESTful methods it handles, and its URI path (omitting the leading slash).
 */

/* A simple actuator example. Toggles the red led */
RESOURCE(toggle, METHOD_PUT | METHOD_POST, "toggle", "title=\"Red LED\";rt=\"Control\"");
void
toggle_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  leds_toggle(LEDS_RED);
}

/*
 *
 */
PROCESS(rest_server, "RESTserver");

/*
 *
 */
PROCESS_THREAD(rest_server, ev, data)
{
	PROCESS_BEGIN();

	PRINTF("RESTserver started%s", "\n");

	/* Initialize the REST framework. */
	rest_init_engine();

	/* Activate the platform-specific resources. */
	rest_activate_resource(&resource_toggle);

	leds_on(0);

	while (1) {

		PROCESS_WAIT_EVENT();

	} /* while (1) */

	PROCESS_END();
}

/* eof */
