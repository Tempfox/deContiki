/*
 * Copyright (c) 2012, Matthias Kovatsch; Swedish Institute of Computer Science
 * and other contributors.
 *
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
 *      Example for the CoAP REST Engine on deRFnode boards by Dresden Elektronik
 *      <http://www.dresden-elektronik.de>
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 * \author
 * 		Joerg Wolf <gwynpen@googlemail.com> (adapted to deRFnode boards)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h> 	/* for size_t */
#include <string.h>

#include <avr/wdt.h>

#include "lib/sensors.h"

#include "contiki.h"
#include "contiki-net.h"

#include "erbium.h"

extern uint16_t get_nodeid_from_eeprom(void);

#include "dev/leds.h"

#if defined (PLATFORM_HAS_BATT_SENSOR)
	#ifndef PLATFORM_HAS_ANY_SENSORS
		#define PLATFORM_HAS_ANY_SENSORS 1
	#endif
	#include "dev/battery-sensor.h"
#endif

#if PLATFORM_HAS_ANY_SENSORS
SENSORS(
#if PLATFORM_HAS_BATT_SENSOR
		&battery_sensor
#endif
);
#endif

/* For CoAP-specific example: not required for normal RESTful Web service. */
#if WITH_COAP == 7
	#include "er-coap-07.h"
#else
	#warning "Unexpected CoAP version!"
#endif /* CoAP-specific example */

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

static char out_buf[200];
static struct etimer et;

/*
 * Perform reboot
 */
static void watchdog_reboot(void) {
	cli();
	wdt_enable(WDTO_2S); //wd on,2s
	while (1)
		; //loop
}

/*
 * Initiate a watch dog reboot after waiting.
 */
static struct etimer et;

PROCESS( reboot_process, "reboot");

PROCESS_THREAD( reboot_process, ev, data )
{
	PROCESS_BEGIN();

	etimer_set(&et, 3 * CLOCK_SECOND);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

	watchdog_reboot();

	PROCESS_END();
}


/* ********************************************************************************************************
 * Resources
 *
 * Resources are defined by the RESOURCE macro.
 * Signature: resource name, the RESTful methods it handles, and its URI path (omitting the leading slash).
 */

/*
 * Resource root
 */
RESOURCE( root, METHOD_GET, "root", "");
void root_handler( void* request, void* response, uint8_t *buffer,
				   uint16_t preferred_size, int32_t *offset)
{
	int payload_len = 0;

	payload_len = snprintf(out_buf, sizeof(out_buf), "%s (%s %s) ",	CONTIKI_VERSION_STRING, __DATE__, __TIME__);
//	payload_len += uptime(out_buf + payload_len);

	REST.set_header_content_type(response, TEXT_PLAIN);
	REST.set_response_payload(response, (uint8_t*) out_buf, payload_len);
}

/*
 * Reboot mote
 */
RESOURCE( reboot, METHOD_POST | METHOD_PUT, "reboot", "");
void reboot_handler( void* request, void* response, uint8_t *buffer,
				     uint16_t preferred_size, int32_t *offset)
{
	REST.set_response_status(response, CONTENT_2_05);
	REST.set_response_payload(response, (uint8_t*) "initiating reboot", 17);

	process_start(&reboot_process, NULL);
}

/*
 * Get process list in plain text
 */
RESOURCE( processes, METHOD_GET, "processes", "");
void processes_handler( void* request, void* response, uint8_t *buffer,
						uint16_t preferred_size, int32_t *offset)
{
	#if 0
	#define CHUNKS_TOTAL    1280
	int32_t strpos = 0;
	PRINTF("preferred_size: %u\n",preferred_size);

	/* Check the offset for boundaries of the resource data. */
	if (*offset>=CHUNKS_TOTAL)
	{
	REST.set_response_status(response, REST.status.BAD_OPTION);
	/* A block error message should not exceed the minimum block size (16). */

	const char *error_msg = "BlockOutOfScope";
	REST.set_response_payload(response, error_msg, strlen(error_msg));
	return;
}

/* Generate data until reaching CHUNKS_TOTAL. */
while (strpos<preferred_size)
{
	strpos += snprintf((char *)buffer+strpos, preferred_size-strpos+1, "|%ld|", *offset);
}

/* snprintf() does not adjust return value if truncated by size. */
if (strpos > preferred_size)
{
	strpos = preferred_size;
}

/* Truncate if above CHUNKS_TOTAL bytes. */
if (*offset+(int32_t)strpos > CHUNKS_TOTAL)
{
	strpos = CHUNKS_TOTAL - *offset;
}

REST.set_response_payload(response, buffer, strpos);
REST.set_header_content_type(response, REST.type.TEXT_PLAIN);

/* IMPORTANT for chunk-wise resources: Signal chunk awareness to REST engine. */
*offset += strpos;

/* Signal end of resource representation. */
if (*offset>=CHUNKS_TOTAL)
{
	*offset = -1;
}
#else
	void* ptr;
	int index = 0;
	//	char out_buf[400];
	size_t strpos = 0; /* position in overall string (which is larger than the buffer) */
	size_t bufpos = 0; /* position within buffer (bytes written) */
	size_t tmplen = 0;

	for ( ptr = PROCESS_LIST(); ptr != NULL; ptr = ((struct process *) ptr)->next ) {
		printf("%s\n", ((struct process *) ptr)->name);
	}

	for ( ptr = PROCESS_LIST(); ptr != NULL; ptr = ((struct process *) ptr)->next ) {
		tmplen = strlen(((struct process *) ptr)->name);
		if ( strpos + tmplen > *offset ) {
			bufpos += snprintf((char*) buffer + bufpos, preferred_size - bufpos + 1,
								"%s\r\n", ((struct process *) ptr)->name);
			if ( bufpos >= preferred_size ) {
				break;
			}
		}
		strpos += tmplen;

		/* buffer full, but resource not completed yet; or: do not break if resource exactly fills buffer. */
		if ( bufpos >= preferred_size && strpos - bufpos > *offset ) {
			PRINTF("res: BREAK at %s\n", ((struct process *)ptr)->name);
			break;
		}
	}

	if ( bufpos > 0 ) {
		PRINTF("BUF %d: %.*s\n", bufpos, bufpos, (char *) buffer);

		coap_set_payload(response, buffer, bufpos);
		coap_set_header_content_type(response, APPLICATION_LINK_FORMAT);
	} else if ( strpos > 0 ) {
		PRINTF("processes_handler(): bufpos<=0\n");
		coap_set_status_code(response, BAD_OPTION_4_02);
		coap_set_payload(response, "BlockOutOfScope", 15);
	}

	if ( ptr == NULL ) {
		PRINTF("res: DONE\n");
		*offset = -1;
	} else {
		PRINTF("res: MORE at %s\n", ((struct process *)ptr)->name);
		*offset += preferred_size;
	}
#endif
}

/*A simple actuator example, depending on the color query parameter and post variable mode, corresponding led is activated or deactivated*/
RESOURCE(led, METHOD_POST | METHOD_PUT , "leds", "title=\"LEDs: ?color=r|g|b, POST/PUT mode=on|off\";rt=\"Control\"");

void
led_handler( void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset )
{
	size_t len = 0;
	const char *color = NULL;
	const char *mode = NULL;
	uint8_t led = 0;
	int success = 1;

	if ( (len=REST.get_query_variable(request, "color", &color)) ) {
		PRINTF("color %d.%s\n", len, color);
		if ( 0 == strncmp(color, "r", len) ) {
			led = LEDS_RED;
		} else if( 0 == strncmp(color,"g", len) ) {
			led = LEDS_GREEN;
		} else if (0 == strncmp(color,"b", len) ) {
			led = LEDS_BLUE;
		} else {
			success = 0;
		}
	} else {
		success = 0;
	}

	if ( success && ( len = REST.get_post_variable(request, "mode", &mode)) ) {
		PRINTF("mode %s\n", mode);

		if ( 0 == strncmp(mode, "on", len) ) {
			leds_on(led);
		} else if ( 0 == strncmp(mode, "off", len) ) {
			leds_off(led);
		} else {
			success = 0;
		}
	} else {
		success = 0;
	}

	if ( !success ) {
		REST.set_response_status(response, REST.status.BAD_REQUEST);
	}
}

/* A simple actuator example. Toggles the red led */
RESOURCE(toggle, METHOD_PUT | METHOD_POST, "toggle", "title=\"Red LED\";rt=\"Control\"");
void
toggle_handler( void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset )
{
	leds_toggle(LEDS_RED);
}

#if PLATFORM_HAS_BATT_SENSOR
/* A simple getter example. Returns the reading from light sensor with a simple etag */
RESOURCE( battery, METHOD_GET, "battery", "title=\"Battery status\";rt=\"Battery\"" );
void
battery_handler( void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset )
{
	uint16_t *accept = NULL;
	int num = REST.get_header_accept(request, &accept);

	if ( (0 == num) || (num && accept[0] == REST.type.TEXT_PLAIN) ) {
		REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
		snprintf((char*)buffer, REST_MAX_CHUNK_SIZE, "%d.%02u", battery_sensor.value(0)/1000, battery_sensor.value(0)-((battery_sensor.value(0)/1000)*1000));

		REST.set_response_payload(response, (uint8_t *)buffer, strlen((char*)buffer));
	}
	else if ( num && (accept[0] == REST.type.APPLICATION_JSON) ) {
		REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
		snprintf((char*)buffer, REST_MAX_CHUNK_SIZE, "{'battery':%d.%02u}", battery_sensor.value(0)/1000, battery_sensor.value(0)-((battery_sensor.value(0)/1000)*1000));

		REST.set_response_payload(response, buffer, strlen((char*)buffer));
	}
	else
	{
		REST.set_response_status(response, REST.status.UNSUPPORTED_MADIA_TYPE);
		REST.set_response_payload(response, (uint8_t *)"Supporting content-types text/plain and application/json", 56);
	}
}
#endif /* PLATFORM_HAS_BATT_SENSOR */

/*
 *
 */
PROCESS(rest_server, "RESTserver");
AUTOSTART_PROCESSES(&rest_server);

/*
 *
 */
PROCESS_THREAD(rest_server, ev, data) {
	PROCESS_BEGIN();

	PRINTF("RESTserver\n");

#ifdef RF_CHANNEL
	PRINTF("RF channel: %u\n", RF_CHANNEL);
#endif
#ifdef IEEE802154_PANID
	PRINTF("PAN ID: 0x%04X\n", IEEE802154_PANID);
#endif

	PRINTF("uIP buffer: %u\n", UIP_BUFSIZE);
	PRINTF("LL header: %u\n", UIP_LLH_LEN);
	PRINTF("IP+UDP header: %u\n", UIP_IPUDPH_LEN);
	PRINTF("REST max chunk: %u\n", REST_MAX_CHUNK_SIZE);

	/* Initialize the REST framework. */
	rest_init_engine();

	/* Activate the platform-specific resources. */
	rest_activate_resource(&resource_root);
	rest_activate_resource(&resource_reboot);
	rest_activate_resource(&resource_processes);

//	rest_activate_periodic_resource(&periodic_resource_uptime);

	rest_activate_resource(&resource_led);
	rest_activate_resource(&resource_toggle);

#if PLATFORM_HAS_BATT_SENSOR
	SENSORS_ACTIVATE(battery_sensor);
	rest_activate_resource(&resource_battery);
#endif

	/*
	 * Activate application-specific resources here.
	 */

	leds_on(0);

	while (1) {

		PROCESS_WAIT_EVENT();

		if (ev == sensors_event) {

		}
	} /* while (1) */

	PROCESS_END();
}

/* eof */
