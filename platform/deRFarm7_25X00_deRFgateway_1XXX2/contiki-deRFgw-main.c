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
 *
 */
/**
 * \file
 * 			contiki-deRFw-main.c
 *
 * \brief
 * 			6LP-GW main file on deRFgateway platform.
 *
 * \author
 * 			Luis Maqueda Ara <luis@sen.se>
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 */

#include "contiki.h"
#include "contiki-conf.h"

#include "sys/process.h"
#include "sys/procinit.h"
#include "contiki-net.h"

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <dev/leds.h>
#include <dbgu/dbgu.h>
#include <utility/trace.h>

// #include "clock_arch.h"		TODO @@@jwg: not used?
#include "net/p-gw/pgw.h"
#include "net/uipv4/uipv4.h"

#include "params.h"

#include "dhcpc-client/dhcp-client.h"
#include "REST-server.h"
// #include "udp-server/udp-server.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#endif

void
uip_log( char*msg ) {
	printf("%s%s", msg, "\n");
}

uip_ipaddr_t myPrefix;	// FIXME @@@jwg just a work around

/*-------------------------Low level initialization------------------------*/
/*------Done in a subroutine to keep main routine stack usage small--------*/
void initialize(void) {
	//	watchdog_init();  TODO @@@jwg
	//	watchdog_start(); TODO @@@jwg

	// configure the Debug Unit for console printout
	TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);

	// globally enable interrupts
	PIO_InitializeInterrupts(AT91C_AIC_PRIOR_LOWEST);

	leds_init();

	leds_on(LEDS_GREEN);

	clock_init();

	printf("\n*** Booting %s (%s %s) ***\n", CONTIKI_VERSION_STRING, __DATE__, __TIME__);

#if 0 //moved back to main()
	/* rtimers needed for radio cycling */
	rtimer_init();

	/* Initialize process subsystem */
	process_init();
	/* etimers must be started before ctimer_init */
	process_start(&etimer_process, NULL);

	ctimer_init();
#endif

}

/*---------------------------------------------------------------------------*/
int
main( void )
{

	initialize();

	/* initialize uip variables */
	memset(uip_buf, 0, UIP_CONF_BUFFER_SIZE);
	uip_len = 0;

	/* Create MAC address.
	 * We use rime addresses in order to use useful rime address handling
	 * functions.
	 * The sicslowmac layer requires the MAC address to be placed in the global
	 * variable rimeaddr_node_addr (declared in rimeaddr.h). */
#if 1
	params_get_eth_eui64(rimeaddr_node_addr.u8);
#else
	rimeaddr_node_addr.u8[0] = 0x02;
	rimeaddr_node_addr.u8[1] = 0x12;
	rimeaddr_node_addr.u8[2] = 0x13;
	rimeaddr_node_addr.u8[3] = 0xff;
	rimeaddr_node_addr.u8[4] = 0xfe;
	rimeaddr_node_addr.u8[5] = 0xed;
	rimeaddr_node_addr.u8[6] = 0xbe;
	rimeaddr_node_addr.u8[7] = 0xef;
#endif

	/* Temporary hack: expected ipv6 prefix */
	uip_ip6addr(&myPrefix, 0x2222, 0x3333, 0x4444, 0, 0, 0, 0, 0);	// FIXME @@@jwg

	/* The following line sets the uIP's link-layer address. This must be done
	 * before the tcpip_process is started since in its initialization
	 * routine the function uip_netif_init() will be called from inside
	 * uip_init()and there the default IPv6 address will be set by combining
	 * the link local prefix (fe80::/64)and the link layer address. */
	rimeaddr_copy((rimeaddr_t*) &uip_lladdr.addr, &rimeaddr_node_addr);

	/* For the IPv4 stack we need an Ethernet address in uip_ethaddr */
#if 1
	params_get_eth_addr(uip_ethaddr.addr);
#else
	uip_ethaddr.addr[0] = 0x02;
	uip_ethaddr.addr[1] = 0x12;
	uip_ethaddr.addr[2] = 0x13;
	uip_ethaddr.addr[3] = 0xff;
	uip_ethaddr.addr[4] = 0xfe;
	uip_ethaddr.addr[5] = 0xed;
#endif

	/* rtimers needed for radio cycling */
	rtimer_init();

	/* Initialize the process module */
	process_init();

	/* etimers must be started before ctimer_init */
	process_start(&etimer_process, NULL);

	ctimer_init();

	/* Initialize stack protocols */
	pgw_netstack_init();

#if 1
	/* Initialize the DHCP client */
	process_start(&dhcp_process, NULL);
	process_start(&rest_server, NULL);
#else
	/* Autostart other processes */
	autostart_start(autostart_processes);  // FIXME @@@jwg: autostart makes the board hang!
#endif

	  /* Enter main loop */
	while ( 1 ) {
		/* poll every running process which has requested to be polled */
		process_run();
		/* etimer_request_poll() called in timer interrupt routine */
	}
	return (0);
}
/* eof */
