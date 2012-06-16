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
 *
 * \file
 * 			eth_driver.c
 * \brief
 * 			This file implements the ethernet driver needed for the 6LP-GW
 * \author
 * 			Luis Maqueda Ara <luis@sen.se>
 *
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 * 			(Adapted to more general lower layer ethernet device)
 */

#include "dev/eth_driver.h"

/*
 * We include the "contiki-net.h" file to get all the network functions.
 */
#include "contiki-net.h"

#include "net/pgw_netstack.h"	// TODO @@@jwg

/*
 * Include lower level driver functions.
 */
#include "ethernet/ethdev.h"

/*
 * And "pgw_fwd.h", which holds the interface data structures.
 */
#include "net/p-gw/pgw_fwd.h"

/* The driver state */
static eth_driver_state_t eth_state = ETH_DRIVER_OFF;

static void init(void);
static void send(const void *payload, unsigned short payload_len);
static int read(const void *payload, unsigned short payload_len);
static int pending_packet(void);
static void on(void);
static void off(void);

/*---------------------------------------------------------------------------*/
/*
 * We declare the process that we use to register with the TCP/IP stack,
 * and to check for incoming packets.
 */
PROCESS(eth_driver_process, "eth_driver_process");
/*---------------------------------------------------------------------------*/
/*
 * This is the poll handler function in the process below. This poll handler
 * function checks for incoming packets and forwards them to the right 
 * interface or delivers them to the TCP/IP stack.
 */
static void
pollhandler(void)
{
	process_poll(&eth_driver_process);
#if 1
	if ( eth_state == ETH_DRIVER_ON ) {
		if ( (uip_len = ethdev_read()) > 0 ) {
			/* Set current incoming interface */
			incoming_if = IEEE_802_3;
//			printf("eth0 rcvd: %d bytes -> ", uip_len);

			/*
			 * Forward the packet to the upper level in the stack
			 */
			NETSTACK_MAC_ETH.input();
		}
	}
#else
	if (pending_packet()) {
		/* Set current incoming interface */
		incoming_if = IEEE_802_3;
		/* Read packet */
		uip_len = read(uip_buf, UIP_BUFSIZE);
		/* 
   	 * Forward the packet to the upper level in the stack
   	 */
  	NETSTACK_MAC_ETH.input();
	}
#endif
}

/*---------------------------------------------------------------------------*/
/*
 * Finally, we define the process that does the work. 
 */
PROCESS_THREAD(eth_driver_process, ev, data)
{
	/*
	 * This process has a poll handler, so we declare it here. Note that
	 * the PROCESS_POLLHANDLER() macro must come before the PROCESS_BEGIN()
   * macro.
   */
	PROCESS_POLLHANDLER(pollhandler());
	
//	/*
//	 * This process has an exit handler, so we declare it here. Note that
//   * the PROCESS_EXITHANDLER() macro must come before the PROCESS_BEGIN()
//   * macro.
//   */
//	PROCESS_EXITHANDLER(_nop());

  /*
   * The process begins here.
   */
  PROCESS_BEGIN();
printf("eth_driver_process started\n");
printf("\n");
  /*
   * Now we'll make sure that the poll handler is executed initially. We do
   * this by calling process_poll() with this process as its argument.
   */
  process_poll(&eth_driver_process);

  /*
   * And we wait for the process to exit.
   */
	PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXIT);

	off();	// TODO @@@jwg exit!?
  /*
   * Here ends the process.
   */
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static void 
init() {
	ethdev_init(uip_lladdr.addr /*rimeaddr_node_addr.u8*/ /*uint8_t* macaddr*/);	// TODO @@@jwg: MAC addr!!!!
	on();
	process_start(&eth_driver_process, NULL);
}

/*---------------------------------------------------------------------------*/
static void
send( const void *payload, unsigned short payload_len )
{
	if ( eth_state == ETH_DRIVER_ON ) {
		ethdev_send();	// Expects payload in uipbuf which is already there!
	}
}

static int
read( const void *payload, unsigned short payload_len )
{
	if ( eth_state == ETH_DRIVER_ON ) {
		return ((int)ethdev_read());	// TODO @@@jwg: read payload is in uipbuf
	} else {
		return(0);
	}
}

static int
pending_packet()
{
	if (eth_state == ETH_DRIVER_ON) {
		return(1/*enc28j60_pending_packet()*/);	// FIXME @@@jwg: implement ethdev_pending_packet()!
	} else {
		return 0;
	}
}

static void
on()
{
	eth_state = ETH_DRIVER_ON;
}

static void
off()
{
	eth_state = ETH_DRIVER_OFF;
}


const eth_driver_t eth_driver =
  {
  	init,
  	send,
  	read,
  	pending_packet,
  	on,
  	off
  };

/*--------------------------------- eof ------------------------------------------*/
