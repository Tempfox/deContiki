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
 * 			radio_driver.c
 *
 * \brief
 * 			This file implements the radio driver needed for the 6LP-GW
 *
 * \author
 * 			Luis Maqueda Ara <luis@sen.se>
 *
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 * 			Adapted to rf231
 */
/*
 *
 */
#include "dev/radio_driver.h"

/*
 * We include the "contiki-net.h" file to get all the network functions.
 */
#include "contiki-net.h"

#include "rf231bb.h"

/*
 * And "pgw_fwd.h", which holds the interface data structures.
 */
#include "net/p-gw/pgw_fwd.h"

#include "net/pgw_netstack.h"

static int init(void);
static int send(const void *payload, unsigned short payload_len);
static int read(void *buf, unsigned short buf_len);
static int pending_packet(void);
static int on(void);
static int off(void);

/* The driver state */
static radio_driver_state_t radio_state = OFF;

static rimeaddr_t radio_macLongAddr;

/*---------------------------------------------------------------------------*/
/*
 * We declare the process that we use to register with the TCP/IP stack,
 * and to check for incoming packets.
 */
PROCESS(radio_driver_process, "radio_driver_process");

/*---------------------------------------------------------------------------*/
/*
 * This is the poll handler function in the process below. This poll handler
 * function checks for incoming packets and forwards them to the right 
 * interface or delivers them to the TCP/IP stack.
 */
static void pollhandler(void) {
	if (rf231_pending_packet()) {

		incoming_if = IEEE_802_15_4;

		packetbuf_clear();
		packetbuf_set_datalen(read(packetbuf_dataptr(), PACKETBUF_SIZE));
		/* 
		 * Forward the packet to the upper level in the stack
		 */
		NETSTACK_MAC_RADIO.input();
	}
	/*
	 * Now we'll make sure that the poll handler is executed repeatedly.
	 * We do this by calling process_poll() with this process as its
	 * argument.
	 */
	process_poll(&radio_driver_process);
}

/*---------------------------------------------------------------------------*/
/*
 * Finally, we define the process that does the work. 
 */PROCESS_THREAD(radio_driver_process, ev, data) {
	/*
	 * This process has a poll handler, so we declare it here. Note that
	 * the PROCESS_POLLHANDLER() macro must come before the PROCESS_BEGIN()
	 * macro.
	 */
	PROCESS_POLLHANDLER(pollhandler());

	/*
	 * This process has an exit handler, so we declare it here. Note that
	 * the PROCESS_EXITHANDLER() macro must come before the PROCESS_BEGIN()
	 * macro.
	 */
	PROCESS_EXITHANDLER(_nop());

	/*
	 * The process begins here.
	 */PROCESS_BEGIN()
		;

		printf("radio_driver_process started\n");
		printf("\n");

		/*
		 * Now we'll make sure that the poll handler is executed initially. We do
		 * this by calling process_poll() with this process as its argument.
		 */
		process_poll(&radio_driver_process);

		/*
		 * And we wait for the process to exit.
		 */
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXIT);

		/*
		 * Here ends the process.
		 */
	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static int init() {
int ret = 0;

if (rf231_init()) {
	on();
	process_start(&radio_driver_process, NULL);

	memset(&radio_macLongAddr, 0, sizeof(rimeaddr_t));

	//	if(get_eui64_from_eeprom(tmp_addr.u8));
	radio_macLongAddr.u8[0] = 0x02;
	radio_macLongAddr.u8[1] = 0x11;
	radio_macLongAddr.u8[2] = 0x22;
	radio_macLongAddr.u8[3] = 0xFF;
	radio_macLongAddr.u8[4] = 0xFE;
	radio_macLongAddr.u8[5] = 0x77;
	radio_macLongAddr.u8[6] = 0x88;
	radio_macLongAddr.u8[7] = 0x77;

	rf231_set_pan_addr(0xABCD /*get_panid_from_eeprom()*/,
			0x0 /*get_panaddr_from_eeprom()*/,
			(uint8_t *) &radio_macLongAddr.u8);
	rf231_set_channel(25 /*get_channel_from_eeprom()*/);
	rf231_set_txpower(0 /*get_txpower_from_eeprom()*/);

	ret = 1;
}

return (ret);
}

/*---------------------------------------------------------------------------*/
static int send(const void *payload, unsigned short payload_len) {
	if ( radio_state == ON ) {
		return (rf231_send(payload, payload_len)); // ORIG
//		return(rf231_send(payload, payload_len + 2));	// TODO @@@jwg!!!
	}

	return (0);
}

static int read(void *buf, unsigned short buf_len) {
	int ret = 0;

	if ( radio_state == ON ) {
		/* substract CRC length */
		ret = rf231_read(buf, buf_len) - 2; // TODO @@@jwg: substract 2 bytes CRC????
	}

	return (ret);
}

static int pending_packet() {
	if ( radio_state == ON ) {
		return (rf231_pending_packet());
	}
	return (0);
}

static int on() {
	radio_state = ON;
	return(1);
}

static int off() {
	radio_state = OFF;
	return(1);
}

/* These functions are not used; They are defined only for compliance with 
 * struct radio_driver defined in radio.h*/
int prepare(const void *payload, unsigned short payload_len) {
	return(1);
}

int transmit(unsigned short transmit_len) {
	return(1);
}

int channel_clear(void) {
	return(1);
}

int receiving_packet(void) {
	return(0);
}

const struct radio_driver radio_driver = {
		init,
		prepare,
		transmit,
		send,
		read,
		channel_clear,
		receiving_packet,
		pending_packet,
		on,
		off
};

/*-------------------------- eof -------------------------------------------------*/
