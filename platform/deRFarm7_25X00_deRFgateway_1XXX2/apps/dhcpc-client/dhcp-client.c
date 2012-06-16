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
 * 			dhcp-client.c
 * \brief
 * 			DHCP client Contiki process
 * \author
 * 			Luis Maqueda Ara <luis@sen.se>
 */

#include "contiki-net.h"

#include "net/uipv4/uipv4.h"
#include "net/uipv4/dhcpc.h"

#include "dhcp-client.h"

PROCESS(dhcp_process, "DHCP client");

/*
 *
 */
PROCESS_THREAD( dhcp_process, ev, data )
{
  PROCESS_BEGIN();
		printf("dhcp_process started\n");printf("\n");

		dhcpc_init(uip_ethaddr.addr, sizeof(uip_ethaddr.addr));
		dhcpc_request();

		while ( 1 ) {
			PROCESS_WAIT_EVENT();
			if ( ev == tcpipv4_event || ev == PROCESS_EVENT_TIMER ) {
				dhcpc_appcall(ev, data);
			}
		}

	PROCESS_END();
}

/*-----------------------------------------------------------------------------------*/
void
dhcpc_configured( const struct dhcpc_state *s )
{
  uipv4_sethostaddr(&s->ipaddr);
  uipv4_setnetmask(&s->netmask);
  uipv4_setdraddr(&s->default_router);
}

/*-----------------------------------------------------------------------------------*/
void
dhcpc_unconfigured( const struct dhcpc_state *s )
{
  uipv4_sethostaddr(&uipv4_all_zeroes_addr);
  uipv4_setnetmask(&uipv4_all_zeroes_addr);
  uipv4_setdraddr(&uipv4_all_zeroes_addr);
}
/*--------------- eof --------------------------------------------------------------------*/
