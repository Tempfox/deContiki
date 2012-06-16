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
 *         Contiki low-layer network stack (NETSTACK)
 * \author
 *         Luis Maqueda Ara <luis@sen.se>
 */
#include "net/pgw_netstack.h"

#include "radio/rf231bb/rf231bb.h"
#include "params.h"

extern const struct network_ipv6_driver ipv6_driver;	// TODO @@@jwg

static rimeaddr_t radio_macLongAddr;

void
pgw_netstack_init(void)
{
	NETSTACK_ETHERNET.init();
	NETSTACK_MAC_ETH.init();

	NETSTACK_RADIO.init();
	memset(&radio_macLongAddr, 0, sizeof(rimeaddr_t));

//	if(get_eui64_from_eeprom(tmp_addr.u8));
#if 1
	params_get_radio_eui64(radio_macLongAddr.u8);
#else
	radio_macLongAddr.u8[0] = 0x02;
	radio_macLongAddr.u8[1] = 0x11;
	radio_macLongAddr.u8[2] = 0x22;
	radio_macLongAddr.u8[3] = 0xFF;
	radio_macLongAddr.u8[4] = 0xFE;
	radio_macLongAddr.u8[5] = 0x77;
	radio_macLongAddr.u8[6] = 0x88;
	radio_macLongAddr.u8[7] = 0x77;
#endif

	rf231_set_pan_addr(params_get_panid(),			// 0xABCD
					   params_get_panaddr(),		// 0x0
					   (uint8_t*)&radio_macLongAddr.u8
					  );
//	rf231_set_channel(params_get_channel()); 		//25
	rf231_set_txpower(params_get_txpower());		// 0

	NETSTACK_MAC_RADIO.init();
	
	NETSTACK_6LOWPAN.init();

	rf231_listen_channel(params_get_channel());		// 25
	rf231_set_promiscuous_mode(1);

	NETSTACK_NETWORK_IPV4.init();

	NETSTACK_NETWORK_IPV6.init();
	NETSTACK_6LPGW.init();
}
