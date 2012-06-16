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
 * 			pgw_fwd.h
 *
 * \brief		
 * 			Forwarding/bridging-related definitions for the 6LoWPAN-ND proxy-gateway 
 *
 * \author		
 * 			Luis Maqueda Ara <luis@sen.se>
 */

#ifndef PGW_FWD_H_
#define PGW_FWD_H_

#include "net/p-gw/pgw.h"
#include "net/p-gw/pgw_nd.h"
#include "contiki.h"
#include "contiki-net.h"


/* Number of entries in the bridge */
#define MAX_BRIDGE_ENTRIES	30

/* 
 * Interface types 
 */
typedef enum {
	UNDEFINED = 0,
	IEEE_802_3,
	IEEE_802_15_4,
	LOCAL
} interface_t;

/* 
 * An entry in the bridge cache 
 */
typedef struct {
	interface_t interface;
	eui64_t addr;
} bridge_entry_t;

typedef struct {
	bridge_entry_t table[MAX_BRIDGE_ENTRIES];
	u8_t elems;
} bridge_table_t;

/** \brief Incoming and outgoing interfaces */
extern interface_t incoming_if, outgoing_if;
/** \brief Source and destination MAC addresses */
extern eui64_t src_eui64, dst_eui64;


/* Packet forwarding and address translation functions */
void pgw_fwd_init(void);
void pgw_fwd_input(void);
void pgw_fwd_output(eui64_t* src, eui64_t* dst);

#endif /*PGW_FWD_H_*/
