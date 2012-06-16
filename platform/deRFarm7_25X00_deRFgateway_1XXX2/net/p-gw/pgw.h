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
 * 			pgw.h
 *
 * \brief			
 * 			Set of functions needed for the 6LP-GW operation 
 *
 * \author		
 * 			Luis Maqueda Ara <luis@sen.se>
 */

#ifndef PGW_H_
#define PGW_H_

#include "contiki.h"
#include "contiki-net.h"
#include "net/rime/rimeaddr.h" 

// FIXME @@@jwg #define NULL 0

/* Time between periodic processings */
#define PGW_PERIOD (CLOCK_SECOND/10)
/* Maximum number of contexts */
#define PGW_CONF_MAX_ADDR_CONTEXTS 16
/* Minimum delay between context changes (I-D.ietf.6lowpan-nd) */
#define PGW_MIN_CONTEXT_CHANGE_DELAY 300 /* seconds */
/* Initial lifetime (in seconds) to be advertised in 6CO options. During this period, the 
 * context is in IN_USE_UNCOMPRESS_ONLY state. Thus, this time should as short as possible,
 * but long enough so that all nodes in the network are aware of the context before changing
 * to IN_USE_COMPRESS state */
#ifdef PGW_CONF_INITIAL_CONTEXT_LIFETIME
#define PGW_INITIAL_CONTEXT_LIFETIME			PGW_CONF_INITIAL_CONTEXT_LIFETIME
#else 
#define PGW_INITIAL_CONTEXT_LIFETIME			600		/* seconds */
#endif /* PGW_CONF_INITIAL_CONTEXT_LIFETIME */

#ifdef PGW_CONF_CONTEXT_LIFETIME
#define PGW_CONTEXT_LIFETIME							PGW_CONF_CONTEXT_LIFETIME
#else
#define PGW_CONTEXT_LIFETIME							3600	/* seconds */
#endif /* PGW_CONF_CONTEXT_LIFETIME */

/* The number of NS messages to be sent for DAD */
#define PGW_MAX_DAD_NS		1

/* Traffic filters */
#define CONF_FILTER_TCP 1
#define CONF_FILTER_PIM 1
#define CONF_FILTER_MLQ 1
#define CONF_FILTER_MLR 1
#define CONF_FILTER_MLR2 1

/*
 * The bridge cache will store only 64 bit addresses. We redefine the
 * 64-bit long address.
 */
typedef rimeaddr_t eui64_t;

#define create_eui64_based_ipaddr(a, m) do {	\
	uip_create_linklocal_prefix(a);				\
    (((a)->u8[8])  = (((m)->u8[0]) ^ 0x02));   	\
    (((a)->u8[9])  = ((m)->u8[1]));	            \
    (((a)->u8[10]) = ((m)->u8[2]));	            \
    (((a)->u8[11]) = ((m)->u8[3]));	            \
    (((a)->u8[12]) = ((m)->u8[4]));	            \
    (((a)->u8[13]) = ((m)->u8[5]));	            \
    (((a)->u8[14]) = ((m)->u8[6]));	            \
    (((a)->u8[15]) = ((m)->u8[7]));				\
} while(0)

#define eui64_cmp(a,b) rimeaddr_cmp((rimeaddr_t*)a,(rimeaddr_t*)b)
#define eui64_copy(dst, src) rimeaddr_copy((rimeaddr_t*)dst, (rimeaddr_t*)src)

/** \briefIPv6 regular router's IPv6 address */
extern uip_ipaddr_t rr_ipaddr;
/** \briefIPv6 regular router's EUI-64 address */
extern eui64_t rr_lladdr;
/** Useful flags */
extern u8_t ra_pending, context_chaged;

/*
 * External functions
 */

void pgw_init(void);
void local_node_output(uip_lladdr_t *localdest);

/* 6LP-GW "driver" datastructre */

struct pgw_driver {
  char *name;

  /** Initialize the network driver */
  void (* init)(void);

  /** Callback for getting notified of incoming packet. */
  void (* input)(void);
  
  /** Output function */
  void (* output)(uip_lladdr_t *localdest);
  
};

/* 6LP-GW "driver" */
extern const struct pgw_driver proxy_gateway_driver;

/* 6LP-GW process */
PROCESS_NAME(pgw_process);

#endif /*PGW_H_*/
