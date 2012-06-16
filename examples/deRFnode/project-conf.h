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
 *
 */

#ifndef __PROJECT_CONF_DERFNODE_H__
#define __PROJECT_CONF_DERFNODE_H__


#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM       6
#endif

/* Increase rpl-border-router IP-buffer when using 128 */
#ifndef REST_MAX_CHUNK_SIZE
#define REST_MAX_CHUNK_SIZE    48 // TODO @@@jwg orig: 64
#endif

/* Decrease to 2 if no space left for stack when using 128-byte chunks */
#ifndef COAP_MAX_OPEN_TRANSACTIONS
#define COAP_MAX_OPEN_TRANSACTIONS   4 
#endif

/* Must be <= open transaction number */
#ifndef COAP_MAX_OBSERVERS
#define COAP_MAX_OBSERVERS      COAP_MAX_OPEN_TRANSACTIONS
#endif


#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  60
#endif

#ifndef WEBSERVER_CONF_CFS_CONNS
#define WEBSERVER_CONF_CFS_CONNS 2
#endif



/* Now we define which one from the above compression methods we are going to
 * use. */
#define SICSLOWPAN_CONF_COMPRESSION           	SICSLOWPAN_COMPRESSION_HC06

/*
 * We define the maximum number of context to use in 6LoWPAN IPHC
 */
//#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS		2

/*
 * 6LoWPAN fragmentation support
 */
//#define SICSLOWPAN_CONF_FRAG                    0

/*
 * Does the local-host behave as router?
 */
#define UIP_CONF_ROUTER							0

/*
 * Does the local-host use a manually-assigned, fixed address?
 */
#define UIP_FIXEDADDR							0

/*
 * Node's MAC address
 */
//#define NODE_BASE_ADDR0	0x00
//#define NODE_BASE_ADDR1 0x07
//#define NODE_BASE_ADDR2 0x62
//#define NODE_BASE_ADDR3 0xff
//#define NODE_BASE_ADDR4 0xfe

/*
 * Do optional option filtering?
 */
#define CONF_OPT_FILTERING 	1

/*
 * Use 6LoWPAN-ND IN HOST? (We use instead traditional IPv6-ND)
 */
#define CONF_6LOWPAN_ND		0

/*
 * Do we implement the 6LoWPAN Context option?
 */
#define CONF_6LOWPAN_ND_6CO		1

/*
 * Do we implement the 6LoWPAN Authoritative Border Router option?
 */
#define CONF_OPT_ABRO		0

/*
 * State-dependent NCEs' lifetime definitions
 */
#define GARBAGE_COLLECTIBLE_NCE_LIFETIME	600 /* 10 minutes */
#define TENTATIVE_NCE_LIFETIME				20 /* 20 seconds */

/*
 * 6LP-GW definitions
 */

/*
 * Maximum number of allowed neighbors
 */
#define MAX_6LOWPAN_NEIGHBORS	25


/*
 * Define the Ethernet header length
 */
//#define UIP_CONF_LLH_LEN						14

/* IPv6 minumum MTU is 1280 bytes. Ethernet header (14) is placed on the same
 * buffer, which defines our maximum buffer size as 1280 + 14 */
//#define UIP_CONF_BUFFER_SIZE					128


#endif /* __PROJECT_CONF_DERFNODE_H__ */
