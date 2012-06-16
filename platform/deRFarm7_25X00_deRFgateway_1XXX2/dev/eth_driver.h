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
 * 			eth_driver.h
 * \brief		
 * 			Includes for the ethernet driver needed for the 6LP-GW
 * \author		
 * 			Luis Maqueda Ara <luis@sen.se>
 *
 */

#ifndef ETH_DRIVER_H_
#define ETH_DRIVER_H_

#include "sys/process.h"

/* Driver state */
typedef enum {
	ETH_DRIVER_ON, ETH_DRIVER_OFF
} eth_driver_state_t;

/**
 * The structure of a device driver for an ethernet controller in Contiki.
 */
typedef struct {
	
	/** Initialize the driver */
  void (* init)(void);
  
  /** Send a packet. */
  void (* send)(const void *payload, unsigned short payload_len);

  /** Read a received packet into a buffer. */
  int (* read)(const void *buf, unsigned short buf_len);

    /** Check if the radio driver has just received a packet */
  int (* pending_packet)(void);

  /** Turn the driver on. */
  void (* on)(void);

  /** Turn the driver off. */
  void (* off)(void);
} eth_driver_t;


const extern eth_driver_t eth_driver;

PROCESS_NAME(eth_driver_process);

#endif /*ETH_DRIVER_H_*/
