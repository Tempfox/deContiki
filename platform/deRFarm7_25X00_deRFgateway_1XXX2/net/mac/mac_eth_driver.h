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
 *         Includes for ethernet MAC file.
 *
 * \author
 *         Luis Maqueda Ara <luis@sen.se>
 */

#ifndef MAC_ETH_DRIVER_H_
#define MAC_ETH_DRIVER_H_

#include "contiki-net.h"

/**
 * The structure of an Ethernet MAC driver in Contiki.
 */
struct mac_eth_driver {
  char *name;

  /** Initialize the Ethernet MAC driver */
  void (* init)(void);

  /** Send a packet from the uip_buf buffer  */
  void (* send)(void);

  /** Callback for getting notified of incoming packet. */
  void (* input)(void);

  /** Turn the MAC layer on. */
  void (* on)(void);

  /** Turn the MAC layer off. */
  void (* off)(void);

};

extern const struct mac_eth_driver mac_eth_driver;

PROCESS_NAME(mac_eth_process);

#endif /*MAC_ETH_DRIVER_H_*/
