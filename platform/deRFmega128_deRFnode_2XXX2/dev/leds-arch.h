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
 * This file is part of the Configurable Sensor Network Application
 * Architecture for sensor nodes running the Contiki operating system.
 *
*/
/**
 * \file
 * 			Hardware specific defines for deRFnode boards with ATmeag128RFA1
 * 			by Dresden Elektronik <http://www.dresden-elektronik.de>
 *
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 *
 */

#ifndef LEDSARCH_H_
#define LEDSARCH_H_
/*
 * PORT where LEDs are connected
 */
#define LED0_PORT                       (PORTG) /**< Port for LED0 */
#define LED0_PORT_DIR                   (DDRG)  /**< Port directions for LED0 */

#define LED_PORT                        (PORTE) /**< Port for LEDs */
#define LED_PORT_DIR                    (DDRE)  /**< Port directions for LEDs */

/*
 * PINs where LEDs are connected
 */
#define LED_PIN_0                       (PG5)   /**< LED 0 pin */
#define LED_PIN_1                       (PE3)   /**< LED 1 pin */
#define LED_PIN_2                       (PE4)   /**< LED 2 pin */


#endif /* LEDSARCH_H_ */
