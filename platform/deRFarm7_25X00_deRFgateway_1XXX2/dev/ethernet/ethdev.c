/*
 * Copyright (c) 2012, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
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
 * \author
 * 			Joerg Wolf <gwynpen@googlemail.com>
 *
 */

#include "contiki-net.h"
#include "net/uip-neighbor.h"

#include <board.h>
#include <AT91SAM7X512.h>

#include <pio/pio.h>
#include <aic/aic.h>
#include <rstc/rstc.h>
#include <dbgu/dbgu.h>
#include <usart/usart.h>

#include <emac/emac.h>
#include <ethernet/dp83848.h>

#include <utility/trace.h>
#include <utility/assert.h>

#include <dev/leds.h>

#include "ethdev.h"

// EMAC power control pin
#if !defined(BOARD_EMAC_POWER_ALWAYS_ON)
static const Pin emacPwrDn[] = {BOARD_EMAC_PIN_PWRDN};
#endif

// The PINs on PHY reset
static const Pin emacRstPins[] = {BOARD_EMAC_RST_PINS};

// The PINs for EMAC
static const Pin emacPins[] = {BOARD_EMAC_RUN_PINS};

// PHY address
#define EMAC_PHY_ADDR 3

// The DP83848 driver instance
static Dp83848 gDp83848;

/*
 * Emac interrupt handler
 */
static void ISR_Emac( void )
{
    EMAC_Handler();
}

/*---------------------------------------------------------------------------*/
/*
 * Init the ethernet device
 */
void
ethdev_init( uint8_t* macaddr )
{
    Dp83848 *pDp = &gDp83848;

	/* Disable pull up on RXDV => PHY normal mode (not in test mode),
		PHY has internal pull down. */
	AT91C_BASE_PIOB->PIO_PPUDR &= ~AT91C_PIO_PB15;

	// clear PHY power down mode
    PIO_Configure(emacPwrDn, 1);
    PIO_Set(emacPwrDn);

    // Enable external reset
    RSTC_SetUserResetEnable(/*AT91C_BASE_RSTC, TODO @@@jwg: wo kam das her!?*/ 1);

    // Init DP83848 driver
    DP83848_Init(pDp, EMAC_PHY_ADDR);

    // PHY initialize
	if ( !DP83848_InitPhy(pDp, BOARD_MCK, emacRstPins, PIO_LISTSIZE(emacRstPins),
										  emacPins, PIO_LISTSIZE(emacPins)) ) {
		TRACE_ERROR("PHY Initialize ERROR!\n\r");
		return;
	}

	// Setup EMAC buffers and interrupts
	AIC_ConfigureIT(AT91C_ID_EMAC, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, ISR_Emac);
	AIC_EnableIT(AT91C_ID_EMAC);

	// Init EMAC driver structure
	EMAC_Init(AT91C_ID_EMAC, macaddr, EMAC_CAF_ENABLE, EMAC_NBC_DISABLE);

	DP83848_Setup(pDp);

	TRACE_INFO("ethdev_init() done\n\r");

//	LED_Set(2);
}

/*
 * Read from EMAC device
 */
unsigned int
ethdev_read( void )
{
    unsigned int pkt_len = 0;

    if ( EMAC_RX_OK != EMAC_Poll( (unsigned char*)uip_buf,
//    if( EMAC_RX_OK != EMAC_GetFrame((unsigned char*)ll_header,
//    								(unsigned char*)uip_buf,
                                     UIP_CONF_BUFFER_SIZE,
                                    &pkt_len) ) {

        pkt_len = 0;
    }
//    else {
//    	/* subtract the 4-byte CRC and Link-layer header lengths */
//    	pkt_len -= UIP_CONF_LLH_LEN;
//    }

    return(pkt_len);
}

/*
 * Send to EMAC device
 */
void ethdev_send(void)
{
    unsigned char emac_rc;

    TRACE_DEBUG("eth0: sent %d bytes\n", uip_len);

    emac_rc = EMAC_Send( (void*)uip_buf, uip_len, (EMAC_TxCallback)0);

    if ( emac_rc != EMAC_TX_OK ) {
        TRACE_ERROR("Send, rc 0x%x\n\r", emac_rc);
    }
}

/*--------------- eof ------------------------------------------------------------*/
