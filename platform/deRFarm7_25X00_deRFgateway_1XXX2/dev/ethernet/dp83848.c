/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Derived from dm9161 code and adapted to DP83848 by: Joerg Wolf <gwynpen@googlemail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

// components/ethernet/dp83848/dp83848.c

//-----------------------------------------------------------------------------
//         Headers
//-----------------------------------------------------------------------------
#include "dp83848.h"
#include "dp83848_define.h"
#include <pio/pio.h>
#include <rstc/rstc.h>
#include <emac/emac.h>
#include <utility/trace.h>
#include <utility/assert.h>

//-----------------------------------------------------------------------------
//         Definitions
//-----------------------------------------------------------------------------

/// Default max retry count
#define DP83848_RETRY_MAX            100000

#define emacRESET_KEY				( ( unsigned long ) 0xA5000000 )
#define emacRESET_LENGTH			( ( unsigned long ) ( 0x01 << 8 ) )

/* Peripheral setup for the EMAC. */
#define emacPERIPHERAL_A_SETUP 		( ( unsigned long ) AT91C_PB2_ETX0			) | \
									( ( unsigned long ) AT91C_PB3_ETX1			) | \
									( ( unsigned long ) AT91C_PB5_ERX0			) | \
									( ( unsigned long ) AT91C_PB6_ERX1			) | \
									( ( unsigned long ) AT91C_PB1_ETXEN			) | \
									( ( unsigned long ) AT91C_PB7_ERXER			) | \
									( ( unsigned long ) AT91C_PB15_ERXDV_ECRSDV	) | \
									( ( unsigned long ) AT91C_PB8_EMDC			) | \
									( ( unsigned long ) AT91C_PB9_EMDIO			);

//-----------------------------------------------------------------------------
/// Dump all the useful registers
/// \param pDp          Pointer to the Dm9161 instance
//-----------------------------------------------------------------------------
static void DP83848_DumpRegisters( Dp83848 * pDp )
{
    unsigned char phyAddress;
    unsigned int retryMax;
    unsigned int value;

    TRACE_INFO("DP83848_DumpRegisters\n\r");
    ASSERT(pDp, "F: DP83848_DumpRegisters\n\r");

    EMAC_EnableMdio();
    phyAddress = pDp->phyAddress;
    retryMax = pDp->retryMax;

    TRACE_INFO("DP83848 (%d) Registers:\n\r", phyAddress);

    EMAC_ReadPhy(phyAddress, DP83848_BMCR, &value, retryMax);
    TRACE_INFO(" _BMCR   : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_BMSR, &value, retryMax);
    TRACE_INFO(" _BMSR   : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_ANAR, &value, retryMax);
    TRACE_INFO(" _ANAR   : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_ANLPAR, &value, retryMax);
    TRACE_INFO(" _ANLPAR : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_ANER, &value, retryMax);
    TRACE_INFO(" _ANER   : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_PHYSTS, &value, retryMax);
    TRACE_INFO(" _PHYSTS  : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_PHYCR, &value, retryMax);
    TRACE_INFO(" _DP83848_PHYCR: 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_LEDCR, &value, retryMax);
    TRACE_INFO(" _DP83848_LEDCR  : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_RBR, &value, retryMax);
    TRACE_INFO(" _DP83848_RBR: 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_MICR, &value, retryMax);
    TRACE_INFO(" _DP83848_MIR : 0x%X\n\r", value);
    EMAC_ReadPhy(phyAddress, DP83848_MISR, &value, retryMax);
    TRACE_INFO(" _DP83848_MISR : 0x%X\n\r", value);

    EMAC_DisableMdio();
}

//-----------------------------------------------------------------------------
/// Find a valid PHY Address ( from 0 to 31 ).
/// Check BMSR register ( not 0 nor 0xFFFF )
/// Return 0xFF when no valid PHY Address found.
/// \param pDp          Pointer to the Dm9161 instance
//-----------------------------------------------------------------------------
static unsigned char DP83848_FindValidPhy( Dp83848 * pDp )
{
    unsigned int  retryMax;
    unsigned int  value=0;
    unsigned char rc;
    unsigned char phyAddress;
    unsigned char cnt;

    TRACE_DEBUG("DP83848_FindValidPhy\n\r");
    ASSERT(pDp, "F: DP83848_FindValidPhy\n\r");

    EMAC_EnableMdio();
    phyAddress = pDp->phyAddress;
    retryMax = pDp->retryMax;

    // Check current phyAddress
    rc = phyAddress;
    if( EMAC_ReadPhy(phyAddress, DP83848_PHYID1, &value, retryMax) == 0 ) {
    	TRACE_WARNING("DP83848 PROBLEM\n\r");
    }
    TRACE_DEBUG("_PHYID1  : 0x%X, addr: %d\n\r", value, phyAddress);

    // Find another one
    if ( value != 0x2000 /*DP83848_OUI_MSB */) {

        rc = 0xFF;
        for( cnt = 0; cnt < 32; cnt ++ ) {

            phyAddress = (phyAddress + 1) & 0x1F;
            if( EMAC_ReadPhy(phyAddress, DP83848_PHYID1, &value, retryMax) == 0 ) {
                TRACE_ERROR("DP83848 PROBLEM\n\r");
            }
            TRACE_DEBUG("_PHYID1  : 0x%X, addr: %d\n\r", value, phyAddress);
            if ( value == 0x2000 /*DP83848_OUI_MSB*/ ) {

                rc = phyAddress;
                break;
            }
        }
    }
    

    if ( rc != 0xFF ) {

        TRACE_INFO("** Valid PHY Found: %d\n\r", rc);
        EMAC_ReadPhy(phyAddress, DP83848_PHYSTS, &value, retryMax);
        TRACE_DEBUG("_PHYSTS  : 0x%X, addr: %d\n\r", value, phyAddress);
    }

	EMAC_DisableMdio();

    return(rc);
}

//-----------------------------------------------------------------------------
//         Exported functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Setup the maximum timeout count of the driver.
/// \param pDp   Pointer to the Dm9161 instance
/// \param toMax Timeout maxmum count.
//-----------------------------------------------------------------------------
void DP83848_SetupTimeout( Dp83848 *pDp, unsigned int toMax ) {
    ASSERT(pDp, "-F- DP83848_SetupTimeout\n\r");

    pDp->retryMax = toMax;
}

//-----------------------------------------------------------------------------
/// Initialize the Dp83848 instance
/// \param pDp          Pointer to the Dp83848 instance
/// \param pEmac        Pointer to the Emac instance for the Dp83848
/// \param phyAddress   The PHY address used to access the PHY
///                     ( pre-defined by pin status on PHY reset )
//-----------------------------------------------------------------------------
void DP83848_Init( Dp83848 *pDp, unsigned char phyAddress )
{
    ASSERT(pDp , "-F- DP83848_Init\n\r");

    pDp->phyAddress = phyAddress;

    // Initialize timeout by default
    pDp->retryMax = DP83848_RETRY_MAX;
}


//-----------------------------------------------------------------------------
/// Issue a SW reset to reset all registers of the PHY
/// Return 1 if successfully, 0 if timeout.
/// \param pDp   Pointer to the Dp83848 instance
//-----------------------------------------------------------------------------
static unsigned char DP83848_ResetPhy( Dp83848 *pDp )
{
    unsigned int retryMax;
    unsigned int bmcr = DP83848_RESET;
    unsigned char phyAddress;
    unsigned int timeout = 10;
    unsigned char ret = 0;

    ASSERT(pDp, "-F- DP83848_ResetPhy");


    phyAddress = pDp->phyAddress;
    retryMax = pDp->retryMax;

    TRACE_INFO(" DP83848_ResetPhy(): phyAddr == %d\n\r", phyAddress);

    EMAC_EnableMdio();

    bmcr = DP83848_RESET;
    if ( (ret = EMAC_WritePhy(phyAddress, DP83848_BMCR, bmcr, retryMax)) ){
		do {
			EMAC_ReadPhy(phyAddress, DP83848_BMCR, &bmcr, retryMax);
			timeout--;
		} while ( (bmcr & DP83848_RESET) && timeout );
    }
    EMAC_DisableMdio();

    TRACE_INFO(" DP83848_ResetPhy() ret == %d, timeout == %d\n\r", ret, timeout);

    if ( !timeout ) {
        ret = 0;
    }

    TRACE_INFO(" DP83848_ResetPhy() returns: %d\n\r", ret);

    return(ret);
}

//-----------------------------------------------------------------------------
/// Do a HW initialize to the PHY ( via RSTC ) and setup clocks & PIOs
/// This should be called only once to initialize the PHY pre-settings.
/// The PHY address is reset status of CRS,RXD[3:0] (the emacPins' pullups).
/// The COL pin is used to select MII mode on reset (pulled up for Reduced MII)
/// The RXDV pin is used to select test mode on reset (pulled up for test mode)
/// The above pins should be predefined for corresponding settings in resetPins
/// The EMAC peripheral pins are configured after the reset done.
/// Return 1 if RESET OK, 0 if timeout.
/// \param pDp         Pointer to the Dm9161 instance
/// \param mck         Main clock setting to initialize clock
/// \param resetPins   Pointer to list of PIOs to configure before HW RESET
///                       (for PHY power on reset configuration latch)
/// \param nbResetPins Number of PIO items that should be configured
/// \param emacPins    Pointer to list of PIOs for the EMAC interface
/// \param nbEmacPins  Number of PIO items that should be configured
//-----------------------------------------------------------------------------
unsigned char DP83848_InitPhy(Dp83848 *pDp, unsigned int mck,
		const Pin *pResetPins, unsigned int nbResetPins, const Pin *pEmacPins,
		unsigned int nbEmacPins) {
	unsigned char rc = 1;
	unsigned char phy;

	ASSERT(pDp, "-F- DP83848_InitPhy\n\r");

	// Perform RESET
	TRACE_DEBUG("HW RESET PHY\n\r");

#if 1
	// After PHY power up, hardware reset.
	AT91C_BASE_RSTC->RSTC_RMR = emacRESET_KEY | emacRESET_LENGTH;
	AT91C_BASE_RSTC->RSTC_RCR = emacRESET_KEY | AT91C_RSTC_EXTRST;

	/* Wait for hardware reset end. */
	while ( !RSTC_GetNrstLevel() ) {
		__asm volatile ( "NOP" );
	}
#else
	if ( pResetPins ) {
		// Configure PINS
		PIO_Configure(pResetPins, nbResetPins);

		// Execute reset
		RSTC_SetExtResetLength(DP83848_RESET_LENGTH);
		RSTC_ExtReset();

		// Wait for end hardware reset
		while ( !RSTC_GetNrstLevel() ) {
			; // TODO @@@jwg: hier noch nopsen!?
		}
	}
#endif

	// Configure EMAC runtime pins
	if ( rc ) {
#if 0
		PIO_Configure(pEmacPins, nbEmacPins);
#else
		AT91C_BASE_PIOB->PIO_ASR = emacPERIPHERAL_A_SETUP;
		AT91C_BASE_PIOB->PIO_PDR = emacPERIPHERAL_A_SETUP;
#endif
		// Enable management port for com between EMAC and PHY.
		EMAC_EnableMdio();

#if 1
		rc = EMAC_SetMdcClock(mck);
		if ( !rc ) {
			TRACE_ERROR("No Valid MDC clock\n\r");
			return (0);
		}
#else
		AT91C_BASE_EMAC->EMAC_NCFGR |= ( 2 ) << 10;
#endif

		// delay!?

		/* Disable management port again. */
		EMAC_DisableMdio();

		/* Enable EMAC in RMII mode */
		EMAC_EnableRMII();
	}

	return (rc);
}

#if 1
/*
 * 
 */
unsigned char DP83848_Setup( Dp83848 *pDp ) {
	unsigned int retryMax;
//	unsigned int value;
	unsigned char count;
	unsigned int bmcr;
	unsigned int phyId1;
	unsigned int phyId2;
//	unsigned int retryCount = 0;
	unsigned char phyAddress;
	unsigned char rc = 1;

	ASSERT(pDp, "-F- DP83848_Setup\n\r");

	// Get configured phyAddress as starting point
	phyAddress = pDp->phyAddress;
	retryMax = pDp->retryMax;

	EMAC_EnableMdio();

	for ( count = 0; count < 32; count ++ ) {
		phyAddress = (phyAddress + 1) & 0x1F;
		if ( !EMAC_ReadPhy(phyAddress, DP83848_PHYID1, &phyId1, retryMax) ) {
			TRACE_ERROR("-FAILED- EMAC_ReadPhy Id1\n\r");
			rc = 0;
			goto SetupExit;
		}
		TRACE_DEBUG("ReadPhy Id1 0x%X, address: %d\n\r", phyId1, phyAddress);

		if ( !EMAC_ReadPhy(phyAddress, DP83848_PHYID2, &phyId2, retryMax) ) {
			TRACE_ERROR("-FAILED- EMAC_ReadPhy Id2\n\r");
			rc = 0;
			goto SetupExit;
		}
		TRACE_DEBUG("ReadPhy Id2 0x%X\n\r", phyId2);

		if ( (phyId1 == DP83848_OUI_MSB)
	/*		&& (((phyAnar >> 10) & DP83848_LSB_MASK) == DP83848_OUI_LSB)*/ ) {

			TRACE_DEBUG("Vendor Number Model = 0x%X\n\r", ((phyId2>>4)&0x3F));
			TRACE_DEBUG("Model Revision Number = 0x%X\n\r", (phyId2&0x7));
//			break;
		}
		else {
			TRACE_DEBUG("Problem OUI value\n\r");
		}
	}

	// Save detected phyAddress
    pDp->phyAddress = phyAddress;

    TRACE_INFO("PHY addr: %d\n\r", pDp->phyAddress);

    // Get current PHY BMCR
	if ( !EMAC_ReadPhy(phyAddress, DP83848_BMCR, &bmcr, retryMax) ) {
		TRACE_ERROR("-FAILED- EMAC_ReadPhy BMCR\n\r");
		rc = 0;
		goto SetupExit;
	}

    // Make settings
	bmcr &= ~(DP83848_LOOPBACK|DP83848_POWER_DOWN);
	bmcr |=  DP83848_ISOLATE;   	// Electrically isolate PHY

    // No auto-negotiation
	bmcr &= ~DP83848_AUTONEG;   	// Remove autonegotiation enable
	bmcr |= DP83848_SPEED_SELECT;	// Set fixed speed to 100 Mb/s
	bmcr |= DP83848_DUPLEX_MODE;	// Enable full duplex

	// Write back control register
    rc = EMAC_WritePhy(phyAddress, DP83848_BMCR, bmcr, retryMax);
    if ( rc == 0 ) {
    	TRACE_ERROR("-FAILED- EMAC_WritePhy(BMCR)\n\r");
         goto SetupExit;
    }

   	// 100 Mb/s, full duplex
    EMAC_SetLinkSpeed(1, 1);

	EMAC_EnableRMII();			// TODO @@@jwg: dopplet hier!?

SetupExit:
	EMAC_DisableMdio();

//	DP83848_DumpRegisters(pDp);

	return(rc);
}

#else
//-----------------------------------------------------------------------------
/// Issue a Auto Negotiation of the PHY
/// Return 1 if successfully, 0 if timeout.
/// \param pDm   Pointer to the Dm9161 instance
//-----------------------------------------------------------------------------
unsigned char DP83848_AutoNegotiate(Dm9161 *pDm)
{
    unsigned int retryMax;
    unsigned int value;
    unsigned int phyAnar;
    unsigned int phyAnalpar;
    unsigned int retryCount= 0;
    unsigned char phyAddress;
    unsigned char rc = 1;

    ASSERT(pDm, "-F- DP83848_AutoNegotiate\n\r");
    phyAddress = pDm->phyAddress;
    retryMax = pDm->retryMax;

    EMAC_EnableMdio();

    if (!EMAC_ReadPhy(phyAddress, DP83848_PHYID1, &value, retryMax)) {
        TRACE_ERROR("Pb EMAC_ReadPhy Id1\n\r");
        rc = 0;
        goto AutoNegotiateExit;
    }
    TRACE_DEBUG("ReadPhy Id1 0x%X, addresse: %d\n\r", value, phyAddress);
    if (!EMAC_ReadPhy(phyAddress, DP83848_PHYID2, &phyAnar, retryMax)) {
        TRACE_ERROR("Pb EMAC_ReadPhy Id2\n\r");
        rc = 0;
        goto AutoNegotiateExit;
    }
    TRACE_DEBUG("ReadPhy Id2 0x%X\n\r", phyAnar);

    if( ( value == DP83848_OUI_MSB )
     && ( ((phyAnar>>10)&DP83848_LSB_MASK) == DP83848_OUI_LSB ) ) {

        TRACE_DEBUG("Vendor Number Model = 0x%X\n\r", ((phyAnar>>4)&0x3F));
        TRACE_DEBUG("Model Revision Number = 0x%X\n\r", (phyAnar&0x7));
    }
    else {
        TRACE_ERROR("Problem OUI value\n\r");
    }        

    // Setup control register
    rc  = EMAC_ReadPhy(phyAddress, DP83848_BMCR, &value, retryMax);
    if (rc == 0) {

        goto AutoNegotiateExit;
    }

    value &= ~DP83848_AUTONEG;   // Remove autonegotiation enable
    value &= ~(DP83848_LOOPBACK|DP83848_POWER_DOWN);
    value |=  DP83848_ISOLATE;   // Electrically isolate PHY
    rc = EMAC_WritePhy(phyAddress, DP83848_BMCR, value, retryMax);
    if (rc == 0) {

        goto AutoNegotiateExit;
    }

    // Set the Auto_negotiation Advertisement Register
    // MII advertising for Next page
    // 100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3
    phyAnar = DP83848_NP | DP83848_TX_FDX | DP83848_TX_HDX |
              DP83848_10_FDX | DP83848_10_HDX | DP83848_AN_IEEE_802_3;
    rc = EMAC_WritePhy(phyAddress, DP83848_ANAR, phyAnar, retryMax);
    if (rc == 0) {

        goto AutoNegotiateExit;
    }

    // Read & modify control register
    rc  = EMAC_ReadPhy(phyAddress, DP83848_BMCR, &value, retryMax);
    if (rc == 0) {

        goto AutoNegotiateExit;
    }

    value |= DP83848_SPEED_SELECT | DP83848_AUTONEG | DP83848_DUPLEX_MODE;
    rc = EMAC_WritePhy(phyAddress, DP83848_BMCR, value, retryMax);
    if (rc == 0) {

        goto AutoNegotiateExit;
    }

    // Restart Auto_negotiation
    value |=  DP83848_RESTART_AUTONEG;
    value &= ~DP83848_ISOLATE;
    rc = EMAC_WritePhy(phyAddress, DP83848_BMCR, value, retryMax);
    if (rc == 0) {

        goto AutoNegotiateExit;
    }
    TRACE_DEBUG(" _BMCR: 0x%X\n\r", value);

    // Check AutoNegotiate complete
    while (1) {

        rc  = EMAC_ReadPhy(phyAddress, DP83848_BMSR, &value, retryMax);
        if (rc == 0) {

            TRACE_ERROR("rc==0\n\r");
            goto AutoNegotiateExit;
        }
        // Done successfully
        if (value & DP83848_AUTONEG_COMP) {

            TRACE_INFO("AutoNegotiate complete\n\r");
            break;
        }
        // Timeout check
        if (retryMax) {

            if (++ retryCount >= retryMax) {

                DP83848_DumpRegisters(pDm);
                TRACE_FATAL("TimeOut\n\r");
                goto AutoNegotiateExit;
            }
        }
    }

    // Get the AutoNeg Link partner base page
    rc  = EMAC_ReadPhy(phyAddress, DP83848_ANLPAR, &phyAnalpar, retryMax);
    if (rc == 0) {

        goto AutoNegotiateExit;
    }

    // Setup the EMAC link speed
    if ((phyAnar & phyAnalpar) & DP83848_TX_FDX) {

        // set MII for 100BaseTX and Full Duplex
        EMAC_SetLinkSpeed(1, 1);
    }
    else if ((phyAnar & phyAnalpar) & DP83848_10_FDX) {

        // set MII for 10BaseT and Full Duplex
        EMAC_SetLinkSpeed(0, 1);
    }
    else if ((phyAnar & phyAnalpar) & DP83848_TX_HDX) {

        // set MII for 100BaseTX and half Duplex
        EMAC_SetLinkSpeed(1, 0);
    }
    else if ((phyAnar & phyAnalpar) & DP83848_10_HDX) {

        // set MII for 10BaseT and half Duplex
        EMAC_SetLinkSpeed(0, 0);
    }

    // Setup EMAC mode
#if BOARD_EMAC_MODE_RMII != 1
    EMAC_EnableMII();
#else
    EMAC_EnableRMII();
#endif

AutoNegotiateExit:
    EMAC_DisableMdio();
    return rc;
}

//-----------------------------------------------------------------------------
/// Get the Link & speed settings, and automatically setup the EMAC with the
/// settings.
/// Return 1 if link found, 0 if no ethernet link.
/// \param pDm          Pointer to the Dm9161 instance
/// \param applySetting Apply the settings to EMAC interface
//-----------------------------------------------------------------------------
unsigned char DP83848_GetLinkSpeed(Dm9161 *pDm, unsigned char applySetting)
{
    unsigned int retryMax;
    unsigned int stat1;
    unsigned int stat2;
    unsigned char phyAddress;
    unsigned char rc = 1;

    TRACE_DEBUG("DP83848_GetLinkSpeed\n\r");
    ASSERT(pDm, "-F- DP83848_GetLinkSpeed\n\r");

    EMAC_EnableMdio();
    phyAddress = pDm->phyAddress;
    retryMax = pDm->retryMax;

    rc  = EMAC_ReadPhy(phyAddress, DP83848_BMSR, &stat1, retryMax);
    if (rc == 0) {

        goto GetLinkSpeedExit;
    }

    if ((stat1 & DP83848_LINK_STATUS) == 0) {

        TRACE_ERROR("Pb: LinkStat: 0x%x\n\r", stat1);

        rc = 0;
        goto GetLinkSpeedExit;
    }

    if (applySetting == 0) {

        TRACE_ERROR("Pb: applySetting: 0x%x\n\r", applySetting);
        goto GetLinkSpeedExit;
    }

    // Re-configure Link speed
    rc  = EMAC_ReadPhy(phyAddress, DP83848_DSCSR, &stat2, retryMax);
    if (rc == 0) {

        TRACE_ERROR("Pb: rc: 0x%x\n\r", rc);
        goto GetLinkSpeedExit;
    }

    if ((stat1 & DP83848_100BASE_TX_FD) && (stat2 & DP83848_100FDX)) {

        // set Emac for 100BaseTX and Full Duplex
        EMAC_SetLinkSpeed(1, 1);
    }

    if ((stat1 & DP83848_10BASE_T_FD) && (stat2 & DP83848_10FDX)) {

        // set MII for 10BaseT and Full Duplex
        EMAC_SetLinkSpeed(0, 1);
    }

    if ((stat1 & DP83848_100BASE_T4_HD) && (stat2 & DP83848_100HDX)) {

        // set MII for 100BaseTX and Half Duplex
        EMAC_SetLinkSpeed(1, 0);
    }

    if ((stat1 & DP83848_10BASE_T_HD) && (stat2 & DP83848_10HDX)) {

        // set MII for 10BaseT and Half Duplex
        EMAC_SetLinkSpeed(0, 0);
    }

    // Start the EMAC transfers
    TRACE_DEBUG("DP83848_GetLinkSpeed passed\n\r");

GetLinkSpeedExit:
    EMAC_DisableMdio();
    return rc;
}
#endif
