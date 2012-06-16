/**
 * @file board.h
 *
 * @brief AT91Lib board specific functionality
 *
 * This file provides a number of definitions, which are needed by the AT91Lib.
 *
 * $Id: board.h,v 1.1 2011/05/02 16:37:10 dam Exp $
 *
 * @author
 *      dresden elektronik: http://www.dresden-elektronik.de
 *      Support email: support@dresden-elektronik.de
 *
 *
 * Copyright (c) 2011, dresden elektronik All rights reserved.
 *
 * Licensed under dresden elektronik's Limited License Agreement --> deEULA.txt
 */

#ifndef BOARD_H
#define BOARD_H

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <AT91SAM7X512.h>

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Board
//------------------------------------------------------------------------------
/// String containing the name of the board.
#define BOARD_NAME      "deRFarm7_25X00_deRFgateway_1XXX2"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Clocks
//------------------------------------------------------------------------------
/// Frequency of the board main oscillator, in Hz.
#define BOARD_MAINOSC           18432000

/// Master clock frequency (when using board_lowlevel.c), in Hz.
#define BOARD_MCK               48000000
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PIO definitions
//------------------------------------------------------------------------------
// DBGU pins definition.
#define PINS_DBGU  {0x18000000, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

/// LED 0 pin definition.
#define PIN_LED_0  {1 << 26, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}
/// LED 1 pin definition.
#define PIN_LED_1  {1 << 21, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}
/// LED 2 pin definition.
#define PIN_LED_2  {1 << 19, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}
/// List of all LEDs pin definitions.
#define PINS_LEDS  PIN_LED_0, PIN_LED_1, PIN_LED_2

/// Push button #0 definition.
#define PIN_PUSHBUTTON_0    {1 << 3, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEGLITCH | PIO_PULLUP}
/// Push button #1 definition
#define PIN_PUSHBUTTON_1    {1 << 25, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEGLITCH | PIO_PULLUP}
/// List of all push button definitions.
#define PINS_PUSHBUTTONS    PIN_PUSHBUTTON_0, PIN_PUSHBUTTON_1
//------------------------------------------------------------------------------
/// Board EMAC Power Down control pin
#define BOARD_EMAC_PIN_PWRDN { AT91C_PIO_PB18, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

/// Board EMAC mode - RMII/MII ( 1/0 )
#define BOARD_EMAC_MODE_RMII 1 // MII is the default mode

/// The PIN list of PIO for EMAC
#if 1
#define BOARD_EMAC_PINS     { (   AT91C_PB2_ETX0 \
								| AT91C_PB3_ETX1 \
								| AT91C_PB1_ETXEN \
							    | AT91C_PB7_ERXER \
							    | AT91C_PB5_ERX0 \
							    | AT91C_PB6_ERX1 \
							    | AT91C_PB15_ERXDV_ECRSDV \
							    | AT91C_PB8_EMDC \
							    | AT91C_PB9_EMDIO ), \
							  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT }
#else
#define BOARD_EMAC_PINS       {0x3EFFF, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#endif
/// The power up reset latch PIO for PHY
#define BOARD_EMAC_PIN_TEST   {(1<<15), AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}
#define BOARD_EMAC_PIN_RMII   {(1<<16), AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

// We force the address
//(1<<6) PHY address 1, (1<<5) PHY address 2,
#define BOARD_EMAC_PINS_PHYAD {(1<<6), AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT},\
                              {(1<<5), AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}

#define BOARD_EMAC_PIN_10BT   {(1<<17), AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

#define BOARD_EMAC_PIN_RPTR   {(1<< 7), AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

/// The PIN Configure list for EMAC on power up reset (MII)
#define BOARD_EMAC_RST_PINS BOARD_EMAC_PINS_PHYAD

/// The runtime pin configure list for EMAC
#define BOARD_EMAC_RUN_PINS BOARD_EMAC_PINS

#endif //#ifndef BOARD_H

