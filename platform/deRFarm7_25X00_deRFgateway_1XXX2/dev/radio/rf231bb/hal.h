/*   Copyright (c) 2008, Swedish Institute of Computer Science
 *  All rights reserved. 
 *
 *  Additional fixes for AVR contributed by:
 *
 *	Colin O'Flynn coflynn@newae.com
 *	Eric Gnoske egnoske@gmail.com
 *	Blake Leverett bleverett@gmail.com
 *	Mike Vidales mavida404@gmail.com
 *	Kevin Brown kbrown3@uccs.edu
 *	Nate Bohlmann nate@elfwerks.com
 *
 *	Adaptions for ARM7 by: Joerg Wolf <gwynpen@googlemail.com>
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *    \addtogroup hal
 *    @{
 */

/**
 *  \file
 *  \brief This file contains low-level radio driver code.
 *
 *   $Id: hal.h,v 1.5 2010/12/03 20:42:01 dak664 Exp $
*/

#ifndef HAL_AT91SAM7X_H
#define HAL_AT91SAM7X_H
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
//#include <util/crc16.h>
#include "contiki-conf.h"

#include <AT91SAM7X512.h>
#include <board.h>

/*
 * Identifiers for PIO's in AT91SAM7X512
 */
typedef enum pio_type_tag
{
    PIO_A,
    PIO_B
} pio_type_t;

/*============================ MACROS ========================================*/
#ifndef _BV
/**
 * Bit value -- compute the bitmask for a bit position
 */
#define _BV(x) (1 << (x))
#endif

#define sei()           __asm__ volatile (                          \
    "STMDB   SP!, {R0}  \n\t" /* Push R0.                   */  \
    "MRS  R0, CPSR      \n\t" /* Get CPSR.                  */  \
    "BIC  R0, R0, #0xC0 \n\t" /* Enable IRQ, FIQ.           */  \
    "MSR  CPSR, R0      \n\t" /* Write back modified value. */  \
    "LDMIA   SP!, {R0}" );    /* Pop R0.                    */


#define cli()           __asm__ volatile (                          \
    "STMDB   SP!, {R0}   \n\t" /* Push R0.                  */  \
    "MRS  R0, CPSR       \n\t" /* Get CPSR.                 */  \
    "ORR  R0, R0, #0xC0  \n\t" /* Disable IRQ, FIQ.         */  \
    "MSR  CPSR, R0       \n\t" /* Write back modified value.*/  \
    "LDMIA   SP!, {R0}" );     /* Pop R0.                   */

/* Gets the current program status word */
#define GET_CPSR(sreg)  __asm__ volatile ("MRS %0, CPSR" : "=r" (sreg) :);

/* Sets the current program status word */
#define SET_CPSR(sreg)  __asm__ volatile ("MSR CPSR_c, %0" : : "r" (sreg));

//#define nop()           do { __asm__ volatile ("nop"); } while (0)

/* Enables the global interrupt. */
#define ENABLE_GLOBAL_IRQ()             sei()
/* Disables the global interrupt. */
#define DISABLE_GLOBAL_IRQ()            cli()

#define ENTER_CRITICAL_REGION() { uint32_t sreg; GET_CPSR(sreg); \
                                DISABLE_GLOBAL_IRQ()

#define LEAVE_CRITICAL_REGION() SET_CPSR(sreg);}

/* Enables the transceiver interrupts. */
#define ENABLE_TRX_IRQ()                  ( PIO_EnableIt(&pinIRQ) )

/* Disables the transceiver interrupts. */
#define DISABLE_TRX_IRQ()                 ( PIO_DisableIt(&pinIRQ) )

/* Clears the transceiver interrupts. */
#define CLEAR_TRX_IRQ()                   ( pinIRQ.pio->PIO_ISR )


/* Enables the transceiver interrupts. */
#define ENABLE_TRX_IRQ_TSTAMP()           ( PIO_EnableIt(&pinRXTS) )

/* Disables the transceiver interrupts. */
#define DISABLE_TRX_IRQ_TSTAMP()          ( PIO_DisableIt(&pinRXTS) )

/* Clears the transceiver interrupts. */
#define CLEAR_TRX_IRQ_TSTAMP()            ( pinRXTS.pio->PIO_ISR )

/*
 * Configures a given pin to trigger interrupts over the PIO Controller
 * This works only after PIO Interrupts have been enabled.
 */
#define PIN_CONFIGURE_PIO_IRQ(irq_pin, handler)     do {          \
    PIO_Configure(&irq_pin, 1);                                   \
    PIO_ConfigureIt(&irq_pin, (void (*)(const Pin *)) handler);   \
} while(0);

#define pal_trx_irq_en()                ENABLE_TRX_IRQ()
#define pal_trx_irq_en_tstamp()         ENABLE_TRX_IRQ_TSTAMP()

/*
 * Write access command of the transceiver
 */
#define WRITE_ACCESS_COMMAND            (0xC0)

/*
 * Read access command to the tranceiver
 */
#define READ_ACCESS_COMMAND             (0x80)

/*
 * Frame write command of transceiver
 */
#define TRX_CMD_FW                      (0x60)

/*
 * Frame read command of transceiver
 */
#define TRX_CMD_FR                      (0x20)

/*
 * SRAM write command of transceiver
 */
#define TRX_CMD_SW                      (0x40)

/*
 * SRAM read command of transceiver
 */
#define TRX_CMD_SR                      (0x00)


/*
 * Sets the pin of requested PIO
 */
#define PIN_SET(pin, pio)   do {                                        \
    AT91S_PIO *pio_ptr=NULL;                                            \
    if(pio == PIO_A)                                                    \
        pio_ptr = AT91C_BASE_PIOA;                                      \
    else if(pio == PIO_B)                                               \
        pio_ptr = AT91C_BASE_PIOB;                                      \
    if(NULL != pio_ptr) {                                               \
        pio_ptr->PIO_SODR = pin;                                        \
    }                                                                   \
} while (0);

/*
 * Clears the pin of requested PIO
 */
#define PIN_CLEAR(pin, pio) do {                                        \
    AT91S_PIO *pio_ptr=NULL;                                            \
    if(pio == PIO_A)                                                    \
        pio_ptr = AT91C_BASE_PIOA;                                      \
    else if(pio == PIO_B)                                               \
        pio_ptr = AT91C_BASE_PIOB;                                      \
    if(NULL != pio_ptr) {                                               \
        pio_ptr->PIO_CODR = pin;                                        \
    }                                                                   \
} while (0);

/*
 * GPIO macros for AT91SAM7X512
 */

/*
 * This board uses an SPI-attached transceiver.
 */
#define PAL_USE_SPI_TRX                 (1)

/* RESET pin */
#define RST                             (AT91C_PIO_PA9)

/* Sleep Transceiver pin */
#define SLP_TR                          (AT91C_PIO_PA8)

/* Slave select pin */
#define SEL                             (AT91C_PIO_PA21)

/* SPI Bus Master Input/Slave Output pin */
#define MISO                            (AT91C_PIO_PA24)

/* SPI Bus Master Output/Slave Input pin */
#define MOSI                            (AT91C_PIO_PA23)

/* SPI serial clock pin */
#define SCK                             (AT91C_PIO_PA22)

/*
 * Set TRX GPIO pins.
 */
#define RST_HIGH()                      {PIN_SET(RST, PIO_A)}
#define RST_LOW()                       {PIN_CLEAR(RST, PIO_A)}
#define SLP_TR_HIGH()                   {PIN_SET(SLP_TR, PIO_A)}
#define SLP_TR_LOW()                    {PIN_CLEAR(SLP_TR, PIO_A)}

#define hal_set_rst_low( )    RST_LOW() /**< This macro pulls the RST pin low. */
#define hal_set_rst_high( )   RST_HIGH() /**< This macro pulls the RST pin high. */
#define hal_set_slptr_high( ) SLP_TR_HIGH()      /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low( )  SLP_TR_LOW()     /**< This macro pulls the SLP_TR pin low. */
//#define hal_get_slptr( ) (    ( TRXPR & ( 1 << SLPTR ) ) >> SLPTR )  /**< Read current state of the SLP_TR pin (High/Low). */
#define hal_get_slptr( )      ( AT91C_BASE_PIOA->PIO_OER?((AT91C_BASE_PIOA->PIO_ODSR & SLP_TR)?true : false):((AT91C_BASE_PIOA->PIO_PDSR & SLP_TR)?true : false))  /**< Read current state of the SLP_TR pin (High/Low). */

/* The pin on AT91SAM7X512 to which the transceiver interrupt is connected. */
#define TRX_IRQ_PIN       {1 << 29, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}

/* The pin on AT91SAM7X512 to which the transceiver timestamp interrupt is connected. */
#define TRX_TS_IRQ_PIN    {1 << 23, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}


/* The CPU clock */
#define F_CPU                           (BOARD_MCK)

/* F_CPU dependent variables */
#if (F_CPU == 32000000UL)
  #error "currently unsupported"

#elif (F_CPU == 48000000UL)

  // F_Xtal=18.432MHz => 96[MHz] = 18,432[MHz] *125/24

  /* PLL multiplier to generate PLL clock of 96Mhz. */
  #define PLL_MULTIPLIER                  (124)

  /* PLL divider to generate PLL clock of 96Mhz. */
  #define PLL_DIVIDER                     (24)

/*
 * The prescaler of the master clock controller used to generate 48MHz clock
 * from 96MHz PLL clock.
 */
#define PMC_PRESCALER                   (AT91C_PMC_PRES_CLK_2)

/* USB PLL divisor value to obtain a 48MHz clock. */
#define BOARD_USBDIV                    (AT91C_CKGR_USBDIV_1)

/* SPI baud rate divider to generate 4MHz SPI clock, when F_CPU is 48MHz. */
#define SPI_BR_DIVIDER                  (12)

#else
#error "unsupported main clock"

#endif /* F_CPU == ... */


/*
 * Macros dealing with the PMC of AT91SAM7X512
 */

/*
 * Enables the clock to the given peripheral id
 */
#define PERIPHERAL_CLOCK_ENABLE(id)     (AT91C_BASE_PMC->PMC_PCER = _BV(id))

/*
 * Disables the clock to the given peripheral id
 */
#define PERIPHERAL_CLOCK_DISABLE(id)    (AT91C_BASE_PMC->PMC_PCDR = _BV(id))



/*
 * SPI Base Register for TRX access
 */
#define AT91C_BASE_SPI_USED             (AT91C_BASE_SPI1)


/*
 * Macros dealing with the PIO of AT91SAM7X512
 */

/*
 * Enables the PIO control for the requested pin and sets it as output
 */
#define PIN_SET_AS_PIO_OUTPUT(pin, pio) do {                            \
    AT91S_PIO *pio_ptr=NULL;                                            \
    if(pio == PIO_A)                                                    \
        pio_ptr = AT91C_BASE_PIOA;                                      \
    else if(pio == PIO_B)                                               \
        pio_ptr = AT91C_BASE_PIOB;                                      \
    if(NULL != pio_ptr) {                                               \
        /* Set Bit corresponding to pin in output enable register. */   \
        pio_ptr->PIO_OER = pin;                                         \
        /* PIO control is enabled on the specified pin. */              \
        pio_ptr->PIO_PER = pin;                                         \
    }                                                                   \
} while (0);

/*
 * Disables the PIO control and enables the peripheral B control for the
 * requested pin.
 */
#define PIN_SET_AS_PERIPHERAL_B(pin, pio) do {                          \
    AT91S_PIO *pio_ptr=NULL;                                            \
    if(pio == PIO_A)                                                    \
        pio_ptr = AT91C_BASE_PIOA;                                      \
    else if(pio == PIO_B)                                               \
        pio_ptr = AT91C_BASE_PIOB;                                      \
                                                                        \
    if(pio_ptr != NULL) {                                               \
        /* Bit corresponding to pin is set in B select register. */     \
        pio_ptr->PIO_BSR = pin;                                         \
        /* PIO control is disabled on the specified pin. */             \
        pio_ptr->PIO_PDR = pin;                                         \
    }                                                                   \
} while (0);

/*
 * TRX Access macros for AT91SAM7X512
 */

/*
 * Position of the PCS (peripheral chip select) field in the SPI_MR register.
 */
#define PCS_FIELD_IN_MR                 (16)

/*
 * Value that needs to be written to in the PCS field of the SPI_MR to
 * activate the line to which the trx select line is connected.
 */
#define PCS_FIELD_VALUE                 (3)

/*
 * Value of PCS field in SPI_MR (mode register) which indicates the contoller
 * about the line to which the slave is connected.
 */
#define SS_ENABLE                       (PCS_FIELD_VALUE << PCS_FIELD_IN_MR)

/*
 * Slave select made low
 */
#define SS_LOW()                        PIN_CLEAR(SEL, PIO_A)

/*
 * Slave select made high
 */
#define SS_HIGH()                       PIN_SET(SEL, PIO_A)

/*
 * Dummy value to be written in SPI transmit register to retrieve data form it
 */
#define SPI_DUMMY_VALUE                 (0x00)

/* Reads the data from the SPI receive register. */
#define SPI_READ(register_value)    do {                        \
    while ((AT91C_BASE_SPI1->SPI_SR & AT91C_SPI_RDRF) == 0);    \
    register_value = (AT91C_BASE_SPI1->SPI_RDR & 0xFFFF);       \
} while (0);

/* Writes the data into the SPI transmit register. */
#define SPI_WRITE(data)     do {                                    \
    while ((AT91C_BASE_SPI1->SPI_SR & AT91C_SPI_TDRE) == 0);        \
    AT91C_BASE_SPI1->SPI_TDR = data & 0xFFFF;                       \
    while ((AT91C_BASE_SPI1->SPI_SR & AT91C_SPI_TXEMPTY) == 0);     \
} while (0);

/**
 * @brief Block for a given time
 * @param value timeout given in microseconds
 */
//inline void delay_us(unsigned int value)
// execute "no-operation" instructions for some time
#define delay_us(value) do { \
    unsigned int i; \
    for(i=0; i<((BOARD_MCK/1000000)*value); i++) \
        __asm__ __volatile__ ("nop"); \
} while(0);

#ifndef RF231_CONF_RX_BUFFERS
#define RF231_CONF_RX_BUFFERS 1
#endif

/** \name Macros for radio operation.
 * \{
 */

#define HAL_BAT_LOW_MASK       ( 0x80 ) /**< Mask for the BAT_LOW interrupt. */
#define HAL_TRX_UR_MASK        ( 0x40 ) /**< Mask for the TRX_UR interrupt. */
#define HAL_TRX_END_MASK       ( 0x08 ) /**< Mask for the TRX_END interrupt. */
#define HAL_RX_START_MASK      ( 0x04 ) /**< Mask for the RX_START interrupt. */
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) /**< Mask for the PLL_UNLOCK interrupt. */
#define HAL_PLL_LOCK_MASK      ( 0x01 ) /**< Mask for the PLL_LOCK interrupt. */

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */
/** \} */

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */
/** \} */

/*============================ TYPDEFS =======================================*/

/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
} hal_rx_frame_t;

/*============================ PROTOTYPES ====================================*/
void 	hal_init( void );
uint8_t hal_register_read( uint8_t address );
void 	hal_register_write( uint8_t address, uint8_t value );
uint8_t hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position );
void 	hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position, uint8_t value );

#if 0
// TEST CODE
#define TRIG1 DDRB |= 0x04, PINB |= 0x04
#define TRIG2 DDRD |= 0x80, PIND |= 0x80

/** \name This is the list of pin configurations needed for a given platform.
 * \brief Change these values to port to other platforms.
 * \{
 */
/* Define all possible revisions here */
// Don't use zero, it will match if undefined!
// RAVEN_D : Raven kit with LCD display
// RAVENUSB_C : used for USB key or Raven card 
// RCB_B : RZ200 kit from Atmel based on 1281V
// ZIGBIT : Zigbit module from Meshnetics
// ATMEGA128RFA1 : Bare chip with internal radio
// IRIS : IRIS Mote from MEMSIC
#define RAVEN_D	    4
#define RAVENUSB_C  1
#define RCB_B	    	2
#define ZIGBIT			3
#define ATMEGA128RFA1   4
#define IRIS			5
#define DE_RF_ARM7		6



/* TODO: Move to platform (or CPU specific) */
#if RCB_REVISION == RCB_B
/* 1281 rcb */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define IRQPORT    D
#   define IRQPIN     (0x04)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_SPARE_TIMER

#elif HARWARE_REVISION == ZIGBIT
/* 1281V Zigbit */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    A
#   define RSTPIN     (0x07)
#   define IRQPORT    E
#   define IRQPIN     (0x05)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define TXCWPORT   B
#   define TXCWPIN    (0x07)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
//#   define TICKTIMER  3
//#   define HAS_SPARE_TIMER // Not used


#elif RAVEN_REVISION == RAVEN_D
/* 1284 raven */
#   define SSPORT     B
#   define SSPIN      (0x04)
#   define SPIPORT    B
#   define MOSIPIN    (0x05)
#   define MISOPIN    (0x06)
#   define SCKPIN     (0x07)
#   define RSTPORT    B
#   define RSTPIN     (0x01)
#   define IRQPORT    D
#   define IRQPIN     (0x06)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x03)
#   define TXCWPORT   B
#   define TXCWPIN    (0x00)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER

#elif RAVEN_REVISION == RAVENUSB_C
/* 1287USB raven */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define IRQPORT    D
#   define IRQPIN     (0x04)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define TXCWPORT   B
#   define TXCWPIN    (0x07)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER

#elif HARWARE_REVISION == ATMEGA128RFA1
/* ATmega1281 with internal AT86RF231 radio */
#if 0
#   define SSPORT     B
#   define SSPIN      (0x04)
#   define SPIPORT    B
#   define MOSIPIN    (0x05)
#   define MISOPIN    (0x06)
#   define SCKPIN     (0x07)
#   define RSTPORT    B
#   define RSTPIN     (0x01)
#   define IRQPORT    D
#   define IRQPIN     (0x06)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x03)
#   define TXCWPORT   B
#   define TXCWPIN    (0x00)
#endif
#   define SLPTRPORT  TRXPR
#   define SLPTRPIN   1
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER

#elif CONTIKI_TARGET_MULLE
/* mulle 5.2 (TODO: move to platform specific) */

#   define SSPORT     3
#   define SSPIN      5
#   define MOSIPORT   1
#   define MOSIPIN    1
#   define MISOPORT   1
#   define MISOPIN    0
#   define SCKPORT    3
#   define SCKPIN     3
#   define RSTPORT    4
#   define RSTPIN     3
#   define IRQPORT    8
#   define IRQPIN     3
#   define SLPTRPORT  0
#   define SLPTRPIN   7
#   define HAS_SPARE_TIMER


#elif HARWARE_REVISION == IRIS
/* 1281 IRIS */
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    A
#   define RSTPIN     (0x06)
#   define IRQPORT    D
#   define IRQPIN     (0x04)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x07)
//#   define TXCWPORT   B
//#   define TXCWPIN    (0x07)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
//#   define TICKTIMER  3
//#   define HAS_SPARE_TIMER // Not used
#elif HARWARE_REVISION == DE_RF_ARM7
#   define SSPORT     B
#   define SSPIN      (0x00)
#   define SPIPORT    B
#   define MOSIPIN    (0x02)
#   define MISOPIN    (0x03)
#   define SCKPIN     (0x01)
#   define RSTPORT    B
#   define RSTPIN     (0x05)
#   define IRQPORT    D
#   define IRQPIN     (0x04)
#   define SLPTRPORT  B
#   define SLPTRPIN   (0x04)
#   define TXCWPORT   B
#   define TXCWPIN    (0x07)
#   define USART      1
#   define USARTVECT  USART1_RX_vect
#   define TICKTIMER  3
#   define HAS_CW_MODE
#   define HAS_SPARE_TIMER




#else

#error "Platform undefined in hal.h"



/* For architectures that have all SPI signals on the same port */
#ifndef SSPORT
#define SSPORT SPIPORT
#endif

#ifndef SCKPORT
#define SCKPORT SPIPORT
#endif

#ifndef MOSIPORT
#define MOSIPORT SPIPORT
#endif

#ifndef MISOPORT
#define MISOPORT SPIPORT
#endif

/** \} */

/**
 * \name Macros used to generate read register names from platform-specific definitions of ports.
 * \brief The various CAT macros (DDR, PORT, and PIN) are used to
 * assign port/pin/DDR names to various macro variables.  The
 * variables are assigned based on the specific connections made in
 * the hardware.  For example TCCR(TICKTIMER,A) can be used in place of TCCR0A
 * if TICKTIMER is defined as 0.
 * \{
 */
#if defined(__AVR__)
#define CAT(x, y)      x##y
#define CAT2(x, y, z)  x##y##z
#define DDR(x)         CAT(DDR,  x)
#define PORT(x)        CAT(PORT, x)
#define PIN(x)         CAT(PIN,  x)
#define UCSR(num, let) CAT2(UCSR,num,let)
#define RXEN(x)        CAT(RXEN,x)
#define TXEN(x)        CAT(TXEN,x)
#define TXC(x)         CAT(TXC,x)
#define RXC(x)         CAT(RXC,x)
#define RXCIE(x)       CAT(RXCIE,x)
#define UCSZ(x,y)      CAT2(UCSZ,x,y)
#define UBRR(x,y)      CAT2(UBRR,x,y)
#define UDRE(x)        CAT(UDRE,x)
#define UDRIE(x)       CAT(UDRIE,x)
#define UDR(x)         CAT(UDR,x)
#define TCNT(x)        CAT(TCNT,x)
#define TIMSK(x)       CAT(TIMSK,x)
#define TCCR(x,y)      CAT2(TCCR,x,y)
#define COM(x,y)       CAT2(COM,x,y)
#define OCR(x,y)       CAT2(OCR,x,y)
#define CS(x,y)        CAT2(CS,x,y)
#define WGM(x,y)       CAT2(WGM,x,y)
#define OCIE(x,y)      CAT2(OCIE,x,y)
#define COMPVECT(x)    CAT2(TIMER,x,_COMPA_vect)
#define UDREVECT(x)    CAT2(USART,x,_UDRE_vect)
#define RXVECT(x)      CAT2(USART,x,_RX_vect)
#endif

/* TODO: Move to CPU specific */
#if defined(CONTIKI_TARGET_MULLE)
#define CAT(x, y)      x##y.BYTE
#define CAT2(x, y, z)  x##y##z.BYTE
#define DDR(x)         CAT(PD,  x)
#define PORT(x)        CAT(P, x)
#define PIN(x)         CAT(P, x)
#define UCSR(num, let) CAT2(UCSR,num,let)
#define RXEN(x)        CAT(RXEN,x)
#define TXEN(x)        CAT(TXEN,x)
#define TXC(x)         CAT(TXC,x)
#define RXC(x)         CAT(RXC,x)
#define RXCIE(x)       CAT(RXCIE,x)
#define UCSZ(x,y)      CAT2(UCSZ,x,y)
#define UBRR(x,y)      CAT2(UBRR,x,y)
#define UDRE(x)        CAT(UDRE,x)
#define UDRIE(x)       CAT(UDRIE,x)
#define UDR(x)         CAT(UDR,x)
#define TCNT(x)        CAT(TCNT,x)
#define TIMSK(x)       CAT(TIMSK,x)
#define TCCR(x,y)      CAT2(TCCR,x,y)
#define COM(x,y)       CAT2(COM,x,y)
#define OCR(x,y)       CAT2(OCR,x,y)
#define CS(x,y)        CAT2(CS,x,y)
#define WGM(x,y)       CAT2(WGM,x,y)
#define OCIE(x,y)      CAT2(OCIE,x,y)
#define COMPVECT(x)    CAT2(TIMER,x,_COMPA_vect)
#define UDREVECT(x)    CAT2(USART,x,_UDRE_vect)
#define RXVECT(x)      CAT2(USART,x,_RX_vect)
#endif

/** \} */

/**
 * \name Pin macros
 * \brief These macros convert the platform-specific pin defines into names and functions
 *       that the source code can directly use.
 * \{
 */
#if defined(__AVR_ATmega128RFA1__)

#define hal_set_rst_low( )    ( TRXPR &= ~( 1 << TRXRST ) ) /**< This macro pulls the RST pin low. */
#define hal_set_rst_high( )   ( TRXPR |= ( 1 << TRXRST ) ) /**< This macro pulls the RST pin high. */
#define hal_set_slptr_high( ) ( TRXPR |= ( 1 << SLPTR ) )      /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low( )  ( TRXPR &= ~( 1 << SLPTR ) )     /**< This macro pulls the SLP_TR pin low. */
//#define hal_get_slptr( ) (    ( TRXPR & ( 1 << SLPTR ) ) >> SLPTR )  /**< Read current state of the SLP_TR pin (High/Low). */
#define hal_get_slptr( )      ( TRXPR & ( 1 << SLPTR ) )  /**< Read current state of the SLP_TR pin (High/Low). */

#else
#define SLP_TR                SLPTRPIN            /**< Pin number that corresponds to the SLP_TR pin. */
#define DDR_SLP_TR            DDR( SLPTRPORT )    /**< Data Direction Register that corresponds to the port where SLP_TR is connected. */
#define PORT_SLP_TR           PORT( SLPTRPORT )   /**< Port (Write Access) where SLP_TR is connected. */
#define PIN_SLP_TR            PIN( SLPTRPORT )    /**< Pin (Read Access) where SLP_TR is connected. */
#define hal_set_slptr_high( ) ( PORT_SLP_TR |= ( 1 << SLP_TR ) )      /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low( )  ( PORT_SLP_TR &= ~( 1 << SLP_TR ) )     /**< This macro pulls the SLP_TR pin low. */
//#define hal_get_slptr( ) (    ( PIN_SLP_TR & ( 1 << SLP_TR ) ) >> SLP_TR )  /**< Read current state of the SLP_TR pin (High/Low). */
#define hal_get_slptr( )      ( PIN_SLP_TR & ( 1 << SLP_TR ) )   /**< Read current state of the SLP_TR pin (High/Low). */
#define RST                   RSTPIN              /**< Pin number that corresponds to the RST pin. */
#define DDR_RST               DDR( RSTPORT )      /**< Data Direction Register that corresponds to the port where RST is */
#define PORT_RST              PORT( RSTPORT )     /**< Port (Write Access) where RST is connected. */
#define PIN_RST               PIN( RSTPORT /* BUG? */)      /**< Pin (Read Access) where RST is connected. */
#define hal_set_rst_high( )   ( PORT_RST |= ( 1 << RST ) )  /**< This macro pulls the RST pin high. */
#define hal_set_rst_low( )    ( PORT_RST &= ~( 1 << RST ) ) /**< This macro pulls the RST pin low. */
#define hal_get_rst( )        ( ( PIN_RST & ( 1 << RST )  ) >> RST )  /**< Read current state of the RST pin (High/Low). */
#define HAL_SS_PIN            SSPIN               /**< The slave select pin. */
#define HAL_SCK_PIN           SCKPIN              /**< Data bit for SCK. */
#define HAL_MOSI_PIN          MOSIPIN
#define HAL_MISO_PIN          MISOPIN
#define HAL_PORT_SPI          PORT( SPIPORT )     /**< The SPI module is located on PORTB. */
#define HAL_PORT_SS            PORT( SSPORT )
#define HAL_PORT_SCK           PORT( SCKPORT )
#define HAL_PORT_MOSI          PORT( MOSIPORT )     /**< The SPI module uses GPIO might be split on different ports. */
#define HAL_PORT_MISO          PORT( MISOPORT )     /**< The SPI module uses GPIO might be split on different ports. */
#define HAL_DDR_SPI           DDR( SPIPORT )      /**< Data Direction Register for PORTB. */
#define HAL_DDR_SS             DDR( SSPORT )      /**< Data Direction Register for MISO GPIO pin. */
#define HAL_DDR_SCK            DDR( SCKPORT )      /**< Data Direction Register for MISO GPIO pin. */
#define HAL_DDR_MOSI           DDR( MOSIPORT )      /**< Data Direction Register for MISO GPIO pin. */
#define HAL_DDR_MISO           DDR( MISOPORT )      /**< Data Direction Register for MOSI GPIO pin. */
#define HAL_DD_SS             SSPIN               /**< Data Direction bit for SS. */
#define HAL_DD_SCK            SCKPIN              /**< Data Direction bit for SCK. */
#define HAL_DD_MOSI           MOSIPIN             /**< Data Direction bit for MOSI. */
#define HAL_DD_MISO           MISOPIN             /**< Data Direction bit for MISO. */
#endif /* defined(__AVR_ATmega128RFA1__) */

/** \} */


#define HAL_SS_HIGH( ) (HAL_PORT_SS |= ( 1 << HAL_SS_PIN )) /**< MACRO for pulling SS high. */
#define HAL_SS_LOW( )  (HAL_PORT_SS &= ~( 1 << HAL_SS_PIN )) /**< MACRO for pulling SS low. */

/** \brief Macros defined for HAL_TIMER1.
 *
 *  These macros are used to define the correct setupt of the AVR's Timer1, and
 *  to ensure that the hal_get_system_time function returns the system time in
 *  symbols (16 us ticks).
 */

#if defined(__AVR__)
#if ( F_CPU == 16000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS12 ) )
    #define HAL_US_PER_SYMBOL ( 1 )
    #define HAL_SYMBOL_MASK   ( 0xFFFFffff )
#elif ( F_CPU == 8000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
    #define HAL_US_PER_SYMBOL ( 2 )
    #define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#elif ( F_CPU == 4000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) | ( 1 << CS10 ) )
    #define HAL_US_PER_SYMBOL ( 1 )
    #define HAL_SYMBOL_MASK   ( 0xFFFFffff )
#elif ( F_CPU == 1000000UL )
    #define HAL_TCCR1B_CONFIG ( ( 1 << ICES1 ) | ( 1 << CS11 ) )
    #define HAL_US_PER_SYMBOL ( 2 )
    #define HAL_SYMBOL_MASK   ( 0x7FFFffff )
#else
    #error "Clock speed not supported."
#endif

#if HARWARE_REVISION == ZIGBIT
// IRQ E5 for Zigbit example
#define RADIO_VECT INT5_vect
#define HAL_ENABLE_RADIO_INTERRUPT( ) { ( EIMSK |= ( 1 << INT5 ) ) ; EICRB |= 0x0C ; PORTE &= ~(1<<PE5);  DDRE &= ~(1<<DDE5); }
#define HAL_DISABLE_RADIO_INTERRUPT( ) ( EIMSK &= ~( 1 << INT5 ) )
#else
#define RADIO_VECT TIMER1_CAPT_vect
// Raven and Jackdaw
#define HAL_ENABLE_RADIO_INTERRUPT( ) ( TIMSK1 |= ( 1 << ICIE1 ) )
#define HAL_DISABLE_RADIO_INTERRUPT( ) ( TIMSK1 &= ~( 1 << ICIE1 ) )
#endif

#define HAL_ENABLE_OVERFLOW_INTERRUPT( ) ( TIMSK1 |= ( 1 << TOIE1 ) )
#define HAL_DISABLE_OVERFLOW_INTERRUPT( ) ( TIMSK1 &= ~( 1 << TOIE1 ) )

/** This macro will protect the following code from interrupts.*/
#define HAL_ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )

/** This macro must always be used in conjunction with HAL_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define HAL_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;}

#else /* MULLE */

#define HAL_ENABLE_RADIO_INTERRUPT( ) ( INT1IC.BYTE |= 1 )
#define HAL_DISABLE_RADIO_INTERRUPT( ) ( INT1IC.BYTE &= ~(1) )

#define HAL_ENABLE_OVERFLOW_INTERRUPT( ) ( TB4IC.BYTE = 1 )
#define HAL_DISABLE_OVERFLOW_INTERRUPT( ) ( TB4IC.BYTE = 0 )

/** This macro will protect the following code from interrupts.*/
#define HAL_ENTER_CRITICAL_REGION( ) MULLE_ENTER_CRITICAL_REGION( )

/** This macro must always be used in conjunction with HAL_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define HAL_LEAVE_CRITICAL_REGION( ) MULLE_LEAVE_CRITICAL_REGION( )

#endif /* !__AVR__ */


/** \brief  Enable the interrupt from the radio transceiver.
 */
#define hal_enable_trx_interrupt( ) HAL_ENABLE_RADIO_INTERRUPT( )

/** \brief  Disable the interrupt from the radio transceiver.
 *
 *  \retval 0 if the pin is low, 1 if the pin is high.
 */
#define hal_disable_trx_interrupt( ) HAL_DISABLE_RADIO_INTERRUPT( )
/*============================ TYPDEFS =======================================*/
/*============================ PROTOTYPES ====================================*/
/*============================ MACROS ========================================*/
/** \name Macros for radio operation.
 * \{ 
 */
#define HAL_BAT_LOW_MASK       ( 0x80 ) /**< Mask for the BAT_LOW interrupt. */
#define HAL_TRX_UR_MASK        ( 0x40 ) /**< Mask for the TRX_UR interrupt. */
#define HAL_TRX_END_MASK       ( 0x08 ) /**< Mask for the TRX_END interrupt. */
#define HAL_RX_START_MASK      ( 0x04 ) /**< Mask for the RX_START interrupt. */
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) /**< Mask for the PLL_UNLOCK interrupt. */
#define HAL_PLL_LOCK_MASK      ( 0x01 ) /**< Mask for the PLL_LOCK interrupt. */

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */
/** \} */
/*============================ TYPDEFS =======================================*/
/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
} hal_rx_frame_t;

/** RX_START event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_rx_start_event_handler(). */
typedef void (*hal_rx_start_isr_event_handler_t)(uint32_t const isr_timestamp, uint8_t const frame_length);

/** RRX_END event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_trx_end_event_handler(). */
typedef void (*hal_trx_end_isr_event_handler_t)(uint32_t const isr_timestamp);

typedef void (*rx_callback_t) (uint16_t data);

/*============================ PROTOTYPES ====================================*/
void hal_init( void );

void hal_reset_flags( void );
uint8_t hal_get_bat_low_flag( void );
void hal_clear_bat_low_flag( void );

hal_trx_end_isr_event_handler_t hal_get_trx_end_event_handler( void );
void hal_set_trx_end_event_handler( hal_trx_end_isr_event_handler_t trx_end_callback_handle );
void hal_clear_trx_end_event_handler( void );

hal_rx_start_isr_event_handler_t hal_get_rx_start_event_handler( void );
void hal_set_rx_start_event_handler( hal_rx_start_isr_event_handler_t rx_start_callback_handle );
void hal_clear_rx_start_event_handler( void );

uint8_t hal_get_pll_lock_flag( void );
void hal_clear_pll_lock_flag( void );

/* Hack for atmega128rfa1 with integrated radio. Access registers directly, not through SPI */
#if defined(__AVR_ATmega128RFA1__)
//#define hal_register_read(address) _SFR_MEM8((uint16_t)address)
#define hal_register_read(address) address
uint8_t hal_subregister_read( uint16_t address, uint8_t mask, uint8_t position );
void hal_subregister_write( uint16_t address, uint8_t mask, uint8_t position,
                            uint8_t value );

//#define hal_register_write(address, value) _SFR_MEM8((uint16_t)address)=value
#define hal_register_write(address, value) address=value
//#define hal_subregister_read( address, mask, position ) (_SFR_MEM8((uint16_t)address)&mask)>>position
//#define hal_subregister_read1( address, mask, position ) (address&mask)>>position
//#define hal_subregister_write( address, mask, position, value ) address=(address<<position)&mask
#else
uint8_t hal_register_read( uint8_t address );
void hal_register_write( uint8_t address, uint8_t value );
uint8_t hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position );
void hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position,
                            uint8_t value );
#endif



//void hal_frame_read(hal_rx_frame_t *rx_frame, rx_callback_t rx_callback);
/* For speed RF230BB does not use a callback */
void hal_frame_read(hal_rx_frame_t *rx_frame);
void hal_frame_write( uint8_t *write_buffer, uint8_t length );
void hal_sram_read( uint8_t address, uint8_t length, uint8_t *data );
void hal_sram_write( uint8_t address, uint8_t length, uint8_t *data );
/* Number of receive buffers in RAM. */
#ifndef RF230_CONF_RX_BUFFERS
#define RF230_CONF_RX_BUFFERS 1
#endif

#endif
#endif
#endif
/** @} */
/*EOF*/
