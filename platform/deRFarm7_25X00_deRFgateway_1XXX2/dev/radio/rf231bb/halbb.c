/*   Copyright (c) 2012, Swedish Institute of Computer Science
 *  All rights reserved. 
 *
 *   All rights reserved.
 *
 *   Adapted for rf231 by: Joerg Wolf <gwynpen@googlemail.com>
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
 *
 * 
*/

/**
 *   \addtogroup wireless
 *  @{
*/

/**
 *   \defgroup hal RF231 hardware level drivers
 *   @{
 */

/**
 *  \file
 *  This file contains low-level radio driver code.
 *  This version is optimized for use with the "barebones" RF231bb driver,
 *  which communicates directly with the contiki core MAC layer.
 *  It is optimized for speed at the expense of generality.
 */

/*============================ INCLUDE =======================================*/
#include "contiki-conf.h"

#if DEBUGFLOWSIZE
	extern uint8_t debugflowsize,debugflow[DEBUGFLOWSIZE];
	#define DEBUGFLOW(c) if (debugflowsize<(DEBUGFLOWSIZE-1)) debugflow[debugflowsize++]=c
#else
	#define DEBUGFLOW(c)
#endif

#include <stdlib.h>
#include <pio/pio.h>
#include <pio/pio_it.h>


#include "hal.h"

extern uint8_t rf231_last_correlation, rf231_last_rssi, rf231_smallest_rssi;

#include <AT91SAM7X512.h>
#include "at86rf231_registermap.h"

/*============================ MACROS =====================================*/

#define SCBR_FIELD_POS_IN_CSR_REG          (8)

/* Value in us used for delay between poll attempts for transceiver access. */
#define TRX_POLL_WAIT_TIME_US       (100)

/* Ratio between max time of TR1 / transceiver poll delay */
#define P_ON_TO_CLKM_ATTEMPTS       ((uint8_t) \
                                     (P_ON_TO_CLKM_AVAILABLE_MAX_US / TRX_POLL_WAIT_TIME_US))

/* Ratio between max time of TR2 / transceiver poll delay */
#define SLEEP_TO_TRX_OFF_ATTEMPTS   ((uint8_t) \
                                     (SLEEP_TO_TRX_OFF_MAX_US / TRX_POLL_WAIT_TIME_US))

/*============================ VARIABLES =====================================*/

/* === Types ============================================================== */

/**
 * This is a typedef of the function which is called from the transceiver ISR
 */
typedef void (*irq_handler_t)(void);

/* These link to the RF231BB driver in rf231bb.c */
void* rf231_interrupt(void);

extern hal_rx_frame_t rxframe[RF231_CONF_RX_BUFFERS];
extern uint8_t rxframe_head, rxframe_tail;

/* rf231interruptflag can be printed in the main idle loop for debugging */
#define DEBUG 1
#if DEBUG
volatile char rf231interruptflag;
#define INTERRUPTDEBUG(arg) rf231interruptflag=arg
#else
#define INTERRUPTDEBUG(arg)
#endif

/* === Globals ============================================================== */

/* pin references */
const Pin pinIRQ  = TRX_IRQ_PIN;
const Pin pinRXTS = TRX_TS_IRQ_PIN;

/* fwd decl */
void pal_trx_irq_init( void* trx_irq_cb );
/*
 * Function pointers to store the callback function of
 * the transceiver interrupt
 */

/** Function pointer to store callback for transceiver main (TX_END) interrupt. */
#ifndef USE_SYS
static irq_handler_t irq_hdl_trx;
#endif /* USE_SYS */

#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) || (defined DOXYGEN)
/** Function pointer to store callback for transceiver timestamp (RX_START) interrupt. */
static irq_handler_t irq_hdl_trx_tstamp;
#endif


/** \brief This is a file internal variable that contains the 16 MSB of the
 *         system time.
 *
 *         The system time (32-bit) is the current time in microseconds. For the
 *         AVR microcontroller implementation this is solved by using a 16-bit
 *         timer (Timer1) with a clock frequency of 1MHz. The hal_system_time is
 *         incremented when the 16-bit timer overflows, representing the 16 MSB.
 *         The timer value it self (TCNT1) is then the 16 LSB.
 *
 *  \see hal_get_system_time
 */
static uint16_t hal_system_time = 0;

/**
 * Current state of the transceiver.
 */
uint8_t tal_trx_status;
 
/**
 * @brief Initializes the transceiver interface
 *
 * This function initializes the transceiver interface.
 * This board uses SPI1.
 */
void trx_interface_init(void)
{
    /*
     * The PIO control is disabled for the SPI pins MISO, MOSI, SCK and the
     * SPI module control for these pins is enabled. The pins are multiplexed
     * by the PIO A and under control of peripheral B
     */
    PIN_SET_AS_PERIPHERAL_B(MISO, PIO_A);
    PIN_SET_AS_PERIPHERAL_B(MOSI, PIO_A);
    PIN_SET_AS_PERIPHERAL_B(SCK, PIO_A);

    /*
     * Select line will be used as a GPIO. The controller recognizes 1 cycle
     * of SPI transaction as 8 bit, hence deactivates the chip select after 1
     * cycle. But the transceiver needs the chip select to be active for two
     * cycles (In one cycle the transceiver gets to know about the address of
     * register or memory location and the kind of operation to be performed
     * on that memory location. And in the second cycle its performs the
     * specified operation). To achieve this, the chip select line will be
     * manually pulled low and high (after the transaction). Hence the SEL line
     * is configured as PIO and the SPI control of SEL is disabled.
     */

    /* Set SEL as output pin. */
    PIN_SET_AS_PIO_OUTPUT(SEL, PIO_A);

    /*
     * Used peripheral interface is SPI1.
     * The clock to the utilized SPI1 peripheral is enabled.
     */
    AT91C_BASE_PMC->PMC_PCER = _BV(AT91C_ID_SPI1);

    /* The controller is configured to be master. */
    AT91C_BASE_SPI1->SPI_MR = (AT91C_SPI_MSTR              // be master
                        | AT91C_SPI_MODFDIS                // mode fault detection disabled
                        | (AT91C_SPI_PCS & SS_ENABLE));    // CS

    /*
     * SPI mode 0 (clock polarity = 0 and clock phase = 1) is selected. The
     * transaction width is set to 8 bits. The SCBR register of the SPI module
     * is written with the divider required for generation of the baud rate. It
     * is calculated as follows. Baud rate = MCK / SPI_BR_DIVIDER.
     */
    AT91C_BASE_SPI1->SPI_CSR[2] = (AT91C_SPI_NCPHA |
                        (AT91C_SPI_BITS & AT91C_SPI_BITS_8) |
                        (AT91C_SPI_SCBR & (SPI_BR_DIVIDER << SCBR_FIELD_POS_IN_CSR_REG)));

    /* The SPI is enabled. */
    AT91C_BASE_SPI1->SPI_CR = AT91C_SPI_SPIEN;
}
/**
 * @brief Initializes the transceiver
 *
 * This function is called to initialize the transceiver.
 *
 * @return MAC_SUCCESS  if the transceiver state is changed to TRX_OFF and the
 *                 current device part number and version number are correct;
 *         FAILURE otherwise
 */
static uint8_t trx_init(void)
{
    uint8_t trx_status;
    uint8_t poll_counter = 0;

    RST_HIGH();
    SLP_TR_LOW();

    /* Wait typical time of timer TR1. */
    delay_us(P_ON_TO_CLKM_AVAILABLE_TYP_US);

    /* Apply reset pulse */
    RST_LOW();
    delay_us(RST_PULSE_WIDTH_US);
    RST_HIGH();

    /* Verify that TRX_OFF can be written */
    do
    {
        /* Wait not more than max. value of TR1. */
        if (poll_counter == P_ON_TO_CLKM_ATTEMPTS)
        {
            return(-1/* FAILURE*/);
        }
        /* Wait a short time interval. */
        delay_us(TRX_POLL_WAIT_TIME_US);
        poll_counter++;
        /* Check if AT86RF231 is connected; omit manufacturer id check */
    } while ((hal_register_read(RG_VERSION_NUM) != AT86RF231_VERSION_NUM) ||
             (hal_register_read(RG_PART_NUM) != AT86RF231_PART_NUM));

    /* Verify that TRX_OFF can be written */
    hal_register_write(RG_TRX_STATE, CMD_TRX_OFF);

    /* Verify that the trx has reached TRX_OFF. */
    poll_counter = 0;
    do
    {
        /* Wait a short time interval. */
        delay_us(TRX_POLL_WAIT_TIME_US);

        trx_status = (uint8_t)hal_subregister_read(SR_TRX_STATUS);

        /* Wait not more than max. value of TR2. */
        if (poll_counter == SLEEP_TO_TRX_OFF_ATTEMPTS)
        {
//#if (DEBUG > 0)
//            pal_alert();
//#endif
            return(-1/* FAILURE*/);
        }
        poll_counter++;
    } while (trx_status != TRX_OFF);

    tal_trx_status = TRX_OFF;

    return(0);
}

/* *************** HAL API functions **************************************** */
void hal_init( void ) {
	/* Reset variables used in file. */
	hal_system_time = 0;

	//  hal_reset_flags();
	/*
	 * The clock to PIO A and PIO B are enabled. This is necessary, as only
	 * when the clock is provided the PIO starts functioning.
	 */
	PERIPHERAL_CLOCK_ENABLE(AT91C_ID_PIOA);
	PERIPHERAL_CLOCK_ENABLE(AT91C_ID_PIOB);

	/* The following pins are output pins.  */
	PIN_SET_AS_PIO_OUTPUT(RST, PIO_A);
	PIN_SET_AS_PIO_OUTPUT(SLP_TR, PIO_A);

	trx_interface_init();

//@@@!?	timer_init();

	trx_init();
// TODO @@@jwg	internal_tal_reset();

	pal_trx_irq_init((void*) rf231_interrupt);
	pal_trx_irq_en();   /* Enable main transceiver interrupt. */
}

/*----------------------------------------------------------------------------*/
/** \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf231_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 */
uint8_t
hal_register_read( uint8_t address ) {
	// derived frompal_trx_access
	uint8_t register_value;

	ENTER_CRITICAL_REGION();

	// Prepare the command byte
	address |= READ_ACCESS_COMMAND;

	// Start SPI transaction by pulling SEL low
	SS_LOW();

	// Send the write command byte
	SPI_WRITE(address);

	// Done to clear the RDRF bit in the SPI status register, which will be set
	// as a result of reception of some data from the transceiver as a result
	// of SPI write operation done above.
	SPI_READ(register_value);

	// Do dummy write for initiating SPI read
	SPI_WRITE(SPI_DUMMY_VALUE);

	// Read the byte received
	SPI_READ(register_value);

	// Stop the SPI transaction by setting SEL high
	SS_HIGH();

    LEAVE_CRITICAL_REGION();

	return (register_value);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf231_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 */
void
hal_register_write( uint8_t address, uint8_t value ) {

	ENTER_CRITICAL_REGION();

    // Prepare the command byte
    address |= WRITE_ACCESS_COMMAND;

    // Start SPI transaction by pulling SEL low
    SS_LOW();

    // Send the Read command byte
    SPI_WRITE(address);

    // Write the byte in the transceiver data register
    SPI_WRITE(value);

    // Stop the SPI transaction by setting SEL high
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf231_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 */
uint8_t
hal_subregister_read( uint8_t address, uint8_t mask, uint8_t position ) {
	/* Read current register value and mask out subregister. */
	uint8_t register_value = hal_register_read(address);
	register_value &= mask;
	register_value >>= position; /* Align subregister value. */

	return(register_value);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf231_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 */
void
hal_subregister_write( uint8_t address, uint8_t mask, uint8_t position, uint8_t value) {
	/* Read current register value and mask area outside the subregister. */
	volatile uint8_t register_value = hal_register_read(address);
	register_value &= ~mask;

	/* Start preparing the new subregister value. shift in place and mask. */
	value <<= position;
	value &= mask;

	value |= register_value; /* Set the new subregister value. */

	/* Write the modified register value. */
	hal_register_write(address, value);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will upload a frame from the radio transceiver's frame
 *          buffer.
 *
 *          If the frame currently available in the radio transceiver's frame buffer
 *          is out of the defined bounds. Then the frame length, lqi value and crc
 *          be set to zero. This is done to indicate an error.
 *          This version is optimized for use with contiki RF230BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *  \param  rx_frame    Pointer to the data structure where the frame is stored.
 *  \param  rx_callback Pointer to callback function for receiving one byte at a time.
 */
void
hal_frame_read( uint8_t *data, uint8_t length )
{
    uint8_t dummy_rx_data;

    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPI_WRITE(TRX_CMD_FR);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
    SPI_READ(dummy_rx_data);

    /*
     * Done to avoid compiler warning about variable being not used after
     * setting.
     */
    dummy_rx_data = dummy_rx_data;

    /* Initiate DMA transfer for ARM7 (synchronous read and write). */

    /* Set DLYBCT on, (32 * DLYBCT / MCK), to generate time t5
     * of transceiver SPI spec. */
    AT91C_BASE_SPI_USED->SPI_CSR[2] |= AT91C_SPI_DLYBCT_1;

    /* Disable both read and write. */
    AT91C_BASE_SPI_USED->SPI_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;

    /*
     * Comment from ARM example code:
     * "MOSI should hold high during read, or there will be wrong data
     * in received data."
     */
    memset(data, 0xFF,length);

    AT91C_BASE_SPI_USED->SPI_RPR = AT91C_BASE_SPI_USED->SPI_TPR = (uint32_t)data;
    AT91C_BASE_SPI_USED->SPI_RCR = AT91C_BASE_SPI_USED->SPI_TCR = length;

    /* Enable read and write. */
    AT91C_BASE_SPI_USED->SPI_PTCR = AT91C_PDC_RXTEN | AT91C_PDC_TXTEN;

    /* Wait for end of read; send counter should not matter. */
    while (AT91C_BASE_SPI_USED->SPI_RCR);

    /* DLYBCT off */
    AT91C_BASE_SPI_USED->SPI_CSR[2] &= ~AT91C_SPI_DLYBCT;

    /* Stop the SPI transaction by setting SEL high. */
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
void
hal_frame_write( uint8_t *write_buffer, uint8_t length )
{
	   ENTER_CRITICAL_REGION();

	    /* Start SPI transaction by pulling SEL low */
	    SS_LOW();

	    /* Send the command byte */
	    SPI_WRITE(TRX_CMD_FW);

	    /* Set DLYBCT on, 32 * DLYBCT / MCK, to generate time t5
	     * of transceiver SPI spec. */
	    AT91C_BASE_SPI_USED->SPI_CSR[2] |= AT91C_SPI_DLYBCT_1;

	    /* DMA transfer for ARM7 */
	    AT91C_BASE_SPI_USED->SPI_PTCR = AT91C_PDC_TXTDIS;

	    /* Download to the Frame Buffer.
	      * When the FCS is autogenerated there is no need to transfer the last two bytes
	      * since they will be overwritten.
	      */
	 #if !RF231_CONF_CHECKSUM
	     length -= 2;
	 #endif
	    /* Start DMA. */
	    AT91C_BASE_SPI_USED->SPI_TPR = (uint32_t)write_buffer;
	    AT91C_BASE_SPI_USED->SPI_TCR = length;
	    AT91C_BASE_SPI_USED->SPI_PTCR = AT91C_PDC_TXTEN;

	    /* Wait for finishing the transfer. */
	    while (!(AT91C_BASE_SPI_USED->SPI_SR & AT91C_SPI_TDRE) ||
	           !(AT91C_BASE_SPI_USED->SPI_SR & AT91C_SPI_TXEMPTY));

	    /* DLYBCT off. */
	    AT91C_BASE_SPI_USED->SPI_CSR[2] &= ~AT91C_SPI_DLYBCT;

	    /* Stop the SPI transaction by setting SEL high. */
	    SS_HIGH();

	    LEAVE_CRITICAL_REGION();
}


/*
 *
 * Separate RF231 has a single radio interrupt and the source must be read from the IRQ_STATUS register
*/
void
hal_trx_ISR( void ) {
	/*The following code reads the current system time. This is done by first
	 reading the hal_system_time and then adding the 16 LSB directly from the
	 hardware counter.
	 */
	//    uint32_t isr_timestamp = hal_system_time;
	//    isr_timestamp <<= 16;
	//    isr_timestamp |= HAL_TICK_UPCNT(); // TODO: what if this wraps after reading hal_system_time?
	volatile uint8_t state;
	uint8_t interrupt_source; /* used after HAL_SPI_TRANSFER_OPEN/CLOSE block */

    INTERRUPTDEBUG(1);

    
    /* Using SPI bus from ISR is generally a bad idea... */
    /* Note: all IRQ are not always automatically disabled when running in ISR */
//   HAL_SPI_TRANSFER_OPEN();

    /*Read Interrupt source.*/
    /*Send Register address and read register content.*/
//    HAL_SPI_TRANSFER_WRITE(0x80 | RG_IRQ_STATUS);

    /* This is the second part of the convertion of system time to a 16 us time
       base. The division is moved here so we can spend less time waiting for SPI
       data.
     */
//   isr_timestamp /= HAL_US_PER_SYMBOL; /* Divide so that we get time in 16us resolution. */
//   isr_timestamp &= HAL_SYMBOL_MASK;

//    HAL_SPI_TRANSFER_WAIT(); /* AFTER possible interleaved processing */

#if 0 //dak
    interrupt_source = HAL_SPI_TRANSFER_READ(); /* The interrupt variable is used as a dummy read. */

    interrupt_source = HAL_SPI_TRANSFER(interrupt_source);
#else
//    interrupt_source = HAL_SPI_TRANSFER(0);
#endif
//    HAL_SPI_TRANSFER_CLOSE();

	interrupt_source = hal_subregister_read(RG_IRQ_STATUS, 0xff,0);

    /* Handle the incomming interrupt. Prioritized. */
	if ( (interrupt_source & HAL_RX_START_MASK) ) {
		INTERRUPTDEBUG(10);

		/* Save RSSI for this packet if not in extended mode, scaling to 1dB resolution */
#if !RF231_CONF_AUTOACK
#if 0  // 3-clock shift and add is faster on machines with no hardware multiply
       // While the compiler should use similar code for multiply by 3 there may be a bug with -Os in avr-gcc that calls the general subroutine
        rf230_last_rssi = hal_subregister_read(SR_RSSI);
        rf230_last_rssi = (rf230_last_rssi <<1)  + rf230_last_rssi;
#else  // Faster with 1-clock multiply. Raven and Jackdaw have 2-clock multiply so same speed while saving 2 bytes of program memory
        rf231_last_rssi = 3 * hal_subregister_read(SR_RSSI);
#endif
#endif
//       if(rx_start_callback != NULL){
//            /* Read Frame length and call rx_start callback. */
//            HAL_SPI_TRANSFER_OPEN();
//            uint8_t frame_length = HAL_SPI_TRANSFER(0x20);
//            frame_length = HAL_SPI_TRANSFER(frame_length);

//            HAL_SPI_TRANSFER_CLOSE();

//            rx_start_callback(isr_timestamp, frame_length);
//       }
	}
	else if ( interrupt_source & HAL_TRX_END_MASK ) {
		INTERRUPTDEBUG(11);
//	   if(trx_end_callback != NULL){
//       trx_end_callback(isr_timestamp);
//     }
        
		state = hal_subregister_read(SR_TRX_STATUS);

		if ( (state == BUSY_RX_AACK) || (state == RX_ON) || (state == BUSY_RX) || (state == RX_AACK_ON) ) {
			/* Received packet interrupt */
			/* Buffer the frame and call rf230_interrupt to schedule poll for rf230 receive process */
//         if (rxframe.length) break;			//toss packet if last one not processed yet
			if ( rxframe[rxframe_tail].length ) {
				INTERRUPTDEBUG(42);
			}
			else {
				INTERRUPTDEBUG(12);
			}
 
#ifdef RF230_MIN_RX_POWER		 
       /* Discard packets weaker than the minimum if defined. This is for testing miniature meshes.*/
       /* Save the rssi for printing in the main loop */
#if RF230_CONF_AUTOACK
 //       rf230_last_rssi=hal_subregister_read(SR_ED_LEVEL);
        rf230_last_rssi=hal_register_read(RG_PHY_ED_LEVEL);
#endif
        if (rf230_last_rssi >= RF230_MIN_RX_POWER) {       
#endif

        	uint8_t len = 0;
//        	hal_frame_read(&rxframe[rxframe_tail].length, 1);
        	hal_frame_read(&len, 1);
        	hal_frame_read(&rxframe[rxframe_tail].length, len);

        	rxframe_tail++;

        	if ( rxframe_tail >= RF231_CONF_RX_BUFFERS ) {
        		rxframe_tail = 0;
        	}

        	rf231_interrupt();

#ifdef RF230_MIN_RX_POWER
        }
#endif

       }
              
    } else if (interrupt_source & HAL_TRX_UR_MASK) {
		INTERRUPTDEBUG(13);
		;
	} else if (interrupt_source & HAL_PLL_UNLOCK_MASK) {
		INTERRUPTDEBUG(14);
		;
	} else if (interrupt_source & HAL_PLL_LOCK_MASK) {
		INTERRUPTDEBUG(15);
		//      hal_pll_lock_flag++;
		;
	} else if ( interrupt_source & HAL_BAT_LOW_MASK ) {
		/*  Disable BAT_LOW interrupt to prevent endless interrupts. The interrupt */
		/*  will continously be asserted while the supply voltage is less than the */
		/*  user-defined voltage threshold. */
		uint8_t trx_isr_mask = hal_register_read(RG_IRQ_MASK);
		trx_isr_mask &= ~HAL_BAT_LOW_MASK;
		hal_register_write(RG_IRQ_MASK, trx_isr_mask);
		//      hal_bat_low_flag++; /* Increment BAT_LOW flag. */
		INTERRUPTDEBUG(16);
		;
	} else {
		INTERRUPTDEBUG(99);
		;
	}
}

/*
 *
 */
void pal_trx_irq_init( void* trx_irq_cb ) {
	/* disable and clear the Interrupt source */
	DISABLE_TRX_IRQ();
	CLEAR_TRX_IRQ();

	/*
	 * Set the handler function.
	 * The handler is set before enabling the interrupt to prepare for
	 * spurious interrupts, that can pop up the moment they are enabled.
	 */
	irq_hdl_trx = (irq_handler_t)trx_irq_cb;

	/* Initialize PIO Interrupt system, since Transceiver-IRQs are PIO controlled */
	PIO_InitializeInterrupts(AT91C_AIC_PRIOR_LOWEST);

	/*
	 * The AIC is configured and updated with the mode for generating
	 * a PIO interrupt for IRQ0, which is connected to the transceiver
	 * interrupt line. Also the interrupt handler is installed for the same.
	 */
	PIN_CONFIGURE_PIO_IRQ(pinIRQ, hal_trx_ISR);

	// FIXME: necessary since the TAL state machine has changed
	// such that from now it exchanges the IRQ handler callbacks
	// but does not re-enable the IRQ itself
	ENABLE_TRX_IRQ();
}

/* EOF */
