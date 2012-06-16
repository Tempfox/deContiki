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

#ifndef _DP83848_DEFINE_H
#define _DP83848_DEFINE_H

// DAVICOM PHYSICAL LAYER TRANSCEIVER DP83848

//-----------------------------------------------------------------------------
///         Definitions
//-----------------------------------------------------------------------------

#define DP83848_BMCR        0x00   // Basic Mode Control Register
#define DP83848_BMSR        0x01   // Basic Mode Status Register
#define DP83848_PHYID1      0x02   // PHY Idendifier Register 1
#define DP83848_PHYID2      0x03   // PHY Idendifier Register 2
#define DP83848_ANAR        0x04   // Auto_Negotiation Advertisement Register
#define DP83848_ANLPAR      0x05   // Auto_negotiation Link Partner Ability Register
#define DP83848_ANER        0x06   // Auto-negotiation Expansion Register
#define DP83848_ANNPTR      0x07   // Auto-negotiation Next Page Transmit Register
#define DP83848_PHYSTS      0x10   // PHY Status Register
#define DP83848_MICR      	0x11   // MII Interrupt Control Register
#define DP83848_MISR      	0x12   // MII Interrupt Status and Misc. Control Register
#define DP83848_FCSCR      	0x14   //
#define DP83848_RECR      	0x15   //
#define DP83848_PCSR      	0x16   //
#define DP83848_RBR      	0x17   //
#define DP83848_LEDCR      	0x18   //
#define DP83848_PHYCR      	0x19   //
#define DP83848_10BTCSR     0x1A   // 10BASE-T Configuration and Satus Register
#define DP83848_CDCTRL1     0x1B
#define DP83848_EDCR	    0x1D


// Basic Mode Control Register (BMCR)
// Bit definitions: DP83848_BMCR
#define DP83848_RESET             (1 << 15) // 1= Software Reset; 0=Normal Operation
#define DP83848_LOOPBACK          (1 << 14) // 1=loopback Enabled; 0=Normal Operation
#define DP83848_SPEED_SELECT      (1 << 13) // 1=100Mbps; 0=10Mbps
#define DP83848_AUTONEG           (1 << 12) // Auto-negotiation Enable
#define DP83848_POWER_DOWN        (1 << 11) // 1=Power down 0=Normal operation
#define DP83848_ISOLATE           (1 << 10) // 1 = Isolates 0 = Normal operation
#define DP83848_RESTART_AUTONEG   (1 << 9)  // 1 = Restart auto-negotiation 0 = Normal operation
#define DP83848_DUPLEX_MODE       (1 << 8)  // 1 = Full duplex operation 0 = Normal operation
#define DP83848_COLLISION_TEST    (1 << 7)  // 1 = Collision test enabled 0 = Normal operation
//      Reserved                  6 to 0   // Read as 0, ignore on write

// Basic Mode Status Register (BMSR)
// Bit definitions: DP83848_BMSR
#define DP83848_100BASE_T4        (1 << 15) // 100BASE-T4 Capable
#define DP83848_100BASE_TX_FD     (1 << 14) // 100BASE-TX Full Duplex Capable
#define DP83848_100BASE_T4_HD     (1 << 13) // 100BASE-TX Half Duplex Capable
#define DP83848_10BASE_T_FD       (1 << 12) // 10BASE-T Full Duplex Capable
#define DP83848_10BASE_T_HD       (1 << 11) // 10BASE-T Half Duplex Capable
//      Reserved                  10 to 7  // Read as 0, ignore on write
#define DP83848_MF_PREAMB_SUPPR   (1 << 6)  // MII Frame Preamble Suppression
#define DP83848_AUTONEG_COMP      (1 << 5)  // Auto-negotiation Complete
#define DP83848_REMOTE_FAULT      (1 << 4)  // Remote Fault
#define DP83848_AUTONEG_ABILITY   (1 << 3)  // Auto Configuration Ability
#define DP83848_LINK_STATUS       (1 << 2)  // Link Status
#define DP83848_JABBER_DETECT     (1 << 1)  // Jabber Detect
#define DP83848_EXTEND_CAPAB      (1 << 0)  // Extended Capability

// PHY ID Identifier Register
// definitions: DP83848_PHYID1
#define DP83848_PHYID1_OUI         0x606E   // OUI: Organizationally Unique Identifier
#define DP83848_LSB_MASK             0x3F
#define DP83848_ID             0x0181b8a0
#define DP83848_OUI_MSB            0x2000
#define DP83848_OUI_LSB              0x2E

// Auto-negotiation Advertisement Register (ANAR)
// Auto-negotiation Link Partner Ability Register (ANLPAR)
// Bit definitions: DP83848_ANAR, DP83848_ANLPAR
#define DP83848_NP               (1 << 15) // Next page Indication
#define DP83848_ACK              (1 << 14) // Acknowledge
#define DP83848_RF               (1 << 13) // Remote Fault
//      Reserved                12 to 11  // Write as 0, ignore on read
#define DP83848_FCS              (1 << 10) // Flow Control Support
#define DP83848_T4               (1 << 9)  // 100BASE-T4 Support
#define DP83848_TX_FDX           (1 << 8)  // 100BASE-TX Full Duplex Support
#define DP83848_TX_HDX           (1 << 7)  // 100BASE-TX Support
#define DP83848_10_FDX           (1 << 6)  // 10BASE-T Full Duplex Support
#define DP83848_10_HDX           (1 << 5)  // 10BASE-T Support
//      Selector                 4 to 0   // Protocol Selection Bits
#define DP83848_AN_IEEE_802_3      0x0001

// Auto-negotiation Expansion Register (ANER)
// Bit definitions: DP83848_ANER
//      Reserved                15 to 5  // Read as 0, ignore on write
#define DP83848_PDF              (1 << 4) // Local Device Parallel Detection Fault
#define DP83848_LP_NP_ABLE       (1 << 3) // Link Partner Next Page Able
#define DP83848_NP_ABLE          (1 << 2) // Local Device Next Page Able
#define DP83848_PAGE_RX          (1 << 1) // New Page Received
#define DP83848_LP_AN_ABLE       (1 << 0) // Link Partner Auto-negotiation Able

// Specified Configuration Register (DSCR)
// Bit definitions: DP83848_DSCR
#define DP83848_BP4B5B           (1 << 15) // Bypass 4B5B Encoding and 5B4B Decoding
#define DP83848_BP_SCR           (1 << 14) // Bypass Scrambler/Descrambler Function
#define DP83848_BP_ALIGN         (1 << 13) // Bypass Symbol Alignment Function
#define DP83848_BP_ADPOK         (1 << 12) // BYPASS ADPOK
#define DP83848_REPEATER         (1 << 11) // Repeater/Node Mode
#define DP83848_TX               (1 << 10) // 100BASE-TX Mode Control
#define DP83848_FEF              (1 << 9)  // Far end Fault enable
#define DP83848_RMII_ENABLE      (1 << 8)  // Reduced MII Enable
#define DP83848_F_LINK_100       (1 << 7)  // Force Good Link in 100Mbps
#define DP83848_SPLED_CTL        (1 << 6)  // Speed LED Disable
#define DP83848_COLLED_CTL       (1 << 5)  // Collision LED Enable
#define DP83848_RPDCTR_EN        (1 << 4)  // Reduced Power Down Control Enable
#define DP83848_SM_RST           (1 << 3)  // Reset State Machine
#define DP83848_MFP_SC           (1 << 2)  // MF Preamble Suppression Control
#define DP83848_SLEEP            (1 << 1)  // Sleep Mode
#define DP83848_RLOUT            (1 << 0)  // Remote Loopout Control

// Specified Configuration and Status Register (DSCSR)
// Bit definitions: DP83848_DSCSR
#define DP83848_100FDX           (1 << 15) // 100M Full Duplex Operation Mode
#define DP83848_100HDX           (1 << 14) // 100M Half Duplex Operation Mode
#define DP83848_10FDX            (1 << 13) // 10M Full Duplex Operation Mode
#define DP83848_10HDX            (1 << 12) // 10M Half Duplex Operation Mode

// 10BASE-T Configuration/Status (10BTCSR)
// Bit definitions: DP83848_10BTCSR
//      Reserved                18 to 15  // Read as 0, ignore on write
#define DP83848_LP_EN            (1 << 14) // Link Pulse Enable
#define DP83848_HBE              (1 << 13) // Heartbeat Enable
#define DP83848_SQUELCH          (1 << 12) // Squelch Enable
#define DP83848_JABEN            (1 << 11) // Jabber Enable
#define DP83848_10BT_SER         (1 << 10) // 10BASE-T GPSI Mode
//      Reserved                 9 to  1  // Read as 0, ignore on write
#define DP83848_POLR             (1 << 0)  // Polarity Reversed

// Specified Interrupt Register
// Bit definitions: DP83848_MDINTR
#define DP83848_INTR_PEND        (1 << 15) // Interrupt Pending
//      Reserved                14 to 12  // Reserved
#define DP83848_FDX_MASK         (1 << 11) // Full-duplex Interrupt Mask
#define DP83848_SPD_MASK         (1 << 10) // Speed Interrupt Mask
#define DP83848_LINK_MASK        (1 << 9)  // Link Interrupt Mask
#define DP83848_INTR_MASK        (1 << 8)  // Master Interrupt Mask
//      Reserved                 7 to 5   // Reserved
#define DP83848_FDX_CHANGE       (1 << 4)  // Duplex Status Change Interrupt
#define DP83848_SPD_CHANGE       (1 << 3)  // Speed Status Change Interrupt
#define DP83848_LINK_CHANGE      (1 << 2)  // Link Status Change Interrupt
//      Reserved                      1   // Reserved
#define DP83848_INTR_STATUS      (1 << 0)  // Interrupt Status

#endif // #ifndef _DP83848_DEFINE_H

