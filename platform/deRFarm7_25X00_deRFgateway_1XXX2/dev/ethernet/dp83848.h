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

//-----------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// Implementation of DP83848 PHY driver
///
/// !Contents
///
/// Please refer to the list of functions in the #Overview# tab of this unit
/// for more detailed information.
//-----------------------------------------------------------------------------


// components/ethernet/dm9161/dm9161.h

#ifndef _DP83848_H
#define _DP83848_H

//-----------------------------------------------------------------------------
//         Headers
//-----------------------------------------------------------------------------
#include <pio/pio.h>

//-----------------------------------------------------------------------------
//         Definitions
//-----------------------------------------------------------------------------

/// The reset length setting for external reset configuration
#define DP83848_RESET_LENGTH         0x1

//-----------------------------------------------------------------------------
//         Types
//-----------------------------------------------------------------------------

/// The DP83848 instance
typedef struct _Dp83848 {

    /// The retry & timeout settings
    unsigned int retryMax;

    /// PHY address ( pre-defined by pins on reset )
    unsigned char phyAddress;

} Dp83848, *pDp83848;

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

extern void DP83848_SetupTimeout(Dp83848 *pDp, unsigned int toMax);

extern void DP83848_Init(Dp83848 *pDp, unsigned char phyAddress);

extern unsigned char DP83848_InitPhy(Dp83848 *pDp,
                                    unsigned int mck,
                                    const Pin *pResetPins,
                                    unsigned int nbResetPins,
                                    const Pin *pEmacPins,
                                    unsigned int nbEmacPins);

#if 1
extern unsigned char DP83848_Setup( Dp83848 *pDp );
#else
extern unsigned char DP83848_AutoNegotiate(Dp83848 *pDp);

extern unsigned char DP83848_GetLinkSpeed(Dp83848 *pDp,
                                         unsigned char applySettings);
#endif

extern unsigned char DP83848_Send(Dp83848 *pDp,
                                 void *pBuffer,
                                 unsigned int size);

extern unsigned int DP83848_Poll(Dp83848 *pDp,
                                unsigned char *pBuffer,
                                unsigned int size);


#endif // #ifndef _DP83848_H

