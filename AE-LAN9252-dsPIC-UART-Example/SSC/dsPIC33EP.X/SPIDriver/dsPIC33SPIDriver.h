/*******************************************************************************
 PIC32 SPI Interface Driver

  Company:
    Microchip Technology Inc.

  File Name:
    SPIDriver.h

  Summary:
    Contains the Header File of PIC32 SPI Interface Driver

  Description:
    This file contains the Header File of PIC32 SPI Interface Driver
	
  Change History:
    Version		Changes
	1.3			Initial version.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

 Microchip licenses to you the right to use, modify, copy and distribute
 Software only when embedded on a Microchip microcontroller or digital signal
 controller that is integrated into your product or third party product
 (pursuant to the sublicense terms in the accompanying license agreement).

 You should refer to the license agreement accompanying this Software for
 additional information regarding your rights and obligations.

 SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
 CONTRACT, NEGLiPMPCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
 OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
 CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
 SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "ecat_def.h"
#include <xc.h>
#include "SPIDriver.h"


#ifndef DSPIC33SPIDRIVER_H
#define	DSPIC33SPIDRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif
	// *****************************************************************************
	// *****************************************************************************
	// Section: File Scope or Global Data Types
	// *****************************************************************************
	// *****************************************************************************
	

	#define SPIGet()     SPI1BUF
	#define CSLOW()      LATBbits.LATB0 = 0; 
	#define CSHIGH()     LATBbits.LATB0 = 1;


	

	// *****************************************************************************
	// *****************************************************************************
	// Section: File Scope Functions
	// *****************************************************************************
	// *****************************************************************************

	void SPIOpen();
	UINT8 SPIReadByte();
    UINT8 SPIReadLastByte();
	void SPIWriteByte(UINT8 data);
	void SPIPut(UINT8 data);
    void Init_DELAY_1US();
    void DELAY_1US();
    
#ifdef	__cplusplus
}
#endif

#endif	/* PMPDRIVER_H */

