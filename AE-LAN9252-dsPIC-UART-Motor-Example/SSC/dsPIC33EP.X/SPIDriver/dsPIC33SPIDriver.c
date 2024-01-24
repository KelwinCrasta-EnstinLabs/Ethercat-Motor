/*******************************************************************************
 PIC32 SPI Interface Driver

  Company:
    Microchip Technology Inc.

  File Name:
    SPIDriver.c

  Summary:
    Contains the functional implementation of PIC32 SPI Interface Driver

  Description:
    This file contains the functional implementation of PIC32 SPI Interface Driver
	
  Change History:
    Version		Changes
	1.3			Initial version.
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

#include "dsPIC33SPIDriver.h"
#include "9252_HW.h"


inline void SPIPut(UINT8 data)
{
    // Wait for free buffer
    //while(!SPI1STATbits.SPITBF);
    SPI1BUF = data;

    // Wait for data UINT8
    while(!SPI1STATbits.SPIRBF)
    {
        ;
    }
}
/*******************************************************************************
  Function:
   void SPIWrite(UINT8 data)
  Summary:
    This function write one byte.
        
  Description:
    This function write one byte.

*****************************************************************************/ 
inline void SPIWriteByte(UINT8 data)
{
    SPIPut(data);
    SPIGet();
}
/*******************************************************************************
  Function:
   void SPIRead(void)
  Summary:
    This function read one byte.
        
  Description:
    This function read one byte.

*****************************************************************************/  
inline UINT8 SPIReadByte()
{
    SPIPut(0);
    return ((UINT8)SPIGet());
 }

/*******************************************************************************
  Function:
   void SPIOpen(void)
  Summary:
    This function configures the SPI module of SOC.
        
  Description:
    This function configures the SPI module of SOC.

*****************************************************************************/  
void SPIOpen()
{
    //Initialize to zero
    SPI1STAT = 0;
    SPI1CON1 = 0;
//  SPI1CON2 = 0;
    SPI1BUF = 0;
    
    IFS0bits.SPI1IF     = 0;    //Clear the Interrupt Flag
    IEC0bits.SPI1IE     = 0;    //disable the Interrupt
     
    //Enable master as SPI1
    SPI1CON1bits.MSTEN = 1;     //Master Mode enable
    SPI1CON1bits.CKP = 0;       //Idle state for clock is a low level; active state is a high level
    SPI1CON1bits.CKE = 1;       //Serial output data changes on transition from active clock state to Idle clock state
    SPI1CON1bits.SMP = 1;       //Input data sampled at end of data output time
    
    
    //Set clock speed 8Mhz. 
    SPI1CON1bits.PPRE = 0b10;    //Primary pre-scale 4:1
    SPI1CON1bits.SPRE = 0b110;	//Secondary pre-scale 2:1
    
    //Configure other lines
    //Configure CS line as GPIO output
  
    TRISBbits.TRISB0 =0;             //Device pin 15 - SS1, RB0
    LATBbits.LATB0 = 1;             // Set CS high/De-assert CS
    TRISCbits.TRISC3 = 0;          //Device pin 35 - SCK1/RPI51/RC3
    //ANSELAbits.ANSA4 = 0;
    TRISAbits.TRISA4 = 0;         //Device pin 33 - CVREF2O/SDO1/RP20/T1CK/RA4
    TRISAbits.TRISA9 = 1;
 
    //Enable SPI
    SPI1STATbits.SPIEN = 1;	 
}

void Init_DELAY_1US()
{
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR2 = 0x00; // Clear timer register
    PR2 = 70; // Load the period value

    IFS0bits.T2IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T2IE = 0; // Enable Timer1 interrupt
    T2CONbits.TON = 0; // Stop Timer
}

void DELAY_1US()
{
	PR2 = 70; // Load the period value
    T2CONbits.TON = 1; // Start Timer
    while(!IFS0bits.T2IF)
    {
        ;
    }
    T2CONbits.TON = 0; // Stop Timer
    IFS0bits.T2IF = 0;
	TMR2 = 0x00;    
}


