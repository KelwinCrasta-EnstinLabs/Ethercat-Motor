//---------------------------------------------------------------------
//
//							 Software License Agreement
//
// The software supplied herewith by Microchip Technology Incorporated 
// (the �Company�) for its PICmicro� Microcontroller is intended and 
// supplied to you, the Company�s customer, for use solely and 
// exclusively on Microchip PICmicro Microcontroller products. The 
// software is owned by the Company and/or its supplier, and is 
// protected under applicable copyright laws. All rights are reserved. 
//  Any use in violation of the foregoing restrictions may subject the 
// user to criminal sanctions under applicable laws, as well as to 
// civil liability for the breach of the terms and conditions of this 
// license.
//
// THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES, 
// WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
// TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
// PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
// IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
//---------------------------------------------------------------------
//	File:		Interrupts.c
//
//						
// 
// The following files should be included in the MPLAB project:
//
//		SensoredBLDC.c		-- Main source code file
//		Interrupts.c
//		Init.c
//		SensoredBLDC.h		-- Header file
//		p33FJ32MC204.gld	-- Linker script file
//				
//
//---------------------------------------------------------------------- 

#include "p33Exxxx.h"
#include "SensoredBLDC.h"


/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)

PreCondition:	The inputs of the hall effect sensors should have low pass
                filters. A simple RC network works.
 
Overview:		This interrupt represents Hall A ISR.
                In Reverse, Hall reading == 3 or 4
                In Forward, Hall reading == 2 or 5
                and generates the next commutation sector.
                Hall A is used for Speed measurement
 ********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void) {

    IFS0bits.IC1IF = 0; // Clear interrupt flag

    HallValue = Hall_Convert();

    if (Flags.Direction) {
        IOCON1 = PWM1_State_Fwd[HallValue];
        IOCON2 = PWM2_State_Fwd[HallValue];
        IOCON3 = PWM3_State_Fwd[HallValue];

    } else {
        IOCON1 = PWM1_State_Rev[HallValue];
        IOCON2 = PWM2_State_Rev[HallValue];
        IOCON3 = PWM3_State_Rev[HallValue];
    }

}


/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt (void)

PreCondition:	The inputs of the hall effect sensors should have 
                low pass filters. A simple RC network works.
 
Overview:		This interrupt represents Hall B ISR.
                Hall reading == 1 or 6
                and generates the next commutation sector.
   				
 ********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void) {
    IFS0bits.IC2IF = 0; // Clear interrupt flag
    //	HallValue = (unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
    HallValue = Hall_Convert();

    if (Flags.Direction) {
        IOCON1 = PWM1_State_Fwd[HallValue];
        IOCON2 = PWM2_State_Fwd[HallValue];
        IOCON3 = PWM3_State_Fwd[HallValue];
    } else {
        IOCON1 = PWM1_State_Rev[HallValue];
        IOCON2 = PWM2_State_Rev[HallValue];
        IOCON3 = PWM3_State_Rev[HallValue];
    }
}

/*********************************************************************
Function:		void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt (void)

PreCondition:	The inputs of the hall effect sensors should have 
                low pass filters. A simple RC network works.
 
Overview:		This interrupt represents Hall C ISR.
                and generates the next commutation sector.
   					
 ********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _IC3Interrupt(void) {
    IFS2bits.IC3IF = 0; // Clear interrupt flag
    HallValue = Hall_Convert();
    if (Flags.Direction) {
        IOCON1 = PWM1_State_Fwd[HallValue];
        IOCON2 = PWM2_State_Fwd[HallValue];
        IOCON3 = PWM3_State_Fwd[HallValue];
    } else {
        IOCON1 = PWM1_State_Rev[HallValue];
        IOCON2 = PWM2_State_Rev[HallValue];
        IOCON3 = PWM3_State_Rev[HallValue];
    }
    TMR3 = 0; //Reset Stall Detect Timer
}


