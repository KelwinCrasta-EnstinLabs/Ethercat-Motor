//---------------------------------------------------------------------
//
//							 Software License Agreement
//
// The software supplied herewith by Microchip Technology Incorporated 
// (the Company) for its PICmicro® Microcontroller is intended and 
// supplied to you, the Companys customer, for use solely and 
// exclusively on Microchip PICmicro Microcontroller products. The 
// software is owned by the Company and/or its supplier, and is 
// protected under applicable copyright laws. All rights are reserved. 
//  Any use in violation of the foregoing restrictions may subject the 
// user to criminal sanctions under applicable laws, as well as to 
// civil liability for the breach of the terms and conditions of this 
// license.
//
// THIS SOFTWARE IS PROVIDED IN AN AS IS CONDITION. NO WARRANTIES, 
// WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
// TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
// PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
// IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
//---------------------------------------------------------------------
//	File:		SensoredBLDC.c
//
//	Written By:		Martin Hill, Microchip Technology
//  Amendments By:  Martin Hill, M.P.H Technical Consultancy Ltd, www.mphelectronics.co.uk
//						
// 
// The following files should be included in the MPLAB project:
//
//		SensoredBLDC.c		-- Main source code file
//		Interrupts.c
//		Init.c
//		SensoredBLDC.h		-- Header file
//		p33FJ256MC710.gld	-- Linker script file
//				
//
//--------------------------------------------------------------------
//
// Revision History
//
// 8/11/17 first version 
//---------------------------------------------------------------------- 

#include "p33Exxxx.h"
#include "SensoredBLDC.h"
#include "DSPIC_Sample.h"
/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/

struct MotorFlags Flags;

unsigned int HallValue;
unsigned int LoopCount; //This is intended to count the main background loops and for NON Blocking usage

/*************************************************************
    Low side driver table is as below.  In the StateLoTableClk
    and the StateLoTableAntiClk tables, the Low side driver is
    PWM while the high side driver is either on or off.  
 *************************************************************/

unsigned int PWM1_State_Fwd[] = {
    0x0000,
    0xC600, //   1L is PWM, 1H OFF
    0xC700, //   1L = 1H = OFF  
    0xC600, //   1L is PWM, 1H OFF
    0xC780, //   1L is OFF, 1H is ON
    0xC700, //   1L = 1H = OFF  
    0xC780, //   1L is OFF, 1H is ON
    0x0000
};
unsigned int PWM2_State_Fwd[] = {
    0x0000,
    0xC700, // 2L = 2H = OFF  
    0xC780, // 2L is OFF, 2H is ON
    0xC780, // 2L is OFF, 2H is ON
    0xC600, // 2L is PWM, 2H OFF
    0xC600, // 2L is PWM, 2H OFF
    0xC700, // 2L = 2H = OFF  
    0x0000
};
unsigned int PWM3_State_Fwd[] = {
    0x0000,
    0xC780, // 3H is ON, 3L OFF	
    0xC600, // 3L is PWM, 3H OFF
    0xC700, // 3L = 3H = OFF  
    0xC700, // 3L = 3H = OFF  
    0xC780, // 3L is OFF, 3H is ON
    0xC600, // 3L is PWM, 3H OFF
    0x0000
};
unsigned int PWM1_State_Rev[] = {
    0x0000,
    0xC780, //   1L is OFF, 1H is ON   
    0xC700, //   1L = 1H = OFF  
    0xC780, //   1L is OFF, 1H is ON
    0xC600, //   1L is PWM, 1H OFF
    0xC700, //   1L = 1H = OFF  
    0xC600, //   1L is PWM, 1H OFF
    0x0000
};
unsigned int PWM2_State_Rev[] = {
    0x0000,
    0xC700, // 2L = 2H = OFF 
    0xC600, // 2L is PWM, 2H OFF
    0xC600, // 2L is PWM, 2H OFF
    0xC780, // 2L is OFF, 2H is ON    
    0xC780, // 2L is OFF, 2H is ON
    0xC700, // 2L = 2H = OFF  
    0x0000
};
unsigned int PWM3_State_Rev[] = {
    0x0000,
    0xC600, // 3L is PWM, 3H OFF    
    0xC780, // 3L is OFF, 3H is ON   
    0xC700, // 3L = 3H = OFF  
    0xC700, // 3L = 3H = OFF 
    0xC600, // 3L is PWM, 3H OFF    
    0xC780, // 3H is ON, 3L OFF	
    0x0000
};

//---------------------------------------------------------------------
// This is a generic 1ms delay routine to give a 1mS to 65.5 Seconds delay
// For N = 1 the delay is 1 mS, for N = 65535 the delay is 65,535 mS. 
// Note that FCY is used in the computation.  Please make the necessary
// Changes(PLLx4 or PLLx8 etc) to compute the right FCY as in the define
// statement above.

void DelayNmSec(unsigned int N)
{
unsigned int j;
while(N--)
 	for(j=0;j < MILLISEC;j++);
}

/*********************************************************************************** 
 * Function: lockIO
 *
 * Preconditions: None.
 *
 * Overview: This executes the necessary process to set the IOLOCK bit to lock
 * I/O mapping from being modified.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
void lockIO() {
    asm volatile ("mov #OSCCON,w1 \n\t"
            "mov #0x46, w2 \n\t"
            "mov #0x57, w3 \n\t"
            "mov.b w2,[w1] \n\t"
            "mov.b w3,[w1] \n\t"
            "bset OSCCON, #6");
}

/*****************************************************************************
 * Function: unlockIO
 *
 * Preconditions: None.
 *
 * Overview: This executes the necessary process to clear the IOLOCK bit to 
 * allow I/O mapping to be modified.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
void unlockIO() {
    asm volatile ("mov #OSCCON,w1 \n\t"
               "mov #0x46, w2 \n\t"
                "mov #0x57, w3 \n\t"
                "mov.b w2,[w1] \n\t"
                "mov.b w3,[w1] \n\t"
                "bclr OSCCON, #6");
}

/*****************************************************************************
 * Function: hall_convert
 *
 * Preconditions: None.
 *
 * Overview: This executes the necessary process to convert the sensed 
 * Hall signals x3 to a code reflecting the unit distance 
 *
 * Input: None.
 *
 * Output: Resulting code.
 *
 *****************************************************************************/
unsigned char Hall_Convert(void) {
    unsigned char result = 0;
    result = HSA; //Read Halls connected to PORTB
    result += HSB * 2;
    result += HSC * 4;
    return (result);
}
/*****************************************************************************
 * Function: ChargeCaps
 *
 * Preconditions: None.
 *
 * Overview: This sets a duty cycle and enables the low side bridge
 * outputs 
 *
 * Input: None.
 *
 * Output: Resulting code.
 *
 *****************************************************************************/
void ChargeCaps(void) {
    MDC = BOOTSTRAP_DC; //Set DC to charge Bootstraps
    PTCON |= 0x8000; //Enable PWM Module
    IOCON1 = 0xC600; // Turn ON PWM low.
    IOCON2 = 0xC600; // Turn ON PWM low.
    IOCON3 = 0xC600; // Turn ON PWM low. 
}
/*****************************************************************************
 * Function: DisableMotCon
 *
 * Preconditions: None.
 *
 * Overview: Disable IC interrupts and MCPWM Module, sets DC to 0
 * Resets Flags state  
 *
 * Input: None.
 *
 * Output: Resulting code.
 *
 *****************************************************************************/
void DisableMotCon(void) {
    IEC0bits.IC1IE = 0;
    IEC0bits.IC2IE = 0;
    IEC2bits.IC3IE = 0;
    LATAbits.LATA10 = 0; //Turn OFF the D12 LED
    LATAbits.LATA3 = 0; //Disable the MCP8026
    MDC = 0; //Set DC to 0
    PTCONbits.PTEN = 0;
    Flags.RunMotor = 0; // reset run flag 
}
/*****************************************************************************
 * Function: EnableMotCon
 *
 * Preconditions: None.
 *
 * Overview: Enable IC interrupts and MCPWM Module, Preset DC, Set MCPWM
 * outputs to initial state, Clear Stall Timer, Set Flags state
 *
 * Input: None.
 *
 * Output: Resulting code.
 *
 *****************************************************************************/
void EnableMotCon(void) {
    MDC = LOWER_LIMIT;
    TMR3 = 0; //Clear Stall Detect Timer    
    T3CONbits.TON = 1; //Start Stall Detect Timer            
    if (Flags.Direction) {
        IOCON1 = PWM1_State_Fwd[HallValue];
        IOCON2 = PWM2_State_Fwd[HallValue];
        IOCON3 = PWM3_State_Fwd[HallValue];
    } else {
        IOCON1 = PWM1_State_Rev[HallValue];
        IOCON2 = PWM2_State_Rev[HallValue];
        IOCON3 = PWM3_State_Rev[HallValue];
    }
    IFS0bits.T3IF = 0; //Reset T3 Stall detect timer overflow flag            
    IEC0bits.IC1IE = 1;
    IEC0bits.IC2IE = 1;
    IEC2bits.IC3IE = 1;
    //            PTCON |= 0x8000; //Enable PWM Module
    PTCONbits.PTEN = 1;
    Flags.RunMotor = 1; // set flag
}
/*****************************************************************************
 * Function: ResetMotCon
 *
 * Preconditions: None.
 *
 * Overview: Disable IC interrupts and MCPWM Module, sets DC to 0
 * Disable stall Timer, Set MCPWM outputs to inactive  
 *
 * Input: None.
 *
 * Output: Resulting code.
 *
 *****************************************************************************/
void ResetMotCon(void) {
    IFS0bits.T3IF = 0; //Reset T3 Stall detect timer overflow flag
    T3CONbits.TON = 0;
    LATAbits.LATA10 = 0; //Turn OFF the D12 LED
    LATAbits.LATA3 = 0; //Disable the MCP8026
    MDC = 0;
    IEC0bits.IC1IE = 0;
    IEC0bits.IC2IE = 0;
    IEC2bits.IC3IE = 0;
    IOCON1 = PWM1_State_Fwd[0];
    IOCON2 = PWM2_State_Fwd[0];
    IOCON3 = PWM3_State_Fwd[0];
    LATAbits.LATA3 = 1; //Enable the MCP8026
    PTCONbits.PTEN = 0; //Disable PWM Module
}