
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
//	File:		Init.c
//
//						
// 
// The following files should be included in the MPLAB project:
//
//		SensoredBLDC.c		-- Main source code file
//		Interrupts.c
//		Init.c
//		SensoredBLDC.h		-- Header file
//				
//
//--------------------------------------------------------------------

#include "p33Exxxx.h"
#include "SensoredBLDC.h"
//#include "userdef.h"

/********************************************************************
InitMCPWM, intializes the PWM as follows:
1. FPWM = 20000 Hz
2. Independent PWMs
3. Control outputs using OVDCON
4. Set Duty Cycle from EtherCat

*********************************************************************/

void InitMCPWM(void)
{
    PTPER = FCY_PWM/FPWM - 1; 

    PWMCON1 = 0x0104;	// Enable PWM output pins 
	PWMCON2 = 0x0104;	// 
	PWMCON3 = 0x0104;

    PTCON = 0;  //ADDED
    
	IOCON1 = 0xC300;    //Configured for Redundant Mode with override enabled for PWHx
	IOCON2 = 0xC300;
	IOCON3 = 0xC300;

	DTR1 = 0x0000;	// 2 us of dead time
	DTR2 = 0x0000;	// 2 us of dead time
	DTR3 = 0x0000;	// 2 us of dead time

	ALTDTR1 = 0x000A;	// 2 us of dead time
	ALTDTR2 = 0x000A;	// 2 us of dead time
	ALTDTR3 = 0x000A;	// 2 us of dead time

	PTCON2 = 0x0000;	// Divide by 1 to generate PWM

    FCLCON1 = 0x0001;   
	FCLCON2 = 0x0001;
	FCLCON3 = 0x0001;

    TRISBbits.TRISB15 = 0;	// Pin-15 of dsPIC33EP64MC504 = Pin-93 of PIM = PWML1
	TRISBbits.TRISB14 = 0;	// Pin-14 of dsPIC33EP64MC504 = Pin-94 of PIM = PWMH1
	TRISBbits.TRISB13 = 0;	// Pin-11 of dsPIC33EP64MC504 = Pin-98 of PIM = PWML2
	TRISBbits.TRISB12 = 0;	// Pin-10 of dsPIC33EP64MC504 = Pin-99 of PIM = PWMH2
	TRISBbits.TRISB11 = 0;	// Pin-9 of dsPIC33EP64MC504 = Pin-100 of PIM = PWML3
	TRISBbits.TRISB10 = 0;	// Pin-8 of dsPIC33EP64MC504 = Pin-3 of PIM = PWMH3	
    
    IOCON1 = PWM1_State_Fwd[HallValue];
    IOCON2 = PWM2_State_Fwd[HallValue];
    IOCON3 = PWM3_State_Fwd[HallValue];
    
	return;				 
}
/*********************************************************************
  Function:        void InitICandCN(void)

  Overview:        Configure Hall sensor inputs, one change notification and 
                   two input captures. on IC7 the actual capture value is used
                   for further period calculation

  Note:            None.
********************************************************************/

void InitIC(void)
{
    ANSELB &= 0xfdf5;   //  RB9 digital (for Hall)), RB2 = PHC (analogue I/P))
    ANSELC &= 0xfc37;   //Make RC6/7/8/9 digital (for Hall))
    ANSELA &= 0xfbf7;   //LED D12 o/p RA10, SW2 Start/stop button RA7
	// Init Input Capture 1
	IC1CON1 = 0x0001;	// Input capture every edge with interrupts and TMR3
	IC1CON2 = 0x0000;
    IPC0bits.IC1IP = 4; //Set IP Priority Level 4
	IFS0bits.IC1IF = 0;	// Clear interrupt flag
	// Init Input Capture 2
	IC2CON1 = 0x0001;	// Input capture every edge with interrupts and TMR3
    IC2CON2 = 0x0000;
    IPC1bits.IC2IP = 4; //Set IP Priority Level 4
	IFS0bits.IC2IF = 0;	// Clear interrupt flag
	// Init Input Capture 3
	IC3CON1 = 0x0001;	// Input capture every edge with interrupts and TMR3
	IC3CON2 = 0x0000;
    IPC9bits.IC3IP = 4; //Set IP PriorityLevel 4
	IFS2bits.IC3IF = 0;	// Clear interrupt flag
    
    //Enable IC Interrupts
//    IEC0bits.IC1IE = 1;
//    IEC0bits.IC2IE = 1;
//    IEC2bits.IC3IE = 1;
    
	return;
}


/************************************************************************
Initialize for Motor Control

 *************************************************************************/
void InitMotCon() {
    //    //Motor Control Related
    TRISA = 0x0283; // SPI MISO on RA9, S2 as input RA7,RA0 = PHA (Analogue I/P), RA1 = PHB (Analogue I/P)
    TRISB = 0x028E; // LAN9252 IRQ on RB7, SPI SS on RB0, ADCBUS on RB2,RB3 = PHC (Analogue I/P), RB9 HALLA for REV A
    //TRISC = 0x03C6; // RC1 Analogue Input Hall input on RC6/7/8/RC9   
    TRISC = 0x03E6; // RC1 Analogue Input Hall input on RC6/7/8/RC9   
    LATA = 0;

    unlockIO();
    RPINR7bits.IC1R = 0x36; // IC1 on RP1/RC6
    RPINR7bits.IC2R = 0x38; // IC2 on RP2/RC8
    RPINR8bits.IC3R = 0x39; // IC3 on RP3/RC9
    lockIO();
    InitMCPWM();
    InitIC();
    InitTMR3();
    Flags.Direction = 0; // initialize direction CW
    Flags.MotorState = MC_STOPPED;
}
void InitClock(){
//    //Initialize clock
//    // Configure Oscillator to operate the device at 140Mhz
//    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
//    // FRC nominal freq - 7.37 Mhz
//    // Fosc= 7.37 * 75/(2*2)= 50Mhz for 8M input clock... M = 75 138.18 Mhz 

    PLLFBD = 75; // M=75. PLLFBD = M-2
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
 OSCTUN=0;					// Tune FRC oscillator, if FRC is used
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;
    __builtin_write_OSCCONH(0x01); //FRC with PLL NOSC=0b001
    __builtin_write_OSCCONL(0x01); //
      while(OSCCONbits.COSC != 0b001)
    {
        ; // Wait for Clock switch to occur
    }
    while (!OSCCONbits.LOCK) // wait for PLL ready. PLL LOCK bit will be set to 1 when the clock is ready
    {
        ;
    }
}

/************************************************************************
Tmr3 is used to determine if the motor has stalled set to count using Tcy*256

 *************************************************************************/

void InitTMR3(void) {
    T3CON = 0x0030; // internal Tcy/256 clock
    TMR3 = 0;
    PR3 = 0xFFFF;
    return;
}


