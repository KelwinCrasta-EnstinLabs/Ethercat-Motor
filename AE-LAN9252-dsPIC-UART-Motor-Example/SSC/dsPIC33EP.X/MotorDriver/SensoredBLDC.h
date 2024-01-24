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
//	File:		SensoredBLDC.h
//
//	Written By:		Martin Hill, Microchip Technology
//  Amendments By:  Martin Hill, M.P.H Technical Consultancy Ltd, www.mphelectronics.co.uk
//
// The following files should be included in the MPLAB project:
//
//		SensoredBLDC.c		-- Main source code file
//		Interrupts.c
//		Init.c
//		SensoredBLDC.h		-- Header file
//		
//				
//
//---------------------------------------------------------------------- 

#define FOSC  138187500 
#define FCY  FOSC/2		
#define FCY_PWM FOSC
#define MILLISEC FCY/3685		// 1 mSec delay constant

#define FPWM 20000

#define S2	PORTAbits.RA7       //CHANGED FOR PANTHER

#define HSA PORTCbits.RC6       //Hall Signal Hardware Mapping
#define HSB PORTCbits.RC8
#define HSC PORTCbits.RC9

//Duty Cycles
#define BOOTSTRAP_DC 100
#define TEST_DC 1000
#define LOWER_LIMIT 1000
#define SLEW_RATE 25


struct MotorFlags
{
unsigned RunMotor 	:1;
unsigned Direction	:1;
unsigned unused		:14;
unsigned int MotorState ;
};
enum MC_States{MC_STOPPED, MC_STARTING, MC_RUNNING, MC_STOPPING, MC_FAULT};

extern struct MotorFlags Flags;

extern unsigned int HallValue;

extern unsigned int LoopCount;

extern unsigned int PWM1_State_Fwd[];
extern unsigned int PWM2_State_Fwd[];
extern unsigned int PWM3_State_Fwd[];
extern unsigned int PWM1_State_Rev[];
extern unsigned int PWM2_State_Rev[];
extern unsigned int PWM3_State_Rev[];

extern void InitADC10(void);
extern void DelayNmSec(unsigned int N);
extern void InitMCPWM(void);
extern void InitIC(void);
extern void lockIO(void);
extern void unlockIO(void);
extern void InitMotCon(void);
extern void InitClock(void);
extern void InitTMR3(void);
extern void ChargeCaps(void);
extern unsigned char Hall_Convert();
extern void DisableMotCon(void);
extern void EnableMotCon(void);
extern void ResetMotCon(void);