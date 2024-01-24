#ifndef USERDEF_H
#define USERDEF_H

/**********************************************************************
* � 2010 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
******************************************************************************/

/* GLOBAL DEFINES FOR FUNCTIONING MODE */
/* undefine for negation */
#define CLOSEDLOOP      // if defined the speed controller will be enabled
#undef PHASE_ADVANCE    // for extended speed ranges this should be defined


#undef RTDM_DEMO // RTDM and DMCI demo enable


#ifdef RTDM_DEMO
    #define DATA_BUFFER_SIZE 100  //Size in 16-bit Words
    #define SNAPDELAY	2000 // In number of PWM Interrupts
    #define	SNAP1		Sector
    #define	SNAP2		Phase
    #define SNAP3	   	PDC1
    #define SNAP4	   	MeasuredSpeed
#endif

/* PROCESSOR SPECIFICS */
// Debug #define FCY  70000000	 // xtal = 8Mhz; with PLL -> 70 MIPS
// Debug #define FPWM 20000		 // 20 kHz, so that no audible noise is present.

#define PDIV 256         // divider for Timer 1,3

#define CW	0		// Counter Clock Wise direction
#define CCW	1		// Clock Wise direction
#define SWITCH_S2	(!PORTGbits.RG7) // Push button S2  


/* ALGORITHM SPECIFICS */
#define NO_POLEPAIRS 5

// In the sinewave generation algorithm we need an offset to be added to the
// pointer when energizing the motor
#define PHASE_OFFSET_CW 56600
#define PHASE_OFFSET_CCW 0

#define MAX_PH_ADV_DEG 40  // This value represents the maximum allowed phase
                           // advance in electrical degrees. Set a value from
                           // 0 to 60. This value will be used to calculate
                           // phase advance only if PHASE_ADVANCE is defined

// Period Calculation
// Period = (FCY / PDIV * 60) / (RPM * NO_POLEPAIRS )
#define MINPERIOD	547		// For 6000 max rpm and 10 poles motor
#define MAXPERIOD	32767	// For 100 min rpm and 10 poles motor

#define _10MILLISEC	 10  // Used as a timeout with no hall effect sensors
                         // transitions and Forcing steps according to the
                         // actual position of the motor
#define _1SEC 1000       // after this time has elapsed, the motor is
                         // consider stalled and it's stopped


#define SPEED_REF_DIV 16 // speed pot divider

#define PERIOD_FILTER_CONST 100 /* the smaller the value, the higher the filter
                                    and the delay introduced */

/* for calculation for PHASE_INC */
/* maximum range of angle, meaning 65536 is divided by fPWM/felectrical */
/* resulting the folowing constant, which is divided by ticks of timer */

#define PHASE_INC_CALC  448000UL //(FCY/(PDIV*2*FPWM)*65536)

/* for speed rpm calculation */
#define SPEED_RPM_CALC 1640625UL //(FCY/(PDIV*2*NO_POLEPAIRS)*60)

// Desired deadtime in us
#define DEADTIME 2
// Calculated deadtime value to be written into ALTDTRx registers
#define ALTDTR_VAL (2*FCY*DEADTIME)/1000000
// Half of the PWM Deadtime
#define	ALTDTR_DIV2	ALTDTR_VAL/2

// Use this MACRO when using floats to initialize signed 16-bit fractional 
// variables
#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (int)(32768 * (Float_Value) - 0.5) \
        : (int)(32767 * (Float_Value) + 0.5))
        
        
// PI parameters        
#define P_SPEED_PI Q15(0.5)   	//prop
#define I_SPEED_PI Q15(0.015)	//integ
#define C_SPEED_PI 0x7FFF                   //windup
#define MAX_SPEED_PI 0x7FFF                 //maxout

//----------------------------------------------------------------------
// 	Hurst Motor Terminals |
// -----------------------|---------------------------------
//	Ground Phase ---------|-- G
//	Phase Red    ---------|-- M1
//	Phase Black  ---------|-- M2
//	Phase White  ---------|-- M3
//	Hall White   ---------|-- HA
//	Hall Brown   ---------|-- HB
//	Hall Green   ---------|-- HC
//  Hall Red     ---------|-- 5V
//  Hall Black   ---------|-- GND

#endif
