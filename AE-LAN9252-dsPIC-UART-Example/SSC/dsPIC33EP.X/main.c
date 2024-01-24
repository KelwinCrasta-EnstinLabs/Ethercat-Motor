/* 
 * File:   main.c
 * Author: I15953
 *
 * Created on April 17, 2017, 2:33 PM
 */

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/* Standard Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>



/* dsPIC related includes */
#include <xc.h>
#include <p33Exxxx.h>
#include "ecat_def.h"
#include "9252_HW.h"
#include "applInterface.h"
#include "ecatappl.h"
#include "SPIDriver/SPIDriver.h"
#include "user.h"
#include "DSPIC_Sample.h"

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

/*
 * Board: dsPIC MCLV-2 Development board
 */
int main(void)
{
//    //Initialize clock
//    // Configure Oscillator to operate the device at 140Mhz
//    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
//    // FRC nominal freq - 7.37 Mhz
//    // Fosc= 7.37 * 75/(2*2)= 50Mhz for 8M input clock... M = 75 or 76? 138.18 Mhz or 140.03 MHz
//    // TBD
    PLLFBD =  74;                 // M=75. PLLFBD = M-2
    CLKDIVbits.PLLPOST = 0;       // N1=2
    CLKDIVbits.PLLPRE = 0;       // N2=2
    OSCTUN=0;					// Tune FRC oscillator, if FRC is used
    
    
    // Disable Watch Dog Timer
	RCONbits.SWDTEN=0;
    // Configure Oscillator to operate the device at 20Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8*10/(2*2)= 20Mhz for 8M input clock

   // PLLFBD = 73; // M=75
   // CLKDIVbits.PLLPOST = 0; // N1=2
   // CLKDIVbits.PLLPRE = 0; // N2=2
   // __builtin_write_OSCCONH(0x03);
  //  __builtin_write_OSCCONL(0x01);   
    /* Initiate Clock Switch to Primary Oscillator with PLL (NOSC = 0x3) */
    __builtin_write_OSCCONH(0x01); //FRC with PLL NOSC=0b001
    __builtin_write_OSCCONL(0x01); //
    
    while(OSCCONbits.COSC != 0b001)
    {
        ; // Wait for Clock switch to occur
    }
    
    while(!OSCCONbits.LOCK)    // wait for PLL ready. PLL LOCK bit will be set to 1 when the clock is ready
    {
		;
	}
    
  
    PDI_Init(); 
    InitApp ();
 
    /* Initialize the Hardware and the EtherCAT Slave Controller */
    LAN9252_Init();
    MainInit();

    /*Create basic mapping*/
    APPL_GenerateMapping(&nPdInputSize,&nPdOutputSize);
 
    bRunApplication = TRUE;

    do
    {
        MainLoop();

    } while (bRunApplication == TRUE);

    HW_Release();

    return 0;  
}
