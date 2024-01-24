/*******************************************************************************
 LAN9252 Hardware Abtraction Layer - Implementation file

  Company:
    Microchip Technology Inc.

  File Name:
    PICHW.c

  Description:
    This file  cContains the functional implementation of PIC32 Hardware Abtraction Layer
	
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

///////////////////////////////////////////////////////////////////////////////
// Included files


#include <xc.h>
#include "9252_HW.h"
#include "ecatappl.h"



/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */
///////////////////////////////////////////////////////////////////////////////
// Global Interrupt setting
#if 1
    // Global Interrupt setting
    volatile int int_reg = 0;
    ///////////////////////////////////////////////////////////////////////////////
    // ESC Interrupt
    //0 - positive edge 1- negative edge

    #define    INIT_ESC_INT             {(INTCON2bits.INT0EP) = 1; TRISBbits.TRISB7 = 1; IPC0bits.INT0IP = 3;IFS0bits.INT0IF=0; IEC0bits.INT0IE=1;}; //RB7 - INT0
    #define    ESC_INT_REQ              (IFS0bits.INT0IF) //ESC Interrupt (INT0) state/flag
    #define    INT_EL                   (_RB7) //ESC Interrupt input port
    #define    EscIsr                   (_INT0Interrupt) // primary interrupt vector name
    #define    ACK_ESC_INT              {(ESC_INT_REQ)=0;}

    #define     IS_ESC_INT_ACTIVE           ((INT_EL) == 0) //0 - fro active low; 1 for hactive high
    #define     IRQ_INTERRUPT_VECTOR 		_EXTERNAL_0_VECTOR


///////////////////////////////////////////////////////////////////////////////
// SYNC0 Interrupt
    #define    INIT_SYNC0_INT                  {(INTCON2bits.INT1EP) = 0;(IPC5bits.INT1IP) = 2;TRISCbits.TRISC7 = 1;IFS1bits.INT1IF = 0; RPINR0 = 0; RPINR0bits.INT1R = 0x37;}  
    #define    SYNC0_INT_REQ                   (IFS1bits.INT1IF) //Sync0 Interrupt (INT1) state
    #define    INT_SYNC0                       (_RC7) //Sync0 Interrupt input port
    #define    Sync0Isr                        (_INT1Interrupt) // primary interrupt vector name
    #define    DISABLE_SYNC0_INT               {IEC1bits.INT1IE=0;}//{(_INT1IE)=0;}//disable interrupt source INT1
    #define    ENABLE_SYNC0_INT                {IEC1bits.INT1IE=1;} //enable interrupt source INT1
    #define    ACK_SYNC0_INT                   {(SYNC0_INT_REQ) = 0;}
    #define    SET_SYNC0_INT                   {(SYNC0_INT_REQ) = 1;}
    #define    SYNC0_INT_PORT_IS_ACTIVE        {(INT_EL) == 0;}
    #define    IS_SYNC0_INT_ACTIVE              ((INT_SYNC0) == 0) //0 - fro active low; 1 for hactive high

    #define    INIT_SYNC1_INT                   {(INTCON2bits.INT2EP) = 0;(IPC7bits.INT2IP) = 2; TRISCbits.TRISC6 = 1; IFS1bits.INT2IF =0; RPINR1 =0; RPINR1bits.INT2R = 0x36;}
    #define    SYNC1_INT_REQ                    (IFS1bits.INT2IF) //(_INT4IF) //Sync1 Interrupt (INT2) state
    #define    INT_SYNC1                        (_RC6) //Sync1 Interrupt input port
    #define    Sync1Isr                         (_INT2Interrupt) // primary interrupt vector name
    #define    DISABLE_SYNC1_INT                {IEC1bits.INT2IE=0;}//disable interrupt source INT2
    #define    ENABLE_SYNC1_INT                 {IEC1bits.INT2IE=1;} //enable interrupt source INT2
    #define    ACK_SYNC1_INT                    {(SYNC1_INT_REQ) = 0;}
    #define    SET_SYNC1_INT                    {(SYNC1_INT_REQ) = 1;}
    #define    SYNC1_INT_PORT_IS_ACTIVE         {(INT_EL) == 0;}
    #define    IS_SYNC1_INT_ACTIVE              ((INT_SYNC1) == 0) //0 - fro active low; 1 for hactive high


///////////////////////////////////////////////////////////////////////////////
// Hardware timer

#define STOP_ECAT_TIMER         {(T1CONbits.TON) = 0; /*disable timer*/}
#define INIT_ECAT_TIMER         {(T1CONbits.TCS) = 0; (T1CONbits.TGATE) = 0; \
                                  (T1CONbits.TCKPS) = 0b01; /* Select 1:8 Prescaler*/ \
                                  (TMR1) = 0x00; /* Clear timer register*/ \
                                  PR1 = 8750; /* Load the period value*/ \
                                  (IPC0bits.T1IP) = 0x01; /* Set Timer 1 Interrupt Priority Level */ \
                                  (IFS0bits.T1IF) = 0; /* Clear Timer 1 Interrupt Flag */ \
                                  (IEC0bits.T1IE) = 1; /*Enable Timer1 interrupt*/ }

#define START_ECAT_TIMER         {(T1CONbits.TON) = 1; /*enable timer*/ }


/*-----------------------------------------------------------------------------------------
------
------    LED defines
------
-----------------------------------------------------------------------------------------*/

#define LED_ECATRED                    LATDbits.LATD1   //??????????

/*  A brief description of a section can be given directly below the section
    banner.
 */

#endif
/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

void PDI_Restore_Global_Interrupt(UINT32 Int_sts)
{
    /* protected code here */
    RESTORE_CPU_IPL(0); 
}

/*******************************************************************************
  Function:
     void PDI_Enable_Global_interrupt()

  Summary:
    Enables the platform to handle interrupts.

  Description:
    This routine enables the core to handle any pending interrupt requests.

  Precondition:
    Need to configure system using interrupt (IRQ, SYNC0 and SYNC1- if they are using)
 *****************************************************************************/
void PDI_Enable_Global_interrupt()
{
    INTCON2bits.GIE = 1;
}

/*******************************************************************************
  Function:
    int PDI_Disable_Global_Interrupt()

  Summary:
    Disables the platform from handling interrupts.

  Description:
    This routine disables the core from handling any pending interrupt requests.

  Returns:
    The previous state of the interrupt Status.  The PDI_Restore_Global_Interrupt
    function can be used in other routines to restore the system interrupt state.

  Example:
    <code>
    unsigned int intStatus;

    intStatus = PDI_Disable_Global_Interrupt();
    </code>

*****************************************************************************/
UINT32 PDI_Disable_Global_Interrupt()
{
    SET_AND_SAVE_CPU_IPL(int_reg, 4); /* disable interrupts */
	return int_reg;
}

/*******************************************************************************
  Function:
    UINT16 PDI_GetTimer(void)

  Summary:
    Get the 1ms current timer value
  Description:
    This routine gets the 1ms current timer value.
*****************************************************************************/
UINT16 PDI_GetTimer()
{
   return(TMR1);
}

/*******************************************************************************
  Function:
    void PDI_ClearTimer(void)

  Summary:
    Clear the 1ms current timer value
  Description:
    This routine clears the 1ms current timer value.
*****************************************************************************/
void PDI_ClearTimer()
{
    TMR1 = 0;
}

/*******************************************************************************
  Function:
  void PDI_Timer_Interrupt(void)

  Summary:
   This function configure and enable the TIMER interrupt for 1ms
        
  Description:
  This function configure and enable the TIMER interrupt for 1ms
*****************************************************************************/
void PDI_Timer_Interrupt()
{
    INIT_ECAT_TIMER;
    
    START_ECAT_TIMER;

}
/*******************************************************************************
  Function:
  void PDI_IRQ_Interrupt(void)

  Summary:
   This function configure and enable the interrupt for IRQ
        
  Description:
  This function configure and enable the interrupt for IRQ
*****************************************************************************/
void PDI_IRQ_Interrupt()
{
    INIT_ESC_INT;
}
/*******************************************************************************
  Function:
    void PDI_Init_SYNC_Interrupts(void)

  Summary:
    The function configure and enable SYNC0 and SYNC1 interrupt.
        
  Description:
	The function configure and enable SYNC0 and SYNC1 interrupt. It is platform dependent. INIT_SYNCx_INT is a macro which will configure the external interrupt and ENABLE_SYNCx_INT macro for enabling interrupt for the particular lines.
  *****************************************************************************/

void PDI_Init_SYNC_Interrupts()
{
    
#ifdef DC_SUPPORTED
	INIT_SYNC0_INT
  	INIT_SYNC1_INT
    ACK_SYNC0_INT
 // ACK_SYNC1_INT
    ENABLE_SYNC0_INT;
 // ENABLE_SYNC1_INT;
#endif
}


/*******************************************************************************
  Function:
  void HW_SetLed(UINT8 RunLed,UINT8 ErrLed)

  Summary:
   This function set the Error LED if it is required.
        
  Description:
  LAN9252 does not support error LED. So this feature has to be enabled by PDI SOC if it is needed.

 Parameters:
 RunLed  - It is not used. This will be set by LAN9252.    
 ErrLed      -  1- ON, 0-0FF.
  *****************************************************************************/
void HW_SetLed(UINT8 RunLed,UINT8 ErrLed)
{
    /* Here RunLed is not used. Because on chip supported RUN Led is available*/    
    //LED_ECATRED   = ErrLed;
}


/*******************************************************************************
  Function:
    void __ISR(_EXTERNAL_0_VECTOR, ipl7srs) ExtInterruptHandler(void)

  Summary:
    Interrupt service routine for the PDI interrupt from the EtherCAT Slave Controller

  *****************************************************************************/

void __attribute__( (interrupt, no_auto_psv) ) _INT0Interrupt( void )
{
    /* reset the interrupt flag */
   ACK_ESC_INT

   PDI_Isr();
}

#ifdef DC_SUPPORTED
/*******************************************************************************
  Function:
    void __ISR(_EXTERNAL_1_VECTOR, ipl5) Sync0Isr(void)

  Summary:
    Interrupt service routine for the interrupts from SYNC0
  *****************************************************************************/

void __attribute__ ( (interrupt, no_auto_psv) ) _INT1Interrupt( void )
{
   
    /* reset the interrupt flag */
    ACK_SYNC0_INT;
    
    Sync0_Isr();
}

/*******************************************************************************
  Function:
    void __ISR(_EXTERNAL_2_VECTOR, ipl4) Sync1Isr(void)


  Summary:
    Interrupt service routine for the interrupts from SYNC1
  *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _INT2Interrupt( void )
{
    
    /* reset the interrupt flag */
    ACK_SYNC1_INT;

    Sync1_Isr();
}
#endif //DC_SUPPORTED

#if ECAT_TIMER_INT
/*******************************************************************************
  Function:
    void __ISR(_TIMER_5_VECTOR,IPL3AUTO) _TIMER5_HANDLER(void)


  Summary:
    Interrupt service routine for the interrupts from TIMER5
  *****************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    /* Interrupt Service Routine code goes here */
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag 
  
    ECAT_CheckTimer();
}

#endif

/* *****************************************************************************
 End of File
 */
