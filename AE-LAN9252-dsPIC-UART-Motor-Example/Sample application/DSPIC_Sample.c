/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
\addtogroup DSPIC_Sample DSPIC_Sample
@{
*/

/**
\file DSPIC_Sample.c
\brief Implementation

\version 1.0.0.11
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"
#include "applInterface.h"
#include <xc.h>
#include "uart2.h"
#include "SensoredBLDC.h"

#define _DSPIC__SAMPLE_ 1
#include "DSPIC_Sample.h"
#undef _DSPIC__SAMPLE_
/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/
typedef struct
{
    BOOL enableUart; /* Subindex1 - enable uart */
    BOOL stopSelectionBit; /* Subindex2 - stop selection bit */
    UINT8 parityAndDataSelectionBits; /* Subindex4 - parity and data selection bits */
    uint32_t buadRate;
} UART_CONFIGDATA ;

//Based on default of values of U2MODE (=0xC008) and U2STA (=0x05D0)
static UART_CONFIGDATA uart_config ={1, 0, 0, 9600}; //these values are given in DSPIC_Sample.xlsx


/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void    APPL_AckErrorInd(UINT16 stateTrans)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
             all general settings were checked to start the mailbox handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
           all general settings were checked to start the input handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize)
{
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;

#if COE_SUPPORTED
    UINT16 PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM * pPDO = NULL;
    UINT16 PDOSubindex0 = 0;
    UINT32 *pPDOEntry = NULL;
    UINT16 PDOEntryCnt = 0;
   
    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);
        if(pPDO != NULL)
        {
            PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
                pPDOEntry = (UINT32 *)((UINT16 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3)/2);    //goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                OutputSize += (UINT16) ((*pPDOEntry) & 0xFF);
            }
        }
        else
        {
            /*assigned PDO was not found in object dictionary. return invalid mapping*/
            OutputSize = 0;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    OutputSize = (OutputSize + 7) >> 3;

    if(result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);
            if(pPDO != NULL)
            {
                PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
                for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
                    pPDOEntry = (UINT32 *)((UINT16 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3)/2);    //goto PDO entry
                    // we increment the expected output size depending on the mapped Entry
                    InputSize += (UINT16) ((*pPDOEntry) & 0xFF);
                }
            }
            else
            {
                /*assigned PDO was not found in object dictionary. return invalid mapping*/
                InputSize = 0;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }
    InputSize = (InputSize + 7) >> 3;

#else
#if _WIN32
   #pragma message ("Warning: Define 'InputSize' and 'OutputSize'.")
#else
    #warning "Define 'InputSize' and 'OutputSize'."
#endif
#endif

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
}
UINT32 gCounter = 0;
/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data

\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_InputMapping(UINT16* pData)
{
   UINT16 j = 0;
   UINT16 *pTmpData = (UINT16 *)pData;

   /* we go through all entries of the TxPDO Assign object to get the assigned TxPDOs */
   for (j = 0; j < sTxPDOassign.u16SubIndex0; j++)
   {
      switch (sTxPDOassign.aEntries[j])
      {
      /* TxPDO 1 */
      case 0x1A00:

          
         *pTmpData++ = (((UINT16 *) &Uart_input0x6000)[1]);
         *pTmpData++ = (((UINT16 *) &Uart_status0x6021)[1]);
         *pTmpData++ = (((UINT16 *) &Motor_ctrl_status0x6030)[1]);
         *pTmpData++ = (((UINT16 *) &Motor_ctrl_status0x6030)[2]);
         *pTmpData++ = (((UINT16 *) &Motor_ctrl_status0x6030)[3]);
         *pTmpData++ = (((UINT16 *) &Motor_ctrl_status0x6030)[4]);
         *pTmpData = (((UINT16 *) &Motor_ctrl_status0x6030)[5]);
         
         break;
      }
   }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;

    /* we go through all entries of the RxPDO Assign object to get the assigned RxPDOs */
    for (j = 0; j < sRxPDOassign.u16SubIndex0; j++)
    {
        switch (sRxPDOassign.aEntries[j])
        {
        /* RxPDO 1*/
        case 0x1600:
            ((UINT16 *) &Uart_output0x7000)[1] = *pTmpData++;
            ((UINT16 *) &Configure_uart0x8000)[1] = *pTmpData++;
            ((UINT16 *) &Configure_uart0x8000)[2] = *pTmpData++;
            ((UINT16 *) &Configure_uart0x8000)[3] = *pTmpData++;
            ((UINT16 *) &Motor_ctrl_output0x7011)[1] = *pTmpData++;
            ((UINT16 *) &Motor_ctrl_output0x7011)[2] = *pTmpData++;
            ((UINT16 *) &Motor_ctrl_output0x7011)[3] = *pTmpData++;
            ((UINT16 *) &Motor_ctrl_output0x7011)[4] = *pTmpData;
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR 
            or from the mainloop if no synchronisation is supported
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{
    APPL_UpdateUARTConfig();
  
    //When UART data is available
    if(!UART2_ReceiveBufferIsEmpty())
    {
        UART2_ReadBuffer(&Uart_input0x6000.Uart_read_buffer, 1);
        Uart_status0x6021.Rx_ready = true; //update rx_ready to twincat master
    }

   
    if(true == Configure_uart0x8000.Tx_ready) //Write UART only if tx_ready is high from twincat master
    {
      UART2_WriteBuffer((uint8_t *) &(Uart_output0x7000.Uart_write_buffer), 1);
    }
   
    if (Motor_ctrl_output0x7011.Start) 
    {
        if(Motor_ctrl_output0x7011.Speed)  //default speed is given in DSPIC_Sample.xlsx
        {
            gCounter = Motor_ctrl_output0x7011.Speed;
        }           
    }
    
    
    /*Motor Control State Processing*/
    switch (Flags.MotorState) {
        case MC_STOPPED:
        {
            DisableMotCon();
            if ( Motor_ctrl_output0x7011.Start || (!S2)) 
            { //Start the motor via EtherCAT Master or on board switch
                Flags.MotorState = MC_STARTING;
                if (!S2) 
                {
                    Motor_ctrl_output0x7011.Direction = !Motor_ctrl_output0x7011.Direction;      //Reverse the motor each time the switch is pressed         
                    Flags.Direction = Motor_ctrl_output0x7011.Direction;
                }
            }
            Motor_ctrl_status0x6030.Motor_state = MC_STOPPED; //Set MC State for EtherCat 
            IFS0bits.T3IF = 0; //Reset T3 Stall detect timer overflow flag
        }
            break;
        case MC_STARTING:
        {
 
            Flags.Direction = Motor_ctrl_output0x7011.Direction;
            LATAbits.LATA10 = 1; //Turn ON the D12 LED
            LATAbits.LATA3 = 1; //Enable the MCP8026 
            Motor_ctrl_status0x6030.Motor_state = MC_STARTING; //Set MC State for EtherCat
            ChargeCaps(); //Bootstrap capacitor pre charging
            DelayNmSec(2); //Bootstrap Charge delay                
            HallValue = Hall_Convert(); //Convert the Hall signals 
            EnableMotCon();
            Flags.MotorState = MC_RUNNING;
        }
            break;
        case MC_RUNNING:
        {
            if (IFS0bits.T3IF) {
                Motor_ctrl_status0x6030.Motor_state = MC_FAULT; //If stalled Set MC State for EtherCat
                LoopCount = 1000; //Set a Delay before moving to the STOPPED state, ensures Transmission of the Fault Over EtherCaT
                Flags.MotorState = MC_FAULT;
                break;
            } 
            else 
            {
                Motor_ctrl_status0x6030.Motor_state = MC_RUNNING; //Set MC State for EtherCat
            }
            //Check the state and adjust the motor speed
            if (!Motor_ctrl_output0x7011.Start && (!S2)) 
            {
                if(MDC < TEST_DC) MDC += SLEW_RATE; //Use this DC if the Switch is pressed, Runs the motor independent of EtherCat
                break;
            } else if (!Motor_ctrl_output0x7011.Start) 
            {
                Flags.MotorState = MC_STOPPED;
                break;
            } else if (gCounter < LOWER_LIMIT) MDC = LOWER_LIMIT; //Set a Min Limit so the motor does not stall
                else if (MDC < gCounter) {
                    MDC += SLEW_RATE;
                    LATAbits.LATA10 = !LATAbits.LATA10; //Toggle the LED D12 while ramping speed
                     break;
                }
                else if (MDC > gCounter){ 
                    MDC -= SLEW_RATE;
                    break;
                }            
            else MDC = gCounter;
        }
            break;
        case MC_STOPPING:
        {
            /*TODO*/
        }
            break;
        case MC_FAULT:
        {
            ResetMotCon();
            // Non Blocking Delay to ensure the Fault State is transmitted over EtherCAT
            if (LoopCount-- == 0)Flags.MotorState = MC_STOPPED;
         }
            break;
        default:
            break;
    }
    
    Motor_ctrl_status0x6030.Speed = gCounter;
    Motor_ctrl_status0x6030.Direction = Motor_ctrl_output0x7011.Direction;

}


void APPL_UpdateUARTConfig(void)
{
    BOOL isModified = false;
    if(Configure_uart0x8000.Buadrate != uart_config.buadRate)
    {
        uart_config.buadRate = Configure_uart0x8000.Buadrate;
        isModified = true;
    }

    if(Configure_uart0x8000.EnableUart != uart_config.enableUart)
    {
        uart_config.enableUart = Configure_uart0x8000.EnableUart;
        isModified = true;
    }

    if(uart_config.parityAndDataSelectionBits != Configure_uart0x8000.ParityAndDataSelectionBits)
    {
        /*
         0x3 = 9-bit data, no parity
         0x2 = 8-bit data, odd parity
         0x1 = 8-bit data, even parity
         0x0 = 8-bit data, no parity
         */
        uart_config.parityAndDataSelectionBits = Configure_uart0x8000.ParityAndDataSelectionBits;
        isModified = true;
    }

    if(uart_config.stopSelectionBit!= Configure_uart0x8000.StopSelectionBit)
    {
      /* 
       * uart_config.stopSelectionBit == 1 --> 2 Stop bits
         uart_config.stopSelectionBit ==0 --> 1 Stop bit
       */        
        uart_config.stopSelectionBit = Configure_uart0x8000.StopSelectionBit;
        isModified = true;
    }
            
    if(isModified)
    {
         unsigned short BRG = (69093750 /(4*uart_config.buadRate))-1; //BRGH =1
            UART2_Initialize(BRG, uart_config.enableUart, uart_config.parityAndDataSelectionBits, uart_config.stopSelectionBit);
    }
           
}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GetDeviceID()
{
#if _WIN32
   #pragma message ("Warning: Implement explicit Device ID latching")
#else
    #warning "Implement explicit Device ID latching"
#endif
    /* Explicit Device 5 is expected by Explicit Device ID conformance tests*/
    return 0x5;
}
#endif



#if USE_DEFAULT_MAIN
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is the main function

*////////////////////////////////////////////////////////////////////////////////////////
#if _PIC24
int main(void)
#else
void main(void)
#endif
{
    /* initialize the Hardware and the EtherCAT Slave Controller */
#if FC1100_HW
    if(HW_Init())
    {
        HW_Release();
        return;
    }
#else
    HW_Init();
#endif
    MainInit();

    bRunApplication = TRUE;
    do
    {
        MainLoop();
        
    } while (bRunApplication == TRUE);

    HW_Release();
#if _PIC24
    return 0;
#endif
}
#endif //#if USE_DEFAULT_MAIN
/** @} */


