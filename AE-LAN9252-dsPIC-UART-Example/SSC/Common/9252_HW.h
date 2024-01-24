/*******************************************************************************
 LAN9252 - Hardware Abtraction Layer header file.

  Company:
    Microchip Technology Inc.

  File Name:
    9252_HW.h

  Description:
    This file contains the defines, function protypes for LAN9252 Hardware Abtraction Layer

  Change History:
    Version		Changes
	0.1			Initial version.
	0.2			-
	0.3			-
	0.4			-
	1.0			-
	1.3			- *Re-arranged the functions. 
    1.4         - *Removed unwanted functions. 
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

#ifndef _9252_HW_H_
#define _9252_HW_H_

///////////////////////////////////////////////////////////////////////////////
// Includes
#include "ecat_def.h"


///////////////////////////////////////////////////////////////////////////////
//9252 HW DEFINES
#define ECAT_REG_BASE_ADDR              0x0300

#define CSR_DATA_REG_OFFSET             0x00
#define CSR_CMD_REG_OFFSET              0x04
#define PRAM_READ_ADDR_LEN_OFFSET       0x08
#define PRAM_READ_CMD_OFFSET            0x0c
#define PRAM_WRITE_ADDR_LEN_OFFSET      0x10
#define PRAM_WRITE_CMD_OFFSET           0x14

#define PRAM_SPACE_AVBL_COUNT_MASK      0x1f00
#define IS_PRAM_SPACE_AVBL_MASK         0x01


#define CSR_DATA_REG                    (ECAT_REG_BASE_ADDR+CSR_DATA_REG_OFFSET)
#define CSR_CMD_REG                     (ECAT_REG_BASE_ADDR+CSR_CMD_REG_OFFSET)
#define PRAM_READ_ADDR_LEN_REG          (ECAT_REG_BASE_ADDR+PRAM_READ_ADDR_LEN_OFFSET)
#define PRAM_READ_CMD_REG               (ECAT_REG_BASE_ADDR+PRAM_READ_CMD_OFFSET)
#define PRAM_WRITE_ADDR_LEN_REG         (ECAT_REG_BASE_ADDR+PRAM_WRITE_ADDR_LEN_OFFSET)
#define PRAM_WRITE_CMD_REG              (ECAT_REG_BASE_ADDR+PRAM_WRITE_CMD_OFFSET)

#define PRAM_READ_FIFO_REG              0x04
#define PRAM_WRITE_FIFO_REG             0x20

#define HBI_INDEXED_DATA0_REG           0x04
#define HBI_INDEXED_DATA1_REG           0x0c
#define HBI_INDEXED_DATA2_REG           0x14

#define HBI_INDEXED_INDEX0_REG          0x00
#define HBI_INDEXED_INDEX1_REG          0x08
#define HBI_INDEXED_INDEX2_REG          0x10

#define HBI_INDEXED_PRAM_READ_WRITE_FIFO    0x18

#define PRAM_RW_ABORT_MASK      (0x4000)
#define PRAM_RW_BUSY_16B        (0x8000)


///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Hardware timer settings

#define ECAT_TIMER_INC_P_MS              8750 /**< \brief 8750 ticks per ms*/


///////////////////////////////////////////////////////////////////////////////
// Interrupt and Timer Defines

#ifndef DISABLE_ESC_INT
    #define    DISABLE_ESC_INT()           {restore_intsts = PDI_Disable_Global_Interrupt();}
#endif
#ifndef ENABLE_ESC_INT
    #define    ENABLE_ESC_INT()           {PDI_Restore_Global_Interrupt(restore_intsts);}
#endif


//TODO
#ifndef HW_GetTimer
    #define HW_GetTimer()       (PDI_GetTimer()) /**< \brief Access to the hardware timer*/
#endif

#ifndef HW_ClearTimer
    #define HW_ClearTimer()       (PDI_ClearTimer()) /**< \brief Clear the hardware timer*/
#endif


#define HW_EscReadDWord(DWordValue, Address) HW_EscRead(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief 32Bit ESC read access*/
#define HW_EscReadMbxMem(pData,Address,Len) HW_EscRead(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len)) /**< \brief The mailbox data is stored in the local uC memory therefore the default read function is used.*/
#define HW_EscReadDWordIsr(DWordValue, Address) HW_EscReadIsr(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief Interrupt specific 32Bit ESC read access*/

#define HW_EscWriteDWord(DWordValue, Address) HW_EscWrite(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief 32Bit ESC write access*/
#define HW_EscWriteMbxMem(pData,Address,Len) HW_EscWrite(((MEM_ADDR *)(pData)),((UINT16)(Address)),(Len)) /**< \brief The mailbox data is stored in the local uC memory therefore the default write function is used.*/
#define HW_EscWriteDWordIsr(DWordValue, Address) HW_EscWriteIsr(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4) /**< \brief Interrupt specific 32Bit ESC write access*/

///////////////////////////////////////////////////////////////////////////////

#if _9252_HW_
    #define PROTO
#else
    #define PROTO extern
#endif

///////////////////////////////////////////////////////////////////////////////
// Global variables extern
PROTO volatile UINT32 restore_intsts;

///////////////////////////////////////////////////////////////////////////////
// Global functions prototype

PROTO void LAN9252_Init(void);
PROTO void HW_Release(void);

PROTO UINT16 HW_GetALEventRegister(void);
PROTO UINT16 HW_GetALEventRegister_Isr(void);

PROTO void HW_EscRead( MEM_ADDR * pData, UINT16 Address, UINT16 Len );
PROTO void HW_EscReadIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len );

PROTO void HW_EscWrite( MEM_ADDR *pData, UINT16 Address, UINT16 Len );
PROTO void HW_EscWriteIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len );

PROTO void HW_SetLed(UINT8 RunLed,UINT8 ErrLed);

PROTO UINT16 PDI_GetTimer();
PROTO void PDI_ClearTimer();

PROTO void PDI_Restore_Global_Interrupt(UINT32 int_sts);
PROTO UINT32 PDI_Disable_Global_Interrupt();
PROTO void PDI_Enable_Global_interrupt();
PROTO void PDIReadReg(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count);
PROTO void PDIWriteReg( UINT8 const *WriteBuffer, UINT16 Address, UINT16 Count);
PROTO void PDIReadLAN9252DirectReg( UINT16 Address, UINT8 *data);
PROTO void PDIWriteLAN9252DirectReg( UINT32 Val, UINT16 Address);
PROTO void PDI_Init();
PROTO void PDI_Timer_Interrupt();
PROTO void PDI_Init_SYNC_Interrupts();
PROTO void PDI_IRQ_Interrupt();
    

#undef    PROTO

#endif
