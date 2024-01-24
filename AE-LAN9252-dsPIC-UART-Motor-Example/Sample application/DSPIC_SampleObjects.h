/*
* This source file is part of the EtherCAT Slave Stack Code licensed by Beckhoff Automation GmbH & Co KG, 33415 Verl, Germany.
* The corresponding license agreement applies. This hint shall not be removed.
*/

/**
* \addtogroup DSPIC_Sample DSPIC_Sample
* @{
*/

/**
\file DSPIC_SampleObjects
\author ET9300Utilities.ApplicationHandler (Version 1.5.0.0) | EthercatSSC@beckhoff.com

\brief DSPIC_Sample specific objects<br>
\brief NOTE : This file will be overwritten if a new object dictionary is generated!<br>
*/

#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
#define PROTO
#else
#define PROTO extern
#endif
/******************************************************************************
*                    Object 0x1600 : Output mapping 0
******************************************************************************/
/**
* \addtogroup 0x1600 0x1600 | Output mapping 0
* @{
* \brief Object 0x1600 (Output mapping 0) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x7000.1<br>
* SubIndex 2 - Padding entry (8Bit)<br>
* SubIndex 3 - Reference to 0x8000.1<br>
* SubIndex 4 - Reference to 0x8000.2<br>
* SubIndex 5 - Reference to 0x8000.3<br>
* SubIndex 6 - Reference to 0x8000.4<br>
* SubIndex 7 - Reference to 0x8000.5<br>
* SubIndex 8 - Padding entry (5Bit)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1600[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x7000.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Padding entry (8Bit) */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex3 - Reference to 0x8000.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex4 - Reference to 0x8000.2 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex5 - Reference to 0x8000.3 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex6 - Reference to 0x8000.4 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex7 - Reference to 0x8000.5 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex8 - Padding entry (5Bit) */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1600[] = "Output mapping 0\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000"
"SubIndex 004\000"
"SubIndex 005\000"
"SubIndex 006\000"
"SubIndex 007\000"
"SubIndex 008\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x7000.1 */
UINT32 SI2; /* Subindex2 - Padding entry (8Bit) */
UINT32 SI3; /* Subindex3 - Reference to 0x8000.1 */
UINT32 SI4; /* Subindex4 - Reference to 0x8000.2 */
UINT32 SI5; /* Subindex5 - Reference to 0x8000.3 */
UINT32 SI6; /* Subindex6 - Reference to 0x8000.4 */
UINT32 SI7; /* Subindex7 - Reference to 0x8000.5 */
UINT32 SI8; /* Subindex8 - Padding entry (5Bit) */
} OBJ_STRUCT_PACKED_END
TOBJ1600;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1600 OutputMapping00x1600
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={8,0x70000108,0x00000008,0x80000120,0x80000208,0x80000301,0x80000401,0x80000501,0x00000005}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1601 : motor_ctrl_output process data mapping
******************************************************************************/
/**
* \addtogroup 0x1601 0x1601 | motor_ctrl_output process data mapping
* @{
* \brief Object 0x1601 (motor_ctrl_output process data mapping) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x7011.1<br>
* SubIndex 2 - Reference to 0x7011.2<br>
* SubIndex 3 - Reference to 0x7011.3<br>
* SubIndex 4 - Padding entry (14Bit)<br>
* SubIndex 5 - Padding entry (16Bit)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1601[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x7011.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Reference to 0x7011.2 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex3 - Reference to 0x7011.3 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex4 - Padding entry (14Bit) */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex5 - Padding entry (16Bit) */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1601[] = "motor_ctrl_output process data mapping\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000"
"SubIndex 004\000"
"SubIndex 005\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x7011.1 */
UINT32 SI2; /* Subindex2 - Reference to 0x7011.2 */
UINT32 SI3; /* Subindex3 - Reference to 0x7011.3 */
UINT32 SI4; /* Subindex4 - Padding entry (14Bit) */
UINT32 SI5; /* Subindex5 - Padding entry (16Bit) */
} OBJ_STRUCT_PACKED_END
TOBJ1601;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1601 Motor_ctrl_outputProcessDataMapping0x1601
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={5,0x70110120,0x70110201,0x70110301,0x0000000E,0x00000010}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1A00 : uart_input process data mapping
******************************************************************************/
/**
* \addtogroup 0x1A00 0x1A00 | uart_input process data mapping
* @{
* \brief Object 0x1A00 (uart_input process data mapping) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x6000.1<br>
* SubIndex 2 - Padding entry (8Bit)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1A00[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x6000.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex2 - Padding entry (8Bit) */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1A00[] = "uart_input process data mapping\000"
"SubIndex 001\000"
"SubIndex 002\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x6000.1 */
UINT32 SI2; /* Subindex2 - Padding entry (8Bit) */
} OBJ_STRUCT_PACKED_END
TOBJ1A00;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1A00 Uart_inputProcessDataMapping0x1A00
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={2,0x60000108,0x00000008}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1A02 : uart_status process data mapping
******************************************************************************/
/**
* \addtogroup 0x1A02 0x1A02 | uart_status process data mapping
* @{
* \brief Object 0x1A02 (uart_status process data mapping) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x6021.1<br>
* SubIndex 2 - Padding entry (7Bit)<br>
* SubIndex 3 - Padding entry (8Bit)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1A02[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x6021.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Padding entry (7Bit) */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex3 - Padding entry (8Bit) */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1A02[] = "uart_status process data mapping\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x6021.1 */
UINT32 SI2; /* Subindex2 - Padding entry (7Bit) */
UINT32 SI3; /* Subindex3 - Padding entry (8Bit) */
} OBJ_STRUCT_PACKED_END
TOBJ1A02;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1A02 Uart_statusProcessDataMapping0x1A02
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={3,0x60210101,0x00000007,0x00000008}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1A03 : motor_ctrl_status process data mapping
******************************************************************************/
/**
* \addtogroup 0x1A03 0x1A03 | motor_ctrl_status process data mapping
* @{
* \brief Object 0x1A03 (motor_ctrl_status process data mapping) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x6030.1<br>
* SubIndex 2 - Reference to 0x6030.2<br>
* SubIndex 3 - Reference to 0x6030.3<br>
* SubIndex 4 - Padding entry (15Bit)<br>
* SubIndex 5 - Padding entry (16Bit)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1A03[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x6030.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Reference to 0x6030.2 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex3 - Reference to 0x6030.3 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex4 - Padding entry (15Bit) */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex5 - Padding entry (16Bit) */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1A03[] = "motor_ctrl_status process data mapping\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000"
"SubIndex 004\000"
"SubIndex 005\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x6030.1 */
UINT32 SI2; /* Subindex2 - Reference to 0x6030.2 */
UINT32 SI3; /* Subindex3 - Reference to 0x6030.3 */
UINT32 SI4; /* Subindex4 - Padding entry (15Bit) */
UINT32 SI5; /* Subindex5 - Padding entry (16Bit) */
} OBJ_STRUCT_PACKED_END
TOBJ1A03;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1A03 Motor_ctrl_statusProcessDataMapping0x1A03
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={5,0x60300120,0x60300220,0x60300301,0x0000000F,0x00000010}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1C12 : SyncManager 2 assignment
******************************************************************************/
/**
* \addtogroup 0x1C12 0x1C12 | SyncManager 2 assignment
* @{
* \brief Object 0x1C12 (SyncManager 2 assignment) definition
*/
#ifdef _OBJD_
/**
* \brief Entry descriptions<br>
* 
* Subindex 0<br>
* Subindex 1 - n (the same entry description is used)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C12[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED16 , 0x10 , ACCESS_READ }};

/**
* \brief Object name definition<br>
* For Subindex 1 to n the syntax 'Subindex XXX' is used
*/
OBJCONST UCHAR OBJMEM aName0x1C12[] = "SyncManager 2 assignment\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16   u16SubIndex0;  /**< \brief Subindex 0 */
UINT16 aEntries[2];  /**< \brief Subindex 1 - 2 */
} OBJ_STRUCT_PACKED_END
TOBJ1C12;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1C12 sRxPDOassign
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={2,{0x1600,0x1601}}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1C13 : SyncManager 3 assignment
******************************************************************************/
/**
* \addtogroup 0x1C13 0x1C13 | SyncManager 3 assignment
* @{
* \brief Object 0x1C13 (SyncManager 3 assignment) definition
*/
#ifdef _OBJD_
/**
* \brief Entry descriptions<br>
* 
* Subindex 0<br>
* Subindex 1 - n (the same entry description is used)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C13[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED16 , 0x10 , ACCESS_READ }};

/**
* \brief Object name definition<br>
* For Subindex 1 to n the syntax 'Subindex XXX' is used
*/
OBJCONST UCHAR OBJMEM aName0x1C13[] = "SyncManager 3 assignment\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16   u16SubIndex0;  /**< \brief Subindex 0 */
UINT16 aEntries[3];  /**< \brief Subindex 1 - 3 */
} OBJ_STRUCT_PACKED_END
TOBJ1C13;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1C13 sTxPDOassign
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={3,{0x1A00,0x1A02,0x1A03}}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x6000 : uart_input
******************************************************************************/
/**
* \addtogroup 0x6000 0x6000 | uart_input
* @{
* \brief Object 0x6000 (uart_input) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - uart_read_buffer<br>
* SubIndex 2<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6000[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED8 , 0x08 , ACCESS_READ | OBJACCESS_TXPDOMAPPING }, /* Subindex1 - uart_read_buffer */
{ DEFTYPE_NULL , 0x08 ,  OBJACCESS_TXPDOMAPPING }}; /* Subindex2 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x6000[] = "uart_input\000"
"uart_read_buffer\000"
"\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT8 Uart_read_buffer; /* Subindex1 - uart_read_buffer */
ALIGN8(SI2) /* Subindex2 */
} OBJ_STRUCT_PACKED_END
TOBJ6000;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ6000 Uart_input0x6000
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={1,0x00,0}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x6021 : uart_status
******************************************************************************/
/**
* \addtogroup 0x6021 0x6021 | uart_status
* @{
* \brief Object 0x6021 (uart_status) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - rx_ready<br>
* SubIndex 2 does not exists<br>
* SubIndex 3<br>
* SubIndex 4<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6021[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_BOOLEAN , 0x01 , ACCESS_READ | OBJACCESS_TXPDOMAPPING }, /* Subindex1 - rx_ready */
{ DEFTYPE_NULL , 0x00 , 0x0000 }, /* Subindex2 does not exists */
{ DEFTYPE_NULL , 0x07 ,  OBJACCESS_TXPDOMAPPING }, /* Subindex3 */
{ DEFTYPE_NULL , 0x08 ,  OBJACCESS_TXPDOMAPPING }}; /* Subindex4 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x6021[] = "uart_status\000"
"rx_ready\000"
"\000"
"\000"
"\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
BOOLEAN(Rx_ready); /* Subindex1 - rx_ready */
ALIGN7(SI3) /* Subindex3 */
ALIGN8(SI4) /* Subindex4 */
} OBJ_STRUCT_PACKED_END
TOBJ6021;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ6021 Uart_status0x6021
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={1,0x00,0,0}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x6030 : motor_ctrl_status
******************************************************************************/
/**
* \addtogroup 0x6030 0x6030 | motor_ctrl_status
* @{
* \brief Object 0x6030 (motor_ctrl_status) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - motor_state<br>
* SubIndex 2 - speed<br>
* SubIndex 3 - direction<br>
* SubIndex 4<br>
* SubIndex 5<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6030[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ | OBJACCESS_TXPDOMAPPING }, /* Subindex1 - motor_state */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ | OBJACCESS_TXPDOMAPPING }, /* Subindex2 - speed */
{ DEFTYPE_BOOLEAN , 0x01 , ACCESS_READ | OBJACCESS_TXPDOMAPPING }, /* Subindex3 - direction */
{ DEFTYPE_NULL , 0x0F ,  OBJACCESS_TXPDOMAPPING }, /* Subindex4 */
{ DEFTYPE_NULL , 0x10 ,  OBJACCESS_TXPDOMAPPING }}; /* Subindex5 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x6030[] = "motor_ctrl_status\000"
"motor_state\000"
"speed\000"
"direction\000"
"\000"
"\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 Motor_state; /* Subindex1 - motor_state */
UINT32 Speed; /* Subindex2 - speed */
BOOLEAN(Direction); /* Subindex3 - direction */
ALIGN15(SI4) /* Subindex4 */
UINT16 SI5; /* Subindex5 */
} OBJ_STRUCT_PACKED_END
TOBJ6030;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ6030 Motor_ctrl_status0x6030
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={3,0x00000000,0x00000000,0x00,0,0}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x7000 : uart_output
******************************************************************************/
/**
* \addtogroup 0x7000 0x7000 | uart_output
* @{
* \brief Object 0x7000 (uart_output) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - uart_write_buffer<br>
* SubIndex 2<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x7000[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED8 , 0x08 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex1 - uart_write_buffer */
{ DEFTYPE_NULL , 0x08 ,  OBJACCESS_RXPDOMAPPING }}; /* Subindex2 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x7000[] = "uart_output\000"
"uart_write_buffer\000"
"\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT8 Uart_write_buffer; /* Subindex1 - uart_write_buffer */
ALIGN8(SI2) /* Subindex2 */
} OBJ_STRUCT_PACKED_END
TOBJ7000;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ7000 Uart_output0x7000
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={1,0x00,0}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x7011 : motor_ctrl_output
******************************************************************************/
/**
* \addtogroup 0x7011 0x7011 | motor_ctrl_output
* @{
* \brief Object 0x7011 (motor_ctrl_output) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - speed<br>
* SubIndex 2 - start<br>
* SubIndex 3 - direction<br>
* SubIndex 4<br>
* SubIndex 5<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x7011[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex1 - speed */
{ DEFTYPE_BOOLEAN , 0x01 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex2 - start */
{ DEFTYPE_BOOLEAN , 0x01 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex3 - direction */
{ DEFTYPE_NULL , 0x0E ,  OBJACCESS_RXPDOMAPPING }, /* Subindex4 */
{ DEFTYPE_NULL , 0x10 ,  OBJACCESS_RXPDOMAPPING }}; /* Subindex5 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x7011[] = "motor_ctrl_output\000"
"speed\000"
"start\000"
"direction\000"
"\000"
"\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 Speed; /* Subindex1 - speed */
BOOLEAN(Start); /* Subindex2 - start */
BOOLEAN(Direction); /* Subindex3 - direction */
ALIGN14(SI4) /* Subindex4 */
UINT16 SI5; /* Subindex5 */
} OBJ_STRUCT_PACKED_END
TOBJ7011;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ7011 Motor_ctrl_output0x7011
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={3,0x00000050,0x00,0x00,0,0}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x8000 : configure_uart
******************************************************************************/
/**
* \addtogroup 0x8000 0x8000 | configure_uart
* @{
* \brief Object 0x8000 (configure_uart) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - buadrate<br>
* SubIndex 2 - parity and data selection bits<br>
* SubIndex 3 - enable uart<br>
* SubIndex 4 - tx_ready<br>
* SubIndex 5 - stop selection bit<br>
* SubIndex 6<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x8000[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex1 - buadrate */
{ DEFTYPE_UNSIGNED8 , 0x08 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex2 - parity and data selection bits */
{ DEFTYPE_BOOLEAN , 0x01 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex3 - enable uart */
{ DEFTYPE_BOOLEAN , 0x01 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex4 - tx_ready */
{ DEFTYPE_BOOLEAN , 0x01 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING }, /* Subindex5 - stop selection bit */
{ DEFTYPE_NULL , 0x05 ,  OBJACCESS_RXPDOMAPPING }}; /* Subindex6 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x8000[] = "configure_uart\000"
"buadrate\000"
"parity and data selection bits\000"
"enable uart\000"
"tx_ready\000"
"stop selection bit\000"
"\000\377";
#endif //#ifdef _OBJD_

#ifndef _DSPIC__SAMPLE_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 Buadrate; /* Subindex1 - buadrate */
UINT8 ParityAndDataSelectionBits; /* Subindex2 - parity and data selection bits */
BOOLEAN(EnableUart); /* Subindex3 - enable uart */
BOOLEAN(Tx_ready); /* Subindex4 - tx_ready */
BOOLEAN(StopSelectionBit); /* Subindex5 - stop selection bit */
ALIGN5(SI6) /* Subindex6 */
} OBJ_STRUCT_PACKED_END
TOBJ8000;
#endif //#ifndef _DSPIC__SAMPLE_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ8000 Configure_uart0x8000
#if defined(_DSPIC__SAMPLE_) && (_DSPIC__SAMPLE_ == 1)
={5,0x00000096,0x00,0x01,0x00,0x00,0}
#endif
;
/** @}*/







#ifdef _OBJD_
TOBJECT    OBJMEM ApplicationObjDic[] = {
/* Object 0x1600 */
{NULL , NULL ,  0x1600 , {DEFTYPE_PDOMAPPING , 8 | (OBJCODE_REC << 8)} , asEntryDesc0x1600 , aName0x1600 , &OutputMapping00x1600 , NULL , NULL , 0x0000 },
/* Object 0x1601 */
{NULL , NULL ,  0x1601 , {DEFTYPE_PDOMAPPING , 5 | (OBJCODE_REC << 8)} , asEntryDesc0x1601 , aName0x1601 , &Motor_ctrl_outputProcessDataMapping0x1601 , NULL , NULL , 0x0000 },
/* Object 0x1A00 */
{NULL , NULL ,  0x1A00 , {DEFTYPE_PDOMAPPING , 2 | (OBJCODE_REC << 8)} , asEntryDesc0x1A00 , aName0x1A00 , &Uart_inputProcessDataMapping0x1A00 , NULL , NULL , 0x0000 },
/* Object 0x1A02 */
{NULL , NULL ,  0x1A02 , {DEFTYPE_PDOMAPPING , 3 | (OBJCODE_REC << 8)} , asEntryDesc0x1A02 , aName0x1A02 , &Uart_statusProcessDataMapping0x1A02 , NULL , NULL , 0x0000 },
/* Object 0x1A03 */
{NULL , NULL ,  0x1A03 , {DEFTYPE_PDOMAPPING , 5 | (OBJCODE_REC << 8)} , asEntryDesc0x1A03 , aName0x1A03 , &Motor_ctrl_statusProcessDataMapping0x1A03 , NULL , NULL , 0x0000 },
/* Object 0x1C12 */
{NULL , NULL ,  0x1C12 , {DEFTYPE_UNSIGNED16 , 2 | (OBJCODE_ARR << 8)} , asEntryDesc0x1C12 , aName0x1C12 , &sRxPDOassign , NULL , NULL , 0x0000 },
/* Object 0x1C13 */
{NULL , NULL ,  0x1C13 , {DEFTYPE_UNSIGNED16 , 3 | (OBJCODE_ARR << 8)} , asEntryDesc0x1C13 , aName0x1C13 , &sTxPDOassign , NULL , NULL , 0x0000 },
/* Object 0x6000 */
{NULL , NULL ,  0x6000 , {DEFTYPE_RECORD , 2 | (OBJCODE_REC << 8)} , asEntryDesc0x6000 , aName0x6000 , &Uart_input0x6000 , NULL , NULL , 0x0000 },
/* Object 0x6021 */
{NULL , NULL ,  0x6021 , {DEFTYPE_RECORD , 4 | (OBJCODE_REC << 8)} , asEntryDesc0x6021 , aName0x6021 , &Uart_status0x6021 , NULL , NULL , 0x0000 },
/* Object 0x6030 */
{NULL , NULL ,  0x6030 , {DEFTYPE_RECORD , 5 | (OBJCODE_REC << 8)} , asEntryDesc0x6030 , aName0x6030 , &Motor_ctrl_status0x6030 , NULL , NULL , 0x0000 },
/* Object 0x7000 */
{NULL , NULL ,  0x7000 , {DEFTYPE_RECORD , 2 | (OBJCODE_REC << 8)} , asEntryDesc0x7000 , aName0x7000 , &Uart_output0x7000 , NULL , NULL , 0x0000 },
/* Object 0x7011 */
{NULL , NULL ,  0x7011 , {DEFTYPE_RECORD , 5 | (OBJCODE_REC << 8)} , asEntryDesc0x7011 , aName0x7011 , &Motor_ctrl_output0x7011 , NULL , NULL , 0x0000 },
/* Object 0x8000 */
{NULL , NULL ,  0x8000 , {DEFTYPE_RECORD , 6 | (OBJCODE_REC << 8)} , asEntryDesc0x8000 , aName0x8000 , &Configure_uart0x8000 , NULL , NULL , 0x0000 },
{NULL,NULL, 0xFFFF, {0, 0}, NULL, NULL, NULL, NULL}};
#endif    //#ifdef _OBJD_
#undef PROTO

/** @}*/
#define _DSPIC__SAMPLE_OBJECTS_H_
