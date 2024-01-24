/*******************************************************************************
 PIC32 SPI Interface Driver

  Company:
    Microchip Technology Inc.

  File Name:
    SPIDriver.c

  Summary:
    Contains the functional implementation of dsPIC33 SPI Interface Driver

  Description:
    This file contains the functional implementation of dsPIC33 SPI Interface Driver
	
  Change History:
    Version		Changes
	0.1			Initial version.
	0.2			-
	0.3			-	
	0.4 		-
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
 
#include "dsPIC33SPIDriver.h"
#include "9252_HW.h"

#define ADDRESS_AUTO_INCREMENT 0x4000

/*******************************************************************************
  Function:
	UINT32 SPIReadDWord (UINT16 Address, UINT8 *ReadBuffer)
  Summary:
    This function reads the LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIReadDWord (UINT16 Address, UINT8 *ReadBuffer)
{
   //Assert CS line
    CSLOW();
 
    //Write Command
    SPIWriteByte(CMD_FAST_READ);
    //Write Address
    SPISendAddr(Address);
      
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);
    //Read Bytes
    *(ReadBuffer + 0) = SPIReadByte();
    *(ReadBuffer + 1) = SPIReadByte();
    *(ReadBuffer + 2) = SPIReadByte();
    *(ReadBuffer + 3) = SPIReadByte();
    
    //De-Assert CS line
    CSHIGH();
}

/*******************************************************************************
  Function:
	void SPISendAddr (UINT16 Address)
  Summary:
    This function write address to SPI data bus.        
  
*****************************************************************************/
void SPISendAddr (UINT16 Address)
{
    //Write Address
    SPIWriteByte((UINT8)((Address & 0xFF00) >> 8));
    SPIWriteByte((UINT8)(Address & 0xFF));
}

/*******************************************************************************
  Function:
	void SPIReadBurstMode (UINT8 *ReadBuffer)
  Summary:
    This function read 4 bytes continuosly.        
  
*****************************************************************************/
void SPIReadBurstMode (UINT8 *ReadBuffer)
{
    //Read Bytes
    *ReadBuffer = SPIReadByte();
    *(ReadBuffer + 1) = SPIReadByte();
    *(ReadBuffer + 2) = SPIReadByte();
    *(ReadBuffer + 3) = SPIReadByte();
}

/*******************************************************************************
  Function:
	void SPIWriteBurstMode (void const *Val)
  Summary:
    This function writes 4 bytes continuosly.        
  
*****************************************************************************/
void SPIWriteBurstMode (UINT8 const *Val)
{   
    //Write Bytes
    SPIWriteByte(*Val); 
    SPIWriteByte(*(Val + 1)); 
    SPIWriteByte(*(Val + 2));
    SPIWriteByte(*(Val + 3));
}


/*******************************************************************************
  Function:
	void SPIWriteBytes(UINT16 Address, UINT8 const *Val, UINT8 nLength)
  Summary:
    This function writes the LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIWriteBytes(UINT16 Address, UINT8 const *Val, UINT16 nLength)
{
    //Assert CS line
    CSLOW();
    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);
    //Write Address
    SPISendAddr(Address | ADDRESS_AUTO_INCREMENT);
 
    //Write Bytes
    while(nLength--)
    {
     	SPIWriteByte(*(Val++));
    }
        
    //De-Assert CS line
    CSHIGH();
}

/*******************************************************************************
  Function:
	void SPIWriteDWord (UINT16 Address, UINT32 Val)
  Summary:
    This function writes the LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIWriteDWord (UINT16 Address, UINT8 const *Val)
{
    //Assert CS line
    CSLOW();
    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);
    //Write Address
    SPISendAddr(Address); 
    //Write Bytes
    SPIWriteByte(*Val);
    SPIWriteByte(*(Val + 1));
    SPIWriteByte(*(Val + 2));
    SPIWriteByte(*(Val + 3));

    //De-Assert CS line
    CSHIGH();
}

/*******************************************************************************
  Function:
   void SPIReadRegUsingCSR(UINT8 *ReadBuffer, UINT16 Address, UINT8 Count)
  Summary:
    This function reads the EtherCAT core registers using LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIReadRegUsingCSR(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
{
    UINT8 u8Buffer[4] = {0};
 
    u8Buffer[0] = (UINT8)(Address & 0x00FF);
    u8Buffer[1] = (UINT8)((Address & 0xFF00) >> 8);
    u8Buffer[2] = (UINT8) Count;
    u8Buffer[3] = ESC_READ_BYTE;

    SPIWriteDWord (ESC_CSR_CMD_REG, &u8Buffer[0]);
#if 0
    do
    {
       SPIReadDWord (ESC_CSR_CMD_REG, &u8Buffer[0]);
		
    }while(u8Buffer[3] & ESC_CSR_BUSY);
#endif
    
    DELAY_1US();
    
    SPIReadDWord (ESC_CSR_DATA_REG, ReadBuffer);
    
       
    return;
}

/*******************************************************************************
  Function:
   void SPIWriteRegUsingCSR( UINT8 *WriteBuffer, UINT16 Address)
  Summary:
    This function writes the EtherCAT core registers using LAN9252 CSR registers.        
  
*****************************************************************************/
void SPIWriteRegUsingCSR( UINT8 *WriteBuffer, UINT16 Address)
{
    UINT8 u8Buffer[4] = {0};
      

    SPIWriteDWord (ESC_CSR_DATA_REG, WriteBuffer);
    
    u8Buffer[0] = (UINT8)(Address & 0x00FF);
    u8Buffer[1] = (UINT8)((Address & 0xFF00) >> 8);
    u8Buffer[2] = 4;
    u8Buffer[3] = ESC_WRITE_BYTE;

    SPIWriteDWord (ESC_CSR_CMD_REG, &u8Buffer[0]);
    DELAY_1US();
#if 0
    do
    {
        SPIReadDWord (ESC_CSR_CMD_REG, &u8Buffer[0]);

    }while(u8Buffer[3] & ESC_CSR_BUSY);
#endif

    return;
}


/*******************************************************************************
  Function:
   void SPIReadPDRamRegister(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function reads the PDRAM using LAN9252 FIFO.        
  
*****************************************************************************/
void SPIReadPDRamRegister(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
{
    UINT8 u8Buffer[4] = {0};
    UINT16 u16Buffer[4] = {0};
    UINT16 i, nlength, nBitPosition, act_read_bytes;//nReadSpaceAvblCount

     /*Reset or Abort any previous commands.*/
    u16Buffer[1] = PRAM_RW_ABORT_MASK;                                                
    SPIWriteDWord (PRAM_READ_CMD_REG, (UINT8*)&u16Buffer[0]);
    
    /*Write Address and Length Register (PRAM_READ_ADDR_LEN) with the
    starting UINT8 address and length) and Set PRAM Read Busy (PRAM_READ_BUSY) bit(-EtherCAT Process RAM Read Command Register)
    to start read operation*/
    u16Buffer[0] = Address;
    u16Buffer[1] = Count;	
    u16Buffer[2] = 0x0;
    u16Buffer[3] = 0x8000;
    SPIWriteBytes (PRAM_READ_CMD_REG, (UINT8*)&u16Buffer[0],8);
    /*Read PRAM Read Data Available (PRAM_READ_AVAIL) bit is set*/
    do
    {
        SPIReadDWord (PRAM_READ_CMD_REG,(UINT8 *)&u8Buffer[0]);
    }while(!(u8Buffer[0] & IS_PRAM_SPACE_AVBL_MASK));

    //nReadSpaceAvblCount = u8Buffer[1] & (PRAM_SPACE_AVBL_COUNT_MASK>>8);

    /*Fifo registers are aliased address. In indexed it will read indexed data reg 0x04, but it will point to reg 0
     In other modes read 0x04 FIFO register since all registers are aliased*/


    //Lets do it in auto increment mode
    CSLOW();

    //Write Command
    SPIWriteByte(CMD_FAST_READ);

    SPISendAddr(PRAM_READ_FIFO_REG);
    
    //Dummy Byte
    SPIWriteByte(CMD_FAST_READ_DUMMY);

    nBitPosition = (Address & 0x03);
    act_read_bytes = 4 - nBitPosition;
    nlength = Count > act_read_bytes ? act_read_bytes : Count;
    
    
    SPIReadBurstMode ((UINT8*)&u8Buffer[0]);
        
    memcpy(ReadBuffer ,&u8Buffer[nBitPosition], nlength);
    i = nlength;
    Count -= nlength;
    
    while(Count)
    {
    	DELAY_1US();
	
	
        SPIReadBurstMode ((UINT8*)&u8Buffer[0]);
        nlength = Count > 4 ? 4 : Count;
        memcpy((ReadBuffer+i) ,&u8Buffer[0], nlength);

    
        i += nlength;
        Count -= nlength;
    }

    CSHIGH();

    return;
}
        
     
/*******************************************************************************
  Function:
   void SPIWritePDRamRegister(UINT8 *WriteBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function writes the PDRAM using LAN9252 FIFO.        
  
*****************************************************************************/
void SPIWritePDRamRegister(UINT8 const *WriteBuffer, UINT16 Address, UINT16 Count)
{
    UINT16 u16Buffer[4] = {0};
    UINT16 i,nlength, nBytePosition ; //nWrtSpcAvlCount;

    /*Reset or Abort any previous commands.*/
    u16Buffer[1] = PRAM_RW_ABORT_MASK;                                                

    SPIWriteDWord (PRAM_WRITE_CMD_REG, (UINT8*)&u16Buffer[0]);

    /*Make sure there is no previous write is pending
    (PRAM Write Busy) bit is a 0 */
    do
    {
       SPIReadDWord (PRAM_WRITE_CMD_REG, (UINT8*)&u16Buffer[0]);

    }while((u16Buffer[1] & PRAM_RW_BUSY_16B));

    /*Write Address and Length Register (ECAT_PRAM_WR_ADDR_LEN) with the
    starting UINT8 address and length) and write to the EtherCAT Process RAM Write Command Register (ECAT_PRAM_WR_CMD) with the  PRAM Write Busy
    (PRAM_WRITE_BUSY) bit set*/
    u16Buffer[0] = Address;
    u16Buffer[1] = Count;
    u16Buffer[2] = 0x0;
    u16Buffer[3] = 0x8000;
    
   SPIWriteBytes (PRAM_WRITE_ADDR_LEN_REG, (UINT8*)&u16Buffer[0],8);

   /*Read PRAM write Data Available (PRAM_READ_AVAIL) bit is set*/
    do
    {
       SPIReadDWord (PRAM_WRITE_CMD_REG,(UINT8*)&u16Buffer[0]);

    }while(!(u16Buffer[0] & IS_PRAM_SPACE_AVBL_MASK));

    /*Write data to Write FIFO) */ 
    /*get the byte lenth for first read*/
    nBytePosition = (Address & 0x03);
    
    //4-nBytePosition - is the actual data read
    nlength = ((4-nBytePosition) > Count) ? Count:(4-nBytePosition);

    memset(&u16Buffer[0],0x00,4);
    memcpy(&u16Buffer[nBytePosition>>1],WriteBuffer, nlength);

    //Auto increment mode
    CSLOW();

    //Write Command
    SPIWriteByte(CMD_SERIAL_WRITE);

    SPISendAddr(PRAM_WRITE_FIFO_REG);
    
    //Write Bytes
    SPIWriteBurstMode((UINT8*)&u16Buffer[0]);
        
    Count-=nlength;
    i=nlength;

    while(Count)
    {
		DELAY_1US();
        nlength = Count > 4 ? 4 : Count;
        memset(&u16Buffer[0],0x00,4);
        memcpy(&u16Buffer[0], (WriteBuffer+i), nlength);
        
        /*Read PRAM write Data Available (PRAM_READ_AVAIL) bit is set*/
	    SPIWriteBurstMode ((UINT8*)&u16Buffer[0]);
        i+=nlength;
        Count-=nlength;
    }

    CSHIGH();
    return;
}

/*******************************************************************************
  Function:
   void PDIReadReg(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function reads the ESC registers using LAN9252 CSR or FIFO.         
  
*****************************************************************************/
void PDIReadReg(UINT8 *ReadBuffer, UINT16 Address, UINT16 Count)
{
    if (Address >= 0x1000)
    {
        SPIReadPDRamRegister(ReadBuffer, Address,Count);
    }
    else
    {
        SPIReadRegUsingCSR(ReadBuffer, Address, Count);
    }
}

/*******************************************************************************
  Function:
   void PDIWriteReg( UINT8 const *WriteBuffer, UINT16 Address, UINT16 Count)
  Summary:
    This function writes the ESC registers using LAN9252 CSR or FIFO.        
  
*****************************************************************************/
void PDIWriteReg( UINT8 const *WriteBuffer, UINT16 Address, UINT16 Count)
{
   
   if (Address >= 0x1000)
   {
	SPIWritePDRamRegister(WriteBuffer, Address,Count);
   }
   else
   {
	SPIWriteRegUsingCSR(WriteBuffer, Address);
   }
    
}

/*******************************************************************************
  Function:
	UINT32 PDIReadLAN9252DirectReg( UINT16 Address)
  Summary:
    This function reads the LAN9252 CSR registers(Not ESC registers).        
  
*****************************************************************************/
void PDIReadLAN9252DirectReg( UINT16 Address, UINT8 *data)
{   
    SPIReadDWord (Address, data);
 }

/*******************************************************************************
  Function:
	void PDIWriteLAN9252DirectReg( UINT32 Val, UINT16 Address)
  Summary:
    This function writes the LAN9252 CSR registers(Not ESC registers).        
  
*****************************************************************************/
void PDIWriteLAN9252DirectReg( UINT32 Val, UINT16 Address)
{
    SPIWriteDWord (Address, (UINT8*)&Val);
}

/*******************************************************************************
  Function:
	void PDI_Init()
  Summary:
    This function initialize the PDI(SPI).        
  
*****************************************************************************/
void PDI_Init()
{
    SPIOpen();
    //This is used for delay in read and write functions
    Init_DELAY_1US();
}

