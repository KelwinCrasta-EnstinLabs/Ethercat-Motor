----------------------------------------------------------------
Uart_output0x7000:
->Uart_write_buffer

*** Provide the data to written in Uart_output0x7000.Uart_write_buffer
*** from twincat master
-----------------------------------------------------------------


-----------------------------------------------------------------
Configure_uart0x8000:
-> Buadrate						
-> ParityAndDataSelectionBits		
-> EnableUart
-> Tx_ready					
-> StopSelectionBit

*** Write the new baudrate to set in Configure_uart0x8000.Buadrate 
*** from twincat master

*** Set Configure_uart0x8000.EnableUart bit high from 
*** twincat master to enable UART

*** If Configure_uart0x8000.Tx_ready is set high from Twincat master, 
*** data in Uart_output0x7000.Uart_write_buffer will be written to UART

*** Set the new Configure_uart0x8000.ParityAndDataSelectionBits value
*** to be changed in twincat master
*** 3 = 9-bit data, no parity
*** 2 = 8-bit data, odd parity
*** 1 = 8-bit data, even parity
*** 0 = 8-bit data, no parity

*** Set the new Configure_uart0x8000.ParityAndDataSelectionBits value
*** from twincat master
*** 1 = 2 Stop bits
*** 0 = 1 Stop bit
-----------------------------------------------------------------


-----------------------------------------------------------------
Uart_input0x6000:
-> Uart_read_buffer

*** Uart_input0x6000.Uart_read_buffer contains the data read through UART
-----------------------------------------------------------------


-----------------------------------------------------------------
Uart_status0x6021:
-> Rx_ready  

*** If UART data is wriiten to dsPIC, 
*** the data will be received in Uart_input0x6000.Uart_read_buffer and Rx_ready will be set high
-----------------------------------------------------------------


-----------------------------------------------------------------
Motor_ctrl_output0x7011:
-> Speed
-> Start
-> Direction

*** Set Motor_ctrl_output0x7011.Speed value from twincat master

*** Set Motor_ctrl_output0x7011.Start bit from twincat master

*** Set Motor_ctrl_output0x7011.Direction (0 - clockwise, 1- anti-clockwise)
*** from twincat master
-----------------------------------------------------------------


-----------------------------------------------------------------
Motor_ctrl_status0x6030:
-> Motor_state
-> Speed
-> Direction


*** Motor_ctrl_status0x6030.Motor_state provides the current state of the motor 
*** which can be one among MC_STOPPED, MC_STARTING, MC_RUNNING, MC_STOPPING and MC_FAULT.

*** Motor_ctrl_status0x6030.Speed provides the current speed of the motor

*** Motor_ctrl_status0x6030.Direction provides current direction of the motor
*** (0 - clockwise, 1- anti-clockwise)
-----------------------------------------------------------------



-----------------------------------------------------------------
Steps to integrate uart-motor-control-application in LAN9252-dsPIC33SDK
-----------------------------------------------------------------

1. Download the LAN9252-dsPIC33 SDK from the AE-LAN9252-EtherCATP board’s product page.

2. Copy \SSC\dsPIC33EP.X\UARTDriver folder and \SSC\dsPIC33EP.X\MotorDriver folder from AE-LAN9252-dsPIC-UART-Motor-Control-Example to \SSC\dsPIC33EP.X folder of AE-LAN9252-dsPIC-SDK-V1.0.

3. Replace nbproject folder, main.c, Makefile, user.c and user.h of AE-LAN9252-dsPIC-SDK-V1.0 with those of AE-LAN9252-dsPIC-UART-Motor-Control-Example.

4. Replace Sample application folder of AE-LAN9252-dsPIC-SDK-V1.0 with that of AE-LAN9252-dsPIC-UART-Motor-Control-Example.

5. In AE-LAN9252-dsPIC-SDK-V1.0, create source file using Sample application\DSPIC_Sample.xlsx file, Microchip_LAN9252_dsPIC33_SSC_Config.xml, and SSC\Common\9252_HW.c files in SSC tool. Ensure that they are copied to SSC\Common folder.

6. In AE-LAN9252-dsPIC-SDK-V1.0, replace DSPIC_Sample.c, DSPIC_Sample.h and DSPIC_SampleObjects.h files under SSC\Common folder with those present under Sample application folder.

7. Now, compile the AE-LAN9252-dsPIC-SDK-V1.0 project in MPLAB X IDE. 