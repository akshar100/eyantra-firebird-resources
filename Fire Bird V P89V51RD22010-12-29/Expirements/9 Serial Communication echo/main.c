/**************************************************************************************************************
		Platform: Fire Bird V P89V51RD2
		Serial Communication echo
		Written by: Omkar Pradhan, NEX Robotics Pvt. Ltd.
		Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
		Last Modification: 2009-12-08
		This Program echos back a character sent from the PC through serial communication
		Compiled with: uVision3 V3.90; C Compiler: C51.Exe, V8.18
**************************************************************************************************************/	

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <intrins.h>
#include "p89v51rx2.H"

sbit buzzer = P2^7;

/**************************************************************************
	Function: uart_setup()
	
	Description: This function configures Timer1 for generating the Baudrate.
			     The TH1 register should be written with following hex numbers
		         for desired baud rate. After setting the baudrate, SCON register
			     for enabling Rx and setting up the frame format.
 	
	       		 9600 ----->  0xFD
			     4800 ----->  0xFA
			     2400 ----->  0xF4
			     1200 ----->  0xE8

**************************************************************************/
void uart_setup()
{
    TMOD = 0x20;		// configure timer1 for Mode 2 operation for the correct baud rate 
    TH1 = 0xFD;     	// 9600 bps for 11.0592 MHz clock 
    TCON = 0x40;    	// Start timer 1 by setting TR1 = 1 
    SCON = 0x50;  		// Set Serial IO to receive and normal mode 
	RI=0;               //Receive Interrupt Flag is cleared
	TI=0;  				//Tx interrupt flag is cleared
}		                 

/***************************************************************************
	Function: void char RxData()
	
	Description: This function copies any data that is present in the SBUF 
			     register and returns it to the calling function.
***************************************************************************/

 unsigned char RxData()
{
    unsigned char rcv_data;
	rcv_data = SBUF;              //if any data is available copy it from SBUF
	return rcv_data;              //return the received data
}

/***************************************************************************
	Function: void TxData(unsigned char tx_data)
	
	Description: This function transmits  any data that is passed to it
	             and waits until it is transmitted.
***************************************************************************/

void TxData(unsigned char tx_data)
{
 	
	SBUF = tx_data;                 //Transmit data that is passed to this function
	while(TI == 0);					//wait while data is being transmitted
}

void main(void)
{
 unsigned char rx_data,tx_data;
 uart_setup(); // calling the UART setup function
 while(1)					//loop continuously
 {
  if (RI==1)                // if any data is present store it and transmit it back 
  {
   rx_data = RxData();      //get the received data
   tx_data = rx_data;
   TxData(tx_data);         // transmit it back to the PC. This data can be viewed onto the 'terminal'
   RI = 0;                  //Clear receive interrupt. Must be cleared by the user.
   TI = 0;					//Clear transmit interrupt. Must be cleared by the user.
  }
 }
}

