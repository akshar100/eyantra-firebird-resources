/**************************************************************************************************************
		Platform: Fire Bird V P89V51RD2
		LCD Interfacing: String Display
		Written by: Omkar Pradhan, NEX Robotics Pvt. Ltd.
		Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
		Last Modification: 2009-12-08
		This program initializes the LCD and displays text onto the screen
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
#include "p89v51rx2.h"

sbit buzzer=P2^7;  //buzzer = 1; buzzer off, buzzer = 0; buzzer on,

//program specific declarations
sbit RS = P2^6;    //P2^6 is connected to the RS line of LCD 
sbit RW = P2^5;    //P2^5 is connected to the RW line of LCD
sbit E = P2^4;	   //P2^4 is connected to the EN line of LCD
sbit BUSY = P2^3;  //P2^3 is connected to the DB3 pin of LCD

unsigned char A=0; //used in the swap(unsigned char tempdata) function

// function for giving a delay of ms milliseconds
void delay_ms(unsigned int ms)
{
unsigned int i,j;

for(i=0;i<ms;i++)
for(j=0;j<53;j++);
}

// subroutines required to check if LCD has finished executing previous command and is ready to accept next one
 void ready(void) 
 {
  bit readybit;
  unsigned char buzzer_status = 0;
  buzzer_status = P2 & 0x80; //storing original buzzer status
    
  E = 0;
  RS = 0;			 //  to select comand register
  RW = 1;			 // to select read mode

  while(readybit == 1)	 // keep checking MSB bit till busy goes low
  {
   E = 1;		 // high to low transition on E pin requird to clock in data
   P2 = 0x7F | buzzer_status;		 // to set this pin in read mode to read busy signal from MSB (D7) of the LCD
   readybit = P2^3;
   E = 0;
  }
 }

/******************************************************************************************************
	Function: swap(unsigned char tempdata)
	
	Description: This function swaps the data which is passed to it 
				 in such away that higher nibble is pushed in place of
				 lower nibble because lower nibble of microcontroller is being used for data transfer.
*******************************************************************************************************/

unsigned char swap(unsigned char tempdata)
{
A = tempdata;
A = _cror_(A,4);	// this uses an inbuilt rotate function to get MSB's in LSB's
tempdata = A & 0x0F;
return tempdata;
}	

/****************************************************************
	Function: commandsend(unsigned char command)
  
    Description: This function sends data to LCD as command by
				 selecting the instruction register
****************************************************************/

void commandsend(unsigned char command)
{ 
unsigned char temp;
unsigned char buzzer_status = 0;
  
buzzer_status = P2 & 0x80; //storing original buzzer status

ready();       // checking for busy bit to send next nibble
temp = swap(command);
P2 = temp | buzzer_status; // to get higher nibble
RW = 0;        // to select write mode
RS = 0;        // to select command register
E = 1;         // to get the enable high to low transition
E = 0;         // in order to gate in the data

P2 = (command & 0x0F) | buzzer_status; // to get the lower nibble
RW = 0;
RS = 0;        // repeat again all above steps    

E = 1;
E = 0;
delay_ms(2);  //delay of 2 ms
}

/***********************************************************
	Function: datasend(unsigned char command)
  
    Description: This function sends data to LCD as data by
				 selecting the data register
***********************************************************/

void datasend(unsigned char lcddata)
{
unsigned char temp;
unsigned char buzzer_status = 0;

buzzer_status = P2 & 0x80; //storing original buzzer status

temp = swap(lcddata);
ready();	// checking for busy bit to send next nibble
P2 = temp | buzzer_status;  // to get higher nibble
RW = 0;	 // to select write mode
RS = 1;  // to select data register
         
E = 1;
E = 0;


P2 = (lcddata & 0x0F)  | buzzer_status;
RW = 0;
RS = 1;
E = 1;
E = 0;
delay_ms(2);	 //delay of 2 ms
}

/***********************************************************************
 Function: lcd_init(void) 
 Description: This function initialises LCD display in 4-bit mode.
              The intialisation functions are 8-bit wide, so we write
			  each inst into two seperate nibbles.
			  Following instructions are used for Lcd init.
			  1.>  0x30 8-bit mode
			  2.>  0x28 4-bit mode and 5x8 dot character font
 			  3.>  0x0E Turn ON lcd and cursor
			  4.>  0x06 Autoincrement cursor position
			  5.>  0x01 Clear Lcd display	
			  6.>  0x80 for setting cursor position
************************************************************************/

void lcd_init(void)
{						
unsigned char buzzer_status = 0;

buzzer_status = P2 & 0x80; //storing original buzzer status


RS = 0;	 //  to select comand register
RW = 0;	 // to select write mode
delay_ms(40);  // on power ON we must allow a delay of 40ms for VCC to settle
 
E = 0;
P2 = 0x03 | buzzer_status; // send instruction to set FUNCTION SET to 8 bit mode though we have only connected 4 bit data bus because by default on power ON LCD is in 8 bit mode
E =  1;
E = 0;

delay_ms(4);  // delay of 4ms
  
P2 = 0x03 | buzzer_status;	  // send the same instruction two times again. This way the LCD controller knows that we are operating in 4 bit mode
E =  1;
E = 0;
delay_ms(2);	   // delay of  2ms
P2 = 0x03  | buzzer_status;
E =  1;
E = 0;
delay_ms(2); // delay of  2ms

P2 = 0x02  | buzzer_status;		 // send FUNCTION SET instruction for 4 bit mode
E =  1;
E = 0;
ready();
P2 = 0x02  | buzzer_status; // again we send FUNCTION SET higher bits to keep 4 bit mode operation but main aim is to set the parameters in lower bits

E =  1;
E = 0;
P2 = 0x08  | buzzer_status; // these are the lower bits of FUNCTION SET. They set LCD to 1 display line and set font size to 5 x 8 dots

E =  1;
E = 0;
ready();
P2 = 0x00  | buzzer_status; // higher bits of DISPLAY OFF instruction
E =  1;
E = 0;
P2 = 0x08  | buzzer_status;  // lower bits of DISPLAY OFF instruction
E =  1;
E = 0;
ready();
P2 = 0x00  | buzzer_status;  // higher bits of CLEAR instruction
E =  1;
E = 0;
P2 = 0x01  | buzzer_status; // lower bits of CLEAR instruction
E =  1;
E = 0;
ready();
P2 = 0x00  | buzzer_status;// higher bits of ENTRY MODE instruction
E =  1;
E = 0;

P2 = 0x06  | buzzer_status;// lower bits of ENTRY MODE instruction to set increment i.e display to right side mode and no shift
E =  1;
E = 0;
ready();

commandsend(0x0F); // set LCD ON; CURSOR OFF; BLINK ON.
}

void lcd_display(void)
{
int i=0;
unsigned char lcd_data1[16]={"  NEX Robotics  "};  // While defining the character array make sure that it is less than 16 characters(for each line of LCD) & long enough to hold all characters in the string 
unsigned char lcd_data2[16]={"ERTSLab CSE IITB"};

commandsend(0x01); // clear display.
commandsend(0x06);  // auto increment cursor position

for(i=0;i<16;i++)
{
 datasend(lcd_data1[i]);
}

commandsend(0xC0);         //80h+40h. 80h for setting cursor position
					      //40h is the address for the first character on thesecon line

for(i=0;i<16;i++)
{
 datasend(lcd_data2[i]);
}
}


void main (void)
{
lcd_init();
buzzer = 1; //buzzer off

while(1)
{
 lcd_display();
 delay_ms(1000);
}

}