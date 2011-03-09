/**************************************************************************************************************
		Platform: Fire Bird V P89V51RD2
		LCD Interfacing: Data Array Display
		Written by: Omkar Pradhan, NEX Robotics Pvt. Ltd.
		Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
		Last Modification: 2009-12-08
		This program displays an 8 element array onto the LCD screen by using a .h file "lcd_array_disp.h"
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
#include "lcd_array_disp.h"

sbit buzzer=P2^7;  //buzzer = 1; buzzer off, buzzer = 0; buzzer on,

unsigned char data_array[8];//initialize an array of 8 elements to display onto the LCD screen
unsigned char i=0; 

void main()
{
 for(i=0;i<8;i++) // used to fill data to be displayed in the arry. You can put any value in the array.
 data_array[i] = i; 

 i = 0; //reset i to 0 for data_array [0] increment display on the LCD
 
 buzzer = 1; // buzzer off
 lcd_init(); //Initialize LCD

 while(1)
 {
  lcdprint(data_array);//call this function to print the array onto the screen
  delay_ms(1000);
  i++;
  data_array [0] = i;
 }
}