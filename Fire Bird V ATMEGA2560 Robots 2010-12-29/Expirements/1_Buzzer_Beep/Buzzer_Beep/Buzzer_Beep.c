/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates the simple operation of Buzzer ON/OFF with
 one second delay. Buzzer is connected to PORTC.3 of ATMEGAM2560

 Concepts covered:
 Output operation, generating exact delay

 Note: Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Microcontroller: atmega2560
 Frequency: 14745600
 Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)
*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.			-*- c -*-
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void port_init (void)
{
 buzzer_pin_config();
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 sei(); //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	while(1)
	{
		buzzer_on();
		_delay_ms(1000);		//delay
		buzzer_off();
		_delay_ms(1000);		//delay
	}
}


