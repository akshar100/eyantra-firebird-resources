/********************************************************************************
 Platform: SPARK V
 Experiment: 3_Timer1_Overflow_Interrupt
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 22nd September 2010
 AVR Studio Version 4.17, Build 666
 
 Concepts covered: Use of timer overflow interrupt to do tasks in a peirodic way

 This experiment demonstrates use of timer overflow interrupt

 In this example timer 1's overflow interrupt is used to turn on and off Buzzer 
 with the time peirod of 1 second. The Buzzer is connected at PORTC3
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800
 	Optimization: -O0 (For more information read section: Selecting proper 
	              optimization options below figure 4.22 in the hardware manual)

 2. TIMSK = 0x04; //timer1 overflow interrupt enable must be done in init devices. 
    It should not be done in timer1_init()

*********************************************************************************/

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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char toggel = 0;   //used as a variable for buzzer state toggel action

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

//TIMER1 initialize - prescale:1024
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 1Hz
// actual value:  1.000Hz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xE3; //setup
 TCNT1L = 0xE1;
 OCR1AH = 0x1C;
 OCR1AL = 0x1F;
 OCR1BH = 0x1C;
 OCR1BL = 0x1F;
 ICR1H  = 0x1C;
 ICR1L  = 0x1F;
 TCCR1A = 0x00;
 TCCR1B = 0x05; //start Timer
}

//This ISR can be used to toggle the buzzer input
ISR(TIMER1_OVF_vect)
{
 //TIMER4 has overflowed
 TCNT1H = 0xE3;   //reload counter high value
 TCNT1L = 0xE1;   //reload counter low value

 toggel = ~toggel;

 if (toggel == 0)
 {
  buzzer_on();
 }
 else
 {
  buzzer_off();
 }
} 

void init_devices(void)
{
 cli();          //Clears the global interrupts
 port_init();    //Initializes all ports 
 timer1_init();
 TIMSK = 0x04;   //timer1 interrupt sources
 sei();          //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	while(1);
}


