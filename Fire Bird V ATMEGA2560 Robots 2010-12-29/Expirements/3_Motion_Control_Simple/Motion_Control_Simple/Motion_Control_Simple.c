/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates simple motion control.

 Concepts covered: Simple motion control

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B of timer 5.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:  	L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1; 


 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
    Frequency: 14745600
 	Optimization: -Os (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current 
	surge which can reset the microcontroller because of sudden fall in voltage. 
	It is a good practice to stop the motors for at least 0.5seconds before changing 
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.
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


void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Function to initialize ports
void port_init()
{
 motion_pin_config();
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}


void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void) //hard stop
{
  motion_set(0x00);
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 sei(); //Enables the global interrupts
}


//Main Function
int main()
{
	init_devices();
		
	while(1)
	{
	
		forward(); //both wheels forward
		_delay_ms(1000);

		stop();						
		_delay_ms(500);
	
		back(); //bpth wheels backward						
		_delay_ms(1000);

		stop();						
		_delay_ms(500);
		
		left(); //Left wheel backward, Right wheel forward
		_delay_ms(1000);
		
		stop();						
		_delay_ms(500);
		
		right(); //Left wheel forward, Right wheel backward
		_delay_ms(1000);

		stop();						
		_delay_ms(500);

		soft_left(); //Left wheel stationary, Right wheel forward
		_delay_ms(1000);
		
		stop();						
		_delay_ms(500);

		soft_right(); //Left wheel forward, Right wheel is stationary
		_delay_ms(1000);

		stop();						
		_delay_ms(500);

		soft_left_2(); //Left wheel backward, right wheel stationary
		_delay_ms(1000);

		stop();						
		_delay_ms(500);

		soft_right_2(); //Left wheel stationary, Right wheel backward
		_delay_ms(1000);

		stop();						
		_delay_ms(1000);
	}
}

