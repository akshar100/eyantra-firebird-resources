/********************************************************************************
 Platform: SPARK V
 Experiment: 4_Motion_Control_Simple
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 22nd September 2010
 AVR Studio Version 4.17, Build 666

 Concepts covered: I/O interfacing, motion control using L293D 

 This experiment demonstrates simple motion control using L293D Motor driver IC.

 There are two components to the motion control:
 1. Direction control using pins PORTB0 to 	PORTB3
 2. Velocity control using Pulse Width Modulation(PWM) on pins PD4 and PD5.

 Pulse width modulation is a process in which duty cycle of constant frequency square wave is modulated to 
 control power delivered to the load i.e. motor. 

 In this experiment for the simplicity PWM pins, PD4 and PD5 are kept at logic 1.
 Use of PWM will be covered in the "5_Velocity_Control_Using_PWM" experiment.
   
 Connection Details:  	L-1---->PB0;		L-2---->PB1;
   						R-1---->PB2;		R-2---->PB3;
   						PD4 (OC1B) ----> Logic 1; 	PD5 (OC1A) ----> Logic 1; 

 Make sure that motors are connected to onboard motor connectors.
 
 For more detail refer to hardware and software manual

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800Hz
 	Optimization: -O0 (For more information read section: Selecting proper 
	              optimization options below figure 4.22 in the hardware manual)

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
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

//Function to initialize ports
void port_init()
{
 motion_pin_config();
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
  motion_set(0x06);
}

void back (void)            //both wheels backward
{
  motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
{   
  motion_set(0x0A);
}

void soft_left (void)       //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void)      //Left wheel forward, Right wheel is stationary
{ 
 motion_set(0x02);
}

void soft_left_2 (void)     //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void)    //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void hard_stop (void)       //hard stop(stop suddenly)
{
  motion_set(0x00);
}

void soft_stop (void)       //soft stop(stops solowly)
{
  motion_set(0x0F);
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
		forward();            //both wheels forward
		_delay_ms(1000);

		hard_stop();						
		_delay_ms(300);

		back();               //both wheels backward						
		_delay_ms(1000);

		hard_stop();						
		_delay_ms(300);
	
		left();               //Left wheel backward, Right wheel forward
		_delay_ms(1000);
	
		hard_stop();						
		_delay_ms(300);
	
		right();              //Left wheel forward, Right wheel backward
		_delay_ms(1000);

		hard_stop();						
		_delay_ms(300);

		soft_left();          //Left wheel stationary, Right wheel forward
		_delay_ms(1000);
	
		hard_stop();						
		_delay_ms(300);

		soft_right();         //Left wheel forward, Right wheel is stationary
		_delay_ms(1000);

		hard_stop();						
		_delay_ms(300);

		soft_left_2();        //Left wheel backward, right wheel stationary
		_delay_ms(1000);

		hard_stop();						
		_delay_ms(300);

		soft_right_2();       //Left wheel stationary, Right wheel backward
		_delay_ms(1000);

		hard_stop();						
		_delay_ms(300);
	}
}

