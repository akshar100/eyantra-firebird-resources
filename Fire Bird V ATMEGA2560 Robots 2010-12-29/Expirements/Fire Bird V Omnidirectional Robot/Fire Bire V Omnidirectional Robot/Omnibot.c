/********************************************************************************
 Written by: Sachitanand Malewar NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates omnidirectional robot motion control.

 Concepts covered: Omnidirectional robot motion control

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA5
 2. Velocity control by PWM on pins PL3, PL4 and PL5 using OC5A, OC5B and OC5C of timer 5 and 3.

 Connection Details:  	A-1---->PA0;		A-2---->PA1;
   						B-1---->PA2;		B-2---->PA3;
						C2-1--->PA6;		C2-2--->PA7;
   						PL3 (OC5A) ----> PWM MOTOR A;
   						PL4 (OC5B) ----> PWM MOTOR B;
   						PE3 (OC3A) ----> PWM MOTOR C;
						 	

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -Os (For more information read section: Selecting proper optimization options 
						below figure 2.20 in the Software Manual)

 2. Auxiliary power can supply current up to 1 Ampere while Battery can supply current 
 	up to 2 Ampere. When both motors of the robot changes direction suddenly without stopping,
	it produces large current surge. When robot is powered by Auxiliary power which can supply 
	only 1 Ampere of current, sudden direction change in both the motors will cause current surge 
	which can reset the microcontroller because of sudden fall in voltage. It is a good practice 
	to stop the motors for at least 0.5seconds before changing the direction. This will also 
	increase the useable time of the fully charged battery.

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


//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0xCF;   //Motion control pins set as output
 PORTA = PORTA & 0x30; //Inital value of the motion control pins set to 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //Setting PL3 and PL4 pins as logic 1
 DDRE = DDRE | 0x08;   //Setting PE3 pin as output for PWM generation
 PORTE = PORTE | 0x08; //Setting PE3 pin as logic 1
}

//Function to initialize ports
void init_ports()
{
 motion_pin_config();
}

// Timer 3 initialized in PWM mode for velocity control
// Prescale:64
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:900Hz
void timer3_init()
{
	TCCR3B = 0x00;	//Stop
	TCNT3H = 0xFF;	//Counter higher 8-bit value to which OCR3xH value is compared with
	TCNT3L = 0x01;	//Counter lower 8-bit value to which OCR3xH value is compared with
	OCR3AH = 0x00;	//Output compare register high value for C2 motor
	OCR3AL = 0xFF;	//Output compare register low value for C2 motor
	OCR3BH = 0x00;	//Not used
	OCR3BL = 0xFF;	//Not used
	OCR3CH = 0x00;	//Not used
	OCR3CL = 0xFF;	//Not used
	TCCR3A = 0x81;	/*{COM3A1=1, COM3A0=0; COM3B1=0, COM3B0=0; COM3C1=0 COM3C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM31=0, WGM30=1} Along With WGM32 in TCCR3B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR3B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:64
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:674.988Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}



//Function for robot velocity control
void velocity (unsigned char A, unsigned char B, unsigned char C)
{
	OCR5AL = (unsigned char)A;
	OCR5BL = (unsigned char)B;
	OCR3AL = (unsigned char)C;
}

//Function used for setting motor's direction
void motion_set (unsigned char ucDirection)
{
 unsigned char ucPortARestore = 0;

 ucDirection &= 0xCF; // removing upper 5th and 4th bits for the protection
 ucPortARestore = PORTA; // reading the PORTA original status
 ucPortARestore &= 0x30; // making all the bits 0 except 5th and 4th bit
 ucPortARestore |= ucDirection; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = ucPortARestore; // executing the command
}

void forward(void) 
{
  motion_set(0x06);
}

void back(void)  
{
  motion_set(0x09);  
}

void left(void)  
{
  motion_set(0x85);
}

void right(void)
{
  motion_set(0x4A);
}

void rotate_left (void)
{
	motion_set(0x45);
}

void rotate_right (void)
{
	motion_set(0x8A);
}

void stop(void)
{
  motion_set(0xFF);
}

void init_devices (void) //use this function to initialize all devices
{
 cli(); //disable all interrupts
 init_ports();
 timer3_init();
 timer5_init();
 sei(); //re-enable interrupts
}

//Main Function
int main()
{
	init_devices();
	
	while(1)
	{	
		forward();				
		velocity(255,255,0);
		_delay_ms(1500);

		stop();
		_delay_ms(1000);
		
		back();					
		velocity(255,255,0);
		_delay_ms(1500);

		stop();
		_delay_ms(1000);
		
		
		left();
		velocity(120,150,255);
		_delay_ms(1500);

		
		stop();
		_delay_ms(1000);
		
		right();
		velocity(150,120,255);
		_delay_ms(1500);

		stop();
		_delay_ms(1000);
		
		rotate_left();
		velocity(255,255,255);
		_delay_ms(1500);

		stop();
		_delay_ms(1000);
	
		rotate_right();
		velocity(255,255,255);
		_delay_ms(1500);

		stop();
		_delay_ms(1000);

	}
}
