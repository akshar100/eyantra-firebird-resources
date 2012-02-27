/********************************************************************************
 Platform: SPARK V
 Experiment: 4_Motion_Control_Simple
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 22nd September 2010
 AVR Studio Version 4.17, Build 666

 Concepts covered: External Interrupts, Position control

 This experiment demonstrates use of position encoders.
 
 Microcontroller pins used:
 PORTB3 to PORTB0: Robot direction control
 PD4, PD5: Robot velocity control. Currently set to 1 as PWM is not used
 PD3 (INT1): External interrupt for left motor position encoder 
 PD2 (INT0): External interrupt for the right position encoder

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

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

//Function to configure INT1 (PORTD 3) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xF7;  //Set the direction of the PORTD 3 pin as input
 PORTD = PORTD | 0x08; //Enable internal pull-up for PORTD 3 pin
}

//Function to configure INT0 (PORTD 2) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xFB;  //Set the direction of the PORTD 2 pin as input
 PORTD = PORTD | 0x04; //Enable internal pull-up for PORTD 2 pin
}

//Function to initialize ports
void port_init()
{
 motion_pin_config();          //robot motion pins config
 left_encoder_pin_config();    //left encoder pin config
 right_encoder_pin_config();   //right encoder pin config	
}

void left_position_encoder_interrupt_init (void) //Interrupt 1 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x08; // INT1 is set to trigger with falling edge
 GICR = GICR | 0x80;   // Enable Interrupt INT1 for left position encoder
 sei(); // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 0 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x02; // INT0 is set to trigger with falling edge
 GICR = GICR | 0x40;   // Enable Interrupt INT5 for right position encoder
 sei(); // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT0_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT1_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortBRestore = PORTB; 		// reading the PORT original status
 PortBRestore &= 0xF0; 		// making lower direction nibbel to 0
 PortBRestore |= Direction; // adding lower nibbel for forward command and restoring the PORTB status
 PORTB = PortBRestore; 		// executing the command
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

void stop (void)
{
  motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 12.85; // division by resolution to get shaft count 
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 12.92; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

//Function to initialize all the devices
void init_devices()
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei();   // Enables the global interrupt 
}


//Main Function

int main(void)
{
	init_devices();

	while(1)
	{
		forward_mm(100);          //Moves robot forward 100mm
		stop();
		_delay_ms(500);			

		back_mm(100);             //Moves robot backward 100mm
	    stop();			
		_delay_ms(500);
		
		left_degrees(90);         //Rotate robot left by 90 degrees
		stop();
		_delay_ms(500);
		
		right_degrees(90);        //Rotate robot right by 90 degrees
		stop();
		_delay_ms(500);
		
		soft_left_degrees(90);    //Rotate (soft turn) by 90 degrees
		stop();
		_delay_ms(500);
		
		soft_right_degrees(90);	  //Rotate (soft turn) by 90 degrees
		stop();
		_delay_ms(500);

		soft_left_2_degrees(90);  //Rotate (soft turn) by 90 degrees
		stop();
		_delay_ms(500);
		
		soft_right_2_degrees(90); //Rotate (soft turn) by 90 degrees
		stop();
		_delay_ms(500);
	}
}
