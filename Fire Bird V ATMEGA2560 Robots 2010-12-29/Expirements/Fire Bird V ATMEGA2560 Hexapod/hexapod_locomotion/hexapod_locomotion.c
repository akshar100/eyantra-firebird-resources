/********************************************************************************
 Platform: Fire Bird V ATMEGA2560 Hexapod
 Experiment: Fire Bird V Hexapod Remote control application for the Robot
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 28th Dec 2010
 AVR Studio Version 4.17, Build 666
 
 In this application the hexapod is moved in Forward ,backward and rotated 
 counterclockwise and clockwise

 Servo motor interfacing:
 Fire Bird V Hexapod has six legs. Each leg have three motors. Fire Bird V Hexapod
 uses High torque NRS-995 metal gear servo motors. We need to control 18 servo motors
 simultaneously mounted on the Hexapod. Each servo motor needs control signal at 40 
 to 50Hz repetition rate with 0.5ms to 2.2ms pulse width depending upon the desired
 position.   
 
 To avoid confusion while addressing motors certain naming convention is used. Starting
 from the top left each leg is named from 1 to 6. Three motors on the each legs are 
 named as A,B or C, i.e. each motor have the unique name as 1A or 3C etc. For more 
 information, read the Fire Bird V Hexapod Manual.
 
	Pin name	Port	 ANDing (make 0)	     ORing (make 1)
	-------		----	----------------		----------------

	SP LEFT	    PL1		1111 1101	0xFD		0000 0010	0x02   ( not used )
	1A		    PD5		1101 1111	0xDF		0010 0000	0x20
	1B			PL2		1111 1011	0xFB		0000 0100	0x04
	1C			PG1		1111 1101	0xFD		0000 0010	0x02
	2A			PL6		1011 1111	0xBF		0100 0000	0x40
	2B			PD4		1110 1111	0xEF		0001 0000	0x10
	2C			PL7		0111 1111	0x7F		1000 0000	0x80
	3A			PH6		1011 1111	0xBF		0100 0000	0x40
	3B			PH4		1110 1111	0xEF		0001 0000	0x10
	3C			PH5		1101 1111	0xDF		0010 0000	0x20

	SP RIGHT	PJ7		0111 1111	0x7F		1000 0000	0x80  ( not used )   
	4A			PG0		1111 1110	0xFE		0000 0001	0x01
	4B			PD7		0111 1111	0x7F		1000 0000	0x80
	4C			PD6		1011 1111	0xBF		0100 0000	0x40
	5A			PJ5		1101 1111	0xDF		0010 0000	0x20
	5B			PJ6		1011 1111	0xBF		0100 0000	0x40
	5C			PJ4		1110 1111	0xEF		0001 0000	0x10
	6A			PJ3		1111 0111	0XF7		0000 1000	0x08
	6B			PJ1		1111 1101	0xFD		0000 0010	0x02
	6C			PJ2		1111 1011	0xFB		0000 0100	0x04

    relay i/p   PD0		1111 1110	0xFE		0000 0001	0x01  (Servo on/off)   

 ----------------------------------------------------------------------------------------------- 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: O0 

*************************************************************************************************/

/*************************************************************************************************

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


*********************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char arm_number = 0;                       // selecting the individual arm of robot		
unsigned char angle_upper_byte = 0; 				// temprory storing upper byte of desired angle value 
unsigned char angle_lower_byte = 0; 				// temprory storing lower byte of desired angle value 
unsigned char degree = 0;                           
// final storing of temprory variable for individual servo as upper & lower bytes
unsigned char angle_upper_byte_1A, angle_lower_byte_1A, angle_upper_byte_1B, angle_lower_byte_1B, angle_upper_byte_1C, angle_lower_byte_1C;
unsigned char angle_upper_byte_2A, angle_lower_byte_2A, angle_upper_byte_2B, angle_lower_byte_2B, angle_upper_byte_2C, angle_lower_byte_2C;
unsigned char angle_upper_byte_3A, angle_lower_byte_3A, angle_upper_byte_3B, angle_lower_byte_3B, angle_upper_byte_3C, angle_lower_byte_3C;
unsigned char angle_upper_byte_4A, angle_lower_byte_4A, angle_upper_byte_4B, angle_lower_byte_4B, angle_upper_byte_4C, angle_lower_byte_4C;
unsigned char angle_upper_byte_5A, angle_lower_byte_5A, angle_upper_byte_5B, angle_lower_byte_5B, angle_upper_byte_5C, angle_lower_byte_5C;
unsigned char angle_upper_byte_6A, angle_lower_byte_6A, angle_upper_byte_6B, angle_lower_byte_6B, angle_upper_byte_6C, angle_lower_byte_6C;

unsigned char motion_busy = 0; //sets to one if any of the motion function is called.

//--------------------------------------------------------------------------------
//port initialisation for Servos i/p's												
//--------------------------------------------------------------------------------																			
void servo_pin_config (void)
{ 
 PORTB = 0x00;
 DDRB  = 0x01;
 DDRD  = 0xF1;    //PD 4,5,6,7 as output
 PORTD = 0x00;
 DDRG  = 0x03;    //PG 0,1 as output
 PORTG = 0x00;
 DDRH  = 0x70;    //PH 4,5,6 as output
 PORTH = 0x00;
 DDRJ  = 0xFE;    //PJ 1,2,3,4,5,6,7 as output
 PORTJ = 0x00;
 DDRL  = 0xC7;    //PL 1,2,3,6,7 as output
 PORTL = 0x00;
}

//--------------------------------------------------------------------------------
// funcion for all servos on and servos off (relay on/off)
//--------------------------------------------------------------------------------
void servo_on (void)                    
{ PORTD = PORTD | 0x01; }

void servo_off (void)                    
{ PORTD = PORTD & 0xFE; }

//--------------------------------------------------------------------------------
// reset & set function of joint 1A,1B..............6C 
//--------------------------------------------------------------------------------
void reset_1A (void)
{ PORTD = PORTD & 0xDF; }

void set_1A (void)
{ PORTD = PORTD | 0x20; }

void reset_1B (void)
{ PORTL = PORTL & 0xFB; }

void set_1B (void)
{ PORTL = PORTL | 0x04; }

void reset_1C (void)
{ PORTG = PORTG & 0xFD; }

void set_1C (void)
{ PORTG = PORTG | 0x02; }

void reset_2A (void)
{ PORTL = PORTL & 0xBF; }

void set_2A (void)
{ PORTL = PORTL | 0x40; }

void reset_2B (void)
{ PORTD = PORTD & 0xEF; }

void set_2B (void)
{ PORTD = PORTD | 0x10; }

void reset_2C (void)
{ PORTL = PORTL & 0x7F; }

void set_2C (void)
{ PORTL = PORTL | 0x80; }

void reset_3A (void)
{ PORTH = PORTH & 0xBF; }

void set_3A (void)
{ PORTH = PORTH | 0x40; }

void reset_3B (void)
{ PORTH = PORTH & 0xEF; }

void set_3B (void)
{ PORTH = PORTH | 0x10; }

void reset_3C (void)
{ PORTH = PORTH & 0xDF; }

void set_3C (void)
{ PORTH = PORTH | 0x20; }

void reset_4A (void)
{ PORTG = PORTG & 0xFE; }

void set_4A (void)
{ PORTG = PORTG | 0x01; }

void reset_4B (void)
{ PORTD = PORTD & 0x7F; }

void set_4B (void)
{ PORTD = PORTD | 0x80; }

void reset_4C (void)
{ PORTD = PORTD & 0xBF; }

void set_4C (void)
{ PORTD = PORTD | 0x40; }

void reset_5A (void)
{ PORTJ = PORTJ & 0xDF; }

void set_5A (void)
{ PORTJ = PORTJ | 0x20; }

void reset_5B (void)
{ PORTJ = PORTJ & 0xBF; }

void set_5B (void)
{ PORTJ = PORTJ | 0x40; }

void reset_5C (void)
{ PORTJ = PORTJ & 0xEF; }

void set_5C (void)
{ PORTJ = PORTJ | 0x10; }

void reset_6A (void)
{ PORTJ = PORTJ & 0xF7; }

void set_6A (void)
{ PORTJ = PORTJ | 0x08; }

void reset_6B (void)
{ PORTJ = PORTJ & 0xFD; }

void set_6B (void)
{ PORTJ = PORTJ | 0x02; }

void reset_6C (void)
{ PORTJ = PORTJ & 0xFB; }

void set_6C (void)
{ PORTJ = PORTJ | 0x04; }


//--------------------------------------------------------------------------------
// TIMER1 initialize - prescale:1
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 400Hz
// actual value: 400.007Hz (0.0%)
//--------------------------------------------------------------------------------
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0x70; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x8F;
 OCR1AL = 0xFF;
 OCR1BH = 0x8F;
 OCR1BL = 0xFF;
 OCR1CH = 0x00;
 OCR1CL = 0x00;
 ICR1H  = 0x8F;
 ICR1L  = 0xFF;
 TCCR1A = 0x00;
 TCCR1C = 0x00;
 TCCR1B = 0x01; //start Timer
}

//--------------------------------------------------------------------------------
// timer1 comparatorA match with timer register ISR,
// This ISR used for reset A-joints of individual arm. 
//--------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
 //compare occured TCNT1=OCR1A
 if (arm_number == 0) { reset_1A(); }
 if (arm_number == 1) { reset_2A(); }
 if (arm_number == 2) { reset_3A(); }
 if (arm_number == 3) { reset_4A(); }
 if (arm_number == 4) { reset_5A(); }
 if (arm_number == 5) { reset_6A(); }
}

//--------------------------------------------------------------------------------
// timer1 comparatorB match with timer register ISR,
// This ISR used for reset B-joints of individual arm.
//--------------------------------------------------------------------------------
ISR(TIMER1_COMPB_vect)
{
 //compare occured TCNT1=OCR1B
 if (arm_number == 0) { reset_1B(); }
 if (arm_number == 1) { reset_2B(); }
 if (arm_number == 2) { reset_3B(); }
 if (arm_number == 3) { reset_4B(); }
 if (arm_number == 4) { reset_5B(); }
 if (arm_number == 5) { reset_6B(); }
}

//--------------------------------------------------------------------------------
// timer1 comparatorA match with timer register ISR,
// This ISR used for reset C-joints of indivisual arm. 
//--------------------------------------------------------------------------------
ISR(TIMER1_COMPC_vect)
{
 //compare occured TCNT1=OCR1c
 if (arm_number == 0) { reset_1C(); }
 if (arm_number == 1) { reset_2C(); }
 if (arm_number == 2) { reset_3C(); }
 if (arm_number == 3) { reset_4C(); }
 if (arm_number == 4) { reset_5C(); }
 if (arm_number == 5) { reset_6C(); }
}

//--------------------------------------------------------------------------------
// timer1 overflow ISR,
// This ISR can be used to load the PWM value. Here each Servo motor is
// move between 0 to 180 degrees proportional to the pulse ON time between 
// 0.5 to 2.2 ms with the frequency between 40 to 60 Hz. ie. 400Hz/8 = 50Hz
//--------------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)
{
 //TIMER1 has overflowed
 TCNT1H = 0x70; //reload counter high value
 TCNT1L = 0x01; //reload counter low value
 arm_number ++;

 if (arm_number>7)
 {
  	arm_number = 0;
 }
   
 if (arm_number == 0)
 {
  set_1A(); 
  set_1B(); 
  set_1C();
  OCR1AH = angle_upper_byte_1A;
  OCR1AL = angle_lower_byte_1A;
  OCR1BH = angle_upper_byte_1B;
  OCR1BL = angle_lower_byte_1B;
  OCR1CH = angle_upper_byte_1C;
  OCR1CL = angle_lower_byte_1C;
 }
 
 if (arm_number == 1)
 {
  set_2A(); 
  set_2B(); 
  set_2C();
  OCR1AH = angle_upper_byte_2A;
  OCR1AL = angle_lower_byte_2A;
  OCR1BH = angle_upper_byte_2B;
  OCR1BL = angle_lower_byte_2B;
  OCR1CH = angle_upper_byte_2C;
  OCR1CL = angle_lower_byte_2C;
 }
 
 if (arm_number == 2)
 {
  set_3A(); 
  set_3B(); 
  set_3C();
  OCR1AH = angle_upper_byte_3A;
  OCR1AL = angle_lower_byte_3A;
  OCR1BH = angle_upper_byte_3B;
  OCR1BL = angle_lower_byte_3B;
  OCR1CH = angle_upper_byte_3C;
  OCR1CL = angle_lower_byte_3C;
 }
 
 if (arm_number == 3)
 {
  set_4A(); 
  set_4B(); 
  set_4C();
  OCR1AH = angle_upper_byte_4A;
  OCR1AL = angle_lower_byte_4A;
  OCR1BH = angle_upper_byte_4B;
  OCR1BL = angle_lower_byte_4B;
  OCR1CH = angle_upper_byte_4C;
  OCR1CL = angle_lower_byte_4C;
 }
 
 if (arm_number == 4)
 {
  set_5A(); 
  set_5B(); 
  set_5C();
  OCR1AH = angle_upper_byte_5A;
  OCR1AL = angle_lower_byte_5A;
  OCR1BH = angle_upper_byte_5B;
  OCR1BL = angle_lower_byte_5B;
  OCR1CH = angle_upper_byte_5C;
  OCR1CL = angle_lower_byte_5C;
 }
 
 if (arm_number == 5)
 {
  set_6A(); 
  set_6B(); 
  set_6C();
  OCR1AH = angle_upper_byte_6A;
  OCR1AL = angle_lower_byte_6A;
  OCR1BH = angle_upper_byte_6B;
  OCR1BL = angle_lower_byte_6B;
  OCR1CH = angle_upper_byte_6C;
  OCR1CL = angle_lower_byte_6C;
 }
 
}

//--------------------------------------------------------------------------------
// function for angular movement calculation
//--------------------------------------------------------------------------------
void angle_value_calculation (void)
{
 unsigned int angle_value = 0;
 unsigned int temp = 0;
 if (degree > 180)
 degree = 180; // limiting the scope of the servo rotation
 
 angle_value = 0x8FAE + (139 * (unsigned char) degree); //actual constant is 139.4
 angle_lower_byte = (unsigned char) angle_value;        //separating the lower byte
 
 temp = angle_value >> 8;
 angle_upper_byte = (unsigned char) temp;               //separating the upper byte
}


//--------------------------------------------------------------------------------
// Function for individual 1A,1B..........6C joints angle calculation call & 
// store into respective variables. 
//--------------------------------------------------------------------------------
void angle_1A (unsigned char angle)                // joint-A of arm 1
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_1A = angle_upper_byte;
 angle_lower_byte_1A = angle_lower_byte;
}

void angle_1B (unsigned char angle)                // joint-B of arm 1
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_1B = angle_upper_byte;
 angle_lower_byte_1B = angle_lower_byte;
}

void angle_1C (unsigned char angle)                // joint-C of arm 1
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_1C = angle_upper_byte;
 angle_lower_byte_1C = angle_lower_byte;
}

void angle_2A (unsigned char angle)                // joint-A of arm 2
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_2A = angle_upper_byte;
 angle_lower_byte_2A = angle_lower_byte;
}

void angle_2B (unsigned char angle)                // joint-B of arm 2
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_2B = angle_upper_byte;
 angle_lower_byte_2B = angle_lower_byte;
}

void angle_2C (unsigned char angle)                // joint-C of arm 2
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_2C = angle_upper_byte;
 angle_lower_byte_2C = angle_lower_byte;
}

void angle_3A (unsigned char angle)                // joint-A of arm 3
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_3A = angle_upper_byte;
 angle_lower_byte_3A = angle_lower_byte;
}

void angle_3B (unsigned char angle)                // joint-B of arm 3
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_3B = angle_upper_byte;
 angle_lower_byte_3B = angle_lower_byte;
}

void angle_3C (unsigned char angle)                // joint-C of arm 3
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_3C = angle_upper_byte;
 angle_lower_byte_3C = angle_lower_byte;
}

void angle_4A (unsigned char angle)                // joint-A of arm 4
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_4A = angle_upper_byte;
 angle_lower_byte_4A = angle_lower_byte;
}

void angle_4B (unsigned char angle)                // joint-B of arm 4
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_4B = angle_upper_byte;
 angle_lower_byte_4B = angle_lower_byte;
}

void angle_4C (unsigned char angle)                // joint-C of arm 4
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_4C = angle_upper_byte;
 angle_lower_byte_4C = angle_lower_byte;
}

void angle_5A (unsigned char angle)                // joint-A of arm 5
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_5A = angle_upper_byte;
 angle_lower_byte_5A = angle_lower_byte;
}

void angle_5B (unsigned char angle)                // joint-B of arm 5
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_5B = angle_upper_byte;
 angle_lower_byte_5B = angle_lower_byte;
}

void angle_5C (unsigned char angle)                // joint-C of arm 5
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_5C = angle_upper_byte;
 angle_lower_byte_5C = angle_lower_byte;
}

void angle_6A (unsigned char angle)                // joint-A of arm 6
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_6A = angle_upper_byte;
 angle_lower_byte_6A = angle_lower_byte;
}

void angle_6B (unsigned char angle)                // joint-B of arm 6
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_6B = angle_upper_byte;
 angle_lower_byte_6B = angle_lower_byte;
}

void angle_6C (unsigned char angle)                // joint-C of arm 6
{
 degree = angle;
 angle_value_calculation();
 angle_upper_byte_6C = angle_upper_byte;
 angle_lower_byte_6C = angle_lower_byte;
}

//--------------------------------------------------------------------------------
// Call this function to initalise all servo motors in default 90 degrees. 
// Use this function while replacing servo motors of the robot for the calibration.
//--------------------------------------------------------------------------------
void robot_arm_calibration (void)
{
angle_1A(90); angle_1B(90); angle_1C(90); angle_2A(90); angle_2B(90); angle_2C(90);
angle_3A(90); angle_3B(90); angle_3C(90); angle_4A(90); angle_4B(90); angle_4C(90);
angle_5A(90); angle_5B(90); angle_5C(90); angle_6A(90); angle_6B(90); angle_6C(90);
}

//--------------------------------------------------------------------------------
// function for forward & backward storke, It pushes the robot in forward or backward
// direction depending on the selected direction
//--------------------------------------------------------------------------------
void wlalk_stroke(void)
{
angle_1A(135); angle_1B(125); angle_1C(90); angle_2A(90); angle_2B(125); angle_2C(90);
angle_3A(45); angle_3B(125); angle_3C(90); angle_4A(45); angle_4B(55); angle_4C(90);
angle_5A(90); angle_5B(55); angle_5C(90); angle_6A(135); angle_6B(55); angle_6C(90);
}

//--------------------------------------------------------------------------------
// function to stand the robot initial position
//--------------------------------------------------------------------------------
void robot_stand_position(void)
{
angle_1A(90); angle_1B(125); angle_1C(90); angle_2A(90); angle_2B(125); angle_2C(90);
angle_3A(90); angle_3B(125); angle_3C(90); angle_4A(90); angle_4B(55); angle_4C(90);
angle_5A(90); angle_5B(55); angle_5C(90); angle_6A(90); angle_6B(55); angle_6C(90);
}

//--------------------------------------------------------------------------------
// This function is used for positioning robot's arms in forward direction just 
// before applying stroke to push the robot forward.
//--------------------------------------------------------------------------------
void fwd_front_step()
{
 //2 and 4 move forward
 angle_2B(90); angle_2C(90); angle_4B(90);angle_4C(90); //2nd & 4th arm BC servo up by 90
 _delay_ms(250);
 angle_2A(70); angle_4A(65);      //move forward while arms are up
 _delay_ms(250);
 angle_2B(125); angle_4B(55);     //put down arm
 _delay_ms(250);
 
 //1 and 6 move forward
 angle_1B(90); angle_1C(90); angle_6B(90); angle_6C(90); //1st & 6th arm BC servo up by 90
 _delay_ms(250);
 angle_1A(115); angle_6A(155);   //move forward while arms are up
 _delay_ms(250);
 angle_1B(125); angle_6B(55);    //put down arm
 _delay_ms(250); 
 
 //3 and 5 move forward
 angle_3B(90); angle_3C(90); angle_5B(90); angle_5C(90); //3rd & 5th arm BC servo up by 90
 _delay_ms(250);
 angle_3A(25); angle_5A(110);    //move forward while arms are up
 _delay_ms(250);
 angle_3B(125); angle_5B(55);    // put down arm
 _delay_ms(250); 
}

//--------------------------------------------------------------------------------
// This function is used for positioning robot's arms in backward direction just 
// before applying stroke to push the robot backward.
//--------------------------------------------------------------------------------
void back_front_step()
{
 //arm 3 and 5 move back
 angle_3B(90); angle_3C(90); angle_5B(90); angle_5C(90); //3rd & 5th arm BC servo up by 90
 _delay_ms(250); 
 angle_3A(65); angle_5A(70);     //move backward while arms are up
 _delay_ms(250); 
 angle_3B(125); angle_5B(55);    // put down arm
 _delay_ms(250); 

 //arm 1 and 6 move back
 angle_1B(90); angle_1C(90); angle_6B(90); angle_6C(90); //1st & 6th arm BC servo up by 90
 _delay_ms(250); 
 angle_1A(155); angle_6A(115);   //move backward while arms are up
 _delay_ms(250); 
 angle_1B(125); angle_6B(55);    // put down arm
 _delay_ms(250);
 
 //arm 2 and 4 move back
 angle_2B(90); angle_2C(90); angle_4B(90); angle_4C(90); //2nd & 4th arm BC servo up by 90
 _delay_ms(250); 
 angle_2A(110); angle_4A(25);    //move backward while arms are up
 _delay_ms(250); 
 angle_2B(125); angle_4B(55);    // put down arm
 _delay_ms(250); 
}

//--------------------------------------------------------------------------------
// function to move repective servos of even and odd arms by specified angle
//--------------------------------------------------------------------------------
void even_A_servo_45 (void) 
{ angle_2A(45); angle_4A(45); angle_6A(45); }

void even_A_servo_135 (void)
{ angle_2A(135); angle_4A(135); angle_6A(135); }

void even_B_servo_up_90 (void)
{ angle_2B(90); angle_4B(90); angle_6B(90); }

void even_B_servo_down_125 (void)
{ angle_2B(125); angle_4B(55); angle_6B(55);}

void odd_A_servo_45 (void)
{ angle_1A(45); angle_3A(45); angle_5A(45); }

void odd_A_servo_135 (void) 
{ angle_1A(135); angle_3A(135); angle_5A(135); }

void odd_B_servo_up_90 (void)
{ angle_1B(90); angle_3B(90); angle_5B(90); }

void odd_B_servo_down_55 (void)
{ angle_1B(125); angle_3B(125); angle_5B(55); }

//--------------------------------------------------------------------------------
// This function is used for positioning robot's arms in clockwise direction just 
// before applying stroke to rotate the robot, clockwise.
//--------------------------------------------------------------------------------
void clock_wise_step(void)
{
 // legs are lifted and moved in the desired direction
 even_B_servo_up_90(); even_A_servo_45(); _delay_ms(250); 
 even_B_servo_down_125(); _delay_ms(250); 
 odd_B_servo_up_90(); odd_A_servo_45(); _delay_ms(250);
 odd_B_servo_down_55(); _delay_ms(250);
}

//--------------------------------------------------------------------------------
// This function is used for positioning robot's arms in counterclockwise direction 
// just before applying stroke to rotate the robot, counterclockwise.
//--------------------------------------------------------------------------------
void counter_clock_wise_step(void)
{
 // legs are lifted and moved in the desired direction 
 even_B_servo_up_90(); even_A_servo_135(); _delay_ms(250); 
 even_B_servo_down_125(); _delay_ms(250); 
 odd_B_servo_up_90(); odd_A_servo_135(); _delay_ms(250); 
 odd_B_servo_down_55(); _delay_ms(250); 
}
//--------------------------------------------------------------------------------
// function to turn right by single step in clockwise direction
//--------------------------------------------------------------------------------
void robot_rotate_clockwise (void)
{
 motion_busy = 1; 
 
 clock_wise_step();                       // move legs to desire direction
 robot_stand_position(); _delay_ms(250);  // apply rotation stroke
  
 motion_busy = 0; 
}

//--------------------------------------------------------------------------------
// function to turn left by single step  in counterclockwise direction
//--------------------------------------------------------------------------------
void robot_rotate_counterclockwise (void)
{
 motion_busy = 1; 
 
 counter_clock_wise_step();              // move legs to desire direction
 robot_stand_position(); _delay_ms(250); // apply rotation stroke

 motion_busy = 0; 
}

//--------------------------------------------------------------------------------
// function to move robot forward
//--------------------------------------------------------------------------------
void forward (void)
{
 motion_busy = 1; 
 fwd_front_step();         // move legs to desire direction
 wlalk_stroke();            // apply backward stroke
 motion_busy = 0; 
}

//--------------------------------------------------------------------------------
// function to move robot back
//--------------------------------------------------------------------------------
void back (void)
{
 motion_busy = 1; 
 back_front_step();        // move legs to desire direction
 wlalk_stroke();            // apply backward stroke
 motion_busy = 0; 
}

//--------------------------------------------------------------------------------
//call this routine to initialize all peripherals
//--------------------------------------------------------------------------------
void init_devices(void)
{
 //stop errant interrupts until set up
 cli();                                // disable all interrupts
 servo_pin_config();                   // servo configuration 
 timer1_init();                        // initilize timer1

 TIMSK1 = 0x0F;                        // timer1 interrupt sources
 
 sei(); //re-enable interrupts
 //all peripherals are now initialized
}

//--------------------------------------------------------------------------------
//main fuction starts here
//--------------------------------------------------------------------------------
int main(void) 
{
 init_devices();
 
 servo_on ();                     //turning on the servo supply

 //call the robot_arm_calibration(); function to calibrate the servo motors while replaceing.
 
 //robot_arm_calibration();         //call while replacing any servo or calibrating 
 //_delay_ms(1500);                 //each arm 

 
 robot_stand_position();  		  //must be called as the first instruction
 _delay_ms(3000);

// call the robot movement sequence here 
// move backward three step 
 forward();
 while(motion_busy == 1);
 forward();
 while(motion_busy == 1);
 forward();
 while(motion_busy == 1); 

 _delay_ms(3000);
// move backward three step 
 back();
 while(motion_busy == 1);
 back();
 while(motion_busy == 1);
 back(); 
 while(motion_busy == 1);

 _delay_ms(3000);

 // rotate counter-clockwise by three step
 robot_rotate_counterclockwise(); 
 while(motion_busy == 1);
 robot_rotate_counterclockwise(); 
 while(motion_busy == 1);
 robot_rotate_counterclockwise(); 
 while(motion_busy == 1); 

 _delay_ms(3000);

// rotate clockwise by three syep
 robot_rotate_clockwise();
 while(motion_busy == 1);
 robot_rotate_clockwise();
 while(motion_busy == 1);
 robot_rotate_clockwise();
 while(motion_busy == 1); 

 _delay_ms(3000);

 servo_off ();

 while(1)
 {}

}

//--------------------------------------------------------------------------------

