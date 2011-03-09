/**************************************************************************************************
		Platform: Fire Bird V P89V51RD2
		IO Operations Motor Direction Control
		Written by: Omkar Pradhan, NEX Robotics Pvt. Ltd.
		Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
		Last Modification: 2009-12-08
		This program shows how to control directions of the robot.
		Compiled with: uVision3 V3.90; C Compiler: C51.Exe, V8.18
**************************************************************************************************/								 

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

sbit buzzer=P2^7;  // buzzer is connected to P2.7. buzzer = 1; buzzer off, buzzer = 0; buzzer on, 
sbit LB=P1^0;	   //LEFT BACK Pin
sbit LF=P1^1;		//LEFT FRONT Pin
sbit RF=P1^2;		//RIGHT FRONT Pin
sbit RB=P3^4;		//RIGHT BACK Pin
sbit left_velocity=P1^3;//Left velocity control pin. 
sbit right_velocity=P1^4;//Right velocity control pin. 

// function for giving a delay of ms milliseconds

void delay_ms(unsigned int ms)
{
unsigned int i,j;

for(i=0;i<ms;i++)
for(j=0;j<53;j++);
}

	
/**********************************************************************************
RF =  RIGHT MOTOR FORWARD
RB = RIGHT MOTOR BACKWARD
LF = LEFT MOTOR FORWARD
LB = LEFT MOTOR BACKWARD
**********************************************************************************/


//direction control subroutines

void forward(void)
{


RF=1;
LF=1;
RB=0;
LB=0;
}

void backward(void)
{


RF=0;
LF=0;
RB=1;
LB=1;
}


void left(void)
{
RF=1;
LF=0;
RB=0;
LB=1;
}

void right(void)  
{
RF=0;
LF=1;
RB=1;
LB=0;
}

void stop(void)
{

RF=0;
LF=0;
RB=0;
LB=0;
}


//main function starts here
void main (void)

{

buzzer=1;		  //switch off the buzzer
left_velocity=1;  //setting this pin to one sets the motor to run at maximum velocity. 
				  //Thus enable pin of Motor driver is always on unlike in PWM mode.
right_velocity=1; //setting this pin to one sets the motor to run at maximum velocity.
				  //Thus enable pin of Motor driver is always on unlike in PWM mode
				 
while(1)
{
forward();
delay_ms(2000); //motor moves in forward direction for 2000 milliseconds

stop();
delay_ms(1000); //motor stops for 1000 milliseconds

backward();
delay_ms(2000); //motor moves in backward direction for 2000 milliseconds

stop();
delay_ms(1000); //motor stops for 1000 milliseconds

left();
delay_ms(2000); //motor moves in left direction for 2000 milliseconds

stop();
delay_ms(1000); //motor stops for 1000 milliseconds

right();
delay_ms(2000); //motor moves in right direction for 2000 milliseconds

stop();
delay_ms(2000); //motor stops for 2000 milliseconds

}
}



