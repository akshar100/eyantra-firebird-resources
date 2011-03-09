/**************************************************************************************************************
		Platform: Fire Bird V P89V51RD2
		PWM using PCA (Timers and Counters)
		Written by: Omkar Pradhan, NEX Robotics Pvt. Ltd.
		Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
		Last Modification: 2009-12-08
		This programme controls the velocity of the motors by (PWM) Pulse Width Modulation
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

//direction control bits

sbit LB=P1^0;  //LEFT BACK Pin
sbit LF=P1^1;  //LEFT FRONT Pin
sbit RF=P1^2;  //RIGHT FRONT Pin
sbit RB=P3^4;  //RIGHT BACK Pin

//Set left motor with velocity vel(compare value). 0x00 will give full (100% duty cycle) velocity, while 0xFF will give zero (0% duty cycle) velocity. Any value in between 0x00 and 0xFF will give intermediate velocity.
void left_motor_velocity(unsigned char vel)
{
CCAP0H=vel;
}

//Set right motor with velocity vel(compare value). 0x00 will give full (100% duty cycle) velocity, while 0xFF will give zero (0% duty cycle) velocity. Any value in between 0x00 and 0xFF will give intermediate velocity.
void right_motor_velocity(unsigned char vel)
{
CCAP1H=vel;
}

// motor direction control subroutines
void forward(void)
{
RF=1;
LF=1;
RB=0;
LB=0;
}

//initialise programmable counter array module to generate pwm
//The CCAPnH register holds the reload value. CCAPnL register increments,and is loaded with this value everytime it overflows
//Thus the value in CCAPnH can be changed to change the motor velocity
void pca_init(void)
{
 CMOD=0x80;	   //80 sets PCA counter to run at Fosc/6; 
 CCON=0x40;	   //start PCA counter
 						
 CCAP0L=0x00;   
 CCAP0H=0xFF;  //Left motor duty cycle register
  
 CCAP1L=0x00;				  
 CCAP1H=0xFF;  //Right motor duty cycle register

 CCAPM0=0x42;  //enable PWM mode and ECOM bits for left motor
 CCAPM1=0x42;  //enable PWM mode and ECOM bits for right motor
}

void main()
{
pca_init();
/*******************************************************************************************
 First set velocity for each motor by calling  left_motor_vel & right_motor_vel	 functions
 Then set the robot to move forward by calling forward function.
 Robot can be made to move right or left by varying the left and right motor velocities.
 If left motor velocity < right motor velocity then robot moves left or vice versa.
 ******************************************************************************************/  

left_motor_velocity(0x1F);  //0x00 will give full (100% duty cycle) velocity, while 0xFF will give zero (0% duty cycle) velocity. Any value in between 0x00 and 0xFF will give intermediate velocity.			
right_motor_velocity(0x1F); //0x00 will give full (100% duty cycle) velocity, while 0xFF will give zero (0% duty cycle) velocity. Any value in between 0x00 and 0xFF will give intermediate velocity.

forward();

while(1);
}

