/**************************************************************************************************************
		Platform: Fire Bird V P89V51RD2
		Position Control Using Timer 1 as Counter
		Written by: Omkar Pradhan, NEX Robotics Pvt. Ltd.
		Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
		Last Modification: 2009-12-08
		This programme counts encoder pulses from the right motor's position encoder using Timer 1 as counter
		at its external clock pin,  compares them to a required distance and stops robot after covering
		required distance. 
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

sbit LB=P1^0;  //LEFT BACK Pin
sbit LF=P1^1;  //LEFT FRONT Pin
sbit RF=P1^2;  //RIGHT FRONT Pin
sbit RB=P3^4;  //RIGHT BACK Pin
sbit left_velocity=P1^3; //Left velocity control pin. 
sbit right_velocity=P1^4; //Right velocity control pin. 

unsigned int right_shaft_count=0; //variable used to store pulse count from right position encoder

//initializing timer/counter1 as counter in mode 2
void  timer1_setup(void)   
{
 TMOD=0x60; // Timer 1 in 8 bit external counter mode
 TH1=0; // reset counter value to 0       
 TL1=0;	// reset counter value to 0  
}
//motor control
void forward(void)
{
RF=1;
LF=1;
RB=0;
LB=0;
}

void stop(void)
{
RF=0;
LF=0;
RB=0;
LB=0;
}

void main()
{
 unsigned int distance=0;
 unsigned int reqd_shaft_count_int=0;
 
 timer1_setup();  //Timer 1 initialization 
 distance=100;    //Enter here the distance to be travelled in mm    
 reqd_shaft_count_int=(unsigned int) (distance*100 / 544);  //This equation will calculate the count required for distance to be travelled
  
 left_velocity=1; //setting this pin to one sets the motor to run at maximum velocity. Here enable pin of Motor driver is always on unlike in PWM mode.
 right_velocity=1;//setting this pin to one sets the motor to run at maximum velocity. Here enable pin of Motor driver is always on unlike in PWM mode
 
 TR1=1;//start timer 1 
 forward();  //set the robot to move forward     
   
 while(reqd_shaft_count_int > right_shaft_count) //loop until the required distance is travelled
 {
  right_shaft_count = TL1;//store the contents of TL1 in left_shaft_count 
 }

 stop();	 //stop the robot once requisite distance has been achieved
 
 while(1);
}//main ends