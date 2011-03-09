/********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe

 Date: 1st OCT 2010
 
 This experiment demonstrates simple motion control.

 Concepts covered: Simple motion control

 There are two components to the motion control:
 1. Direction control using pins P0.22, P1.21, P0.10, P0.11
 2. Velocity control by PWM on pins P0.7/PWM2 and P0.21/PWM5.

 In this experiment for the simplicity P0.7 and P0.21 are kept at logic 1.
 
 
 Connection Details:  	L-1---->P1.21;		L-2---->P0.22;
   						R-1---->P0.10;		R-2---->P0.11;
   						P0.7 (PWM2) ----> Logic 1; 	P0.21 (PWM5) ----> Logic 1; 


 Note: 
 
 1. Note: Make sure that in the Target options following settings are 
 	done for proper operation of the code

 	Microcontroller: LPC2148
 	Frequency: 12 Mhz
 	Create Hex File: Checked (For more information read section 4.3.1 "Setting up Project in Keil uVision" in the hardware manual)

 2. Ensure that following settings are done in Startup.s configuration wizard:

 	Clock Settings:
	
	PLL Steup	 >>	 MSEL=5, PSEL=2
	VPBDIV Setup >>  VPBCLK = CPU Clock/4
	
	For more details refer section 4.8 in the hardware manual.

 3. Auxiliary power can supply current up to 1 Ampere while Battery can supply current 
 	up to 2 Ampere. When robot is on auxiliary power and robot changes is direction suddenly 
	from forward to backward or vice versa, it will cause large current surge and 
	robot's microcontroller might get reset. This will happen repeatedly and you might 
	think that problem is in the software. To avoid this problem use stop for at 
	least 0.5 second before changing direction. This problem will not occur when robot 
	is on battery power and batteries have sufficiently charge. But still it is a 
	good practice to stop robot for 0.5 seconds before changing the direction to increase 
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

#include  <lpc214x.h>


/**********Macros******************/

#define BUZZER_OFF() IO0CLR=(1<<25)		   //Macro to turn OFF buzzer
#define BUZZER_ON() IO0SET=(1<<25)		   //Macro to turn ON buzzer

/***********************************/

/*****Function Prototypes*********************/

void DelaymSec(unsigned int j);
void Forward(void);
void Back(void);
void Left(void);
void Right(void);
void Stop(void);
void Soft_Left(void);
void Soft_Right(void);
void Soft_Left2(void);
void Soft_Right2(void);
void L_Forward(void);
void L_Back(void);
void R_Forward(void);
void R_Back(void);
void L_Stop(void);
void R_Stop(void);
void Init_Motion_Pin(void);
void Init_Buzzer_Pin(void);
void Init_Peripherals(void);
void Init_Ports(void);

/**********************************************/


void DelaymSec(unsigned int j)		//Generates milli second delay
{  
 unsigned int  i;
 for(;j>0;j--)
 {
  for(i=0; i<10000; i++);
 } 
}


/************************************************************

	Function 		: Init_Buzzer_Pin
	Return type		: None
	Parameters		: None
	Description 	: Initialises Buzzer pin
************************************************************/

void Init_Buzzer_Pin(void)
{
 PINSEL1&=0xFFF3FFFF;		
 PINSEL1|=0x00000000; 		//Set P0.25 as GPIO
 IO0DIR&=0xFDFFFFFF;
 IO0DIR|= (1<<25);  		//Set P0.25 as Output
 BUZZER_OFF();				//Initially turn OFF buzzer
}


/************************************************************

	Function 		: Init_Motion_Pin
	Return type		: None
	Parameters		: None
	Description 	: Initialises Motion control pins
************************************************************/

void Init_Motion_Pin(void)
{
 PINSEL0&=0xFF0F3FFF;		
 PINSEL0|=0x00000000;		//Set Port pins P0.7, P0.10, P0.11 as GPIO
 PINSEL1&=0xFFFFF0FF;
 PINSEL1|=0x00000000;		//Set Port pins P0.21 and 0.22 as GPIO
 IO0DIR&=0xFF9FF37F;
 IO0DIR|= (1<<10) | (1<<11) | (1<<21) | (1<<22) | (1<<7); 	//Set Port pins P0.10, P0.11, P0.21, P0.22, P0.7 as Output pins
 IO1DIR&=0xFFDFFFFF;
 IO1DIR|= (1<<21);		// Set P1.21 as output pin
 Stop();				// Stop both the motors on start up
 IO0SET = 0x00200080;	// Set PWM pins P0.7/PWM2 and P0.21/PWM5 to logic 1
}

//Function to move Left motor forward
void L_Forward(void)
{
 IO1SET = 0x00200000;		//Set P1.21 to logic '1'
}

//Function to move Left motor backward
void L_Back(void)
{
 IO0SET = 0x00400000;		//Set P0.22 to logic '1'
}

//Function to move Right motor forward
void R_Forward(void)
{
 IO0SET = 0x00000400;		//Set P0.10 to logic '1'
}

//Function to move Right motor backward
void R_Back(void)
{
 IO0SET = 0x00000800;		//Set P0.11 to logic '1'
}

//Function to stop left motor
void L_Stop(void)
{
 IO1CLR = 0x00200000;		//Set P1.21 to logic '0'
 IO0CLR = 0x00400000;		//Set P0.22 to logic '0'
}

//Function to stop Right motor
void R_Stop(void)
{
 IO0CLR = 0x00000400;		//Set P0.10 to logic '0'
 IO0CLR = 0x00000800;		//Set P0.11 to logic '0'
}

//Function to move robot in forward direction
void Forward(void)
{
 Stop();
 L_Forward();
 R_Forward();
}

//Function to move robot in backward direction
void Back(void)
{
 Stop();
 L_Back();
 R_Back();
}

//Function to turn robot in Left direction
void Left(void)
{ 
 Stop();
 L_Back();
 R_Forward();
}

//Function to turn robot in right direction
void Right(void)
{ 
 Stop();
 L_Forward();
 R_Back();
}

//Function to turn robot in Left direction by moving right wheel forward
void Soft_Left(void)
{
 Stop();
 R_Forward();
}

//Function to turn robot in right direction by moving left wheel forward
void Soft_Right(void)
{
 Stop();
 L_Forward();
}

//Function to turn robot in left direction by moving left wheel backward
void Soft_Left2(void)
{
 Stop();
 L_Back();
}

//Function to turn robot in right direction by moving right wheel backward 
void Soft_Right2(void)
{
 Stop();
 R_Back();
}

//Function to stop the robot at its current location
void Stop(void)
{
 L_Stop();
 R_Stop();
}

void Init_Ports(void)
{
 Init_Motion_Pin();
 Init_Buzzer_Pin();
}

void Init_Peripherals(void)
{
 Init_Ports();
}


int main(void)
{  
 PINSEL0 = 0x00000000;		// Enable GPIO on all pins
 PINSEL1 = 0x00000000;
 PINSEL2 = 0x00000000;
 Init_Peripherals();
 while(1)	
 { 
  Forward();
  DelaymSec(1000);
  Stop();
  DelaymSec(500);

  Back();
  DelaymSec(1000);
  Stop();
  DelaymSec(500);

  Left();
  DelaymSec(1000);
  Stop();
  DelaymSec(500);

  Right();
  DelaymSec(1000);
  Stop();
  DelaymSec(500);

  Soft_Left();
  DelaymSec(1000);
  Stop();
  DelaymSec(500);

  Soft_Right();
  DelaymSec(1000);
  Stop();
  DelaymSec(500);

  Soft_Left2();
  DelaymSec(1000);
  Stop();
  DelaymSec(500);

  Soft_Right2();
  DelaymSec(1000);
  Stop();
  DelaymSec(1000);
 }
 
}

