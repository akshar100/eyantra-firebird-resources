/********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe

 Date: 1st OCT 2010
 
 This experiment demonstrates Servo motor control using PWM module.

 Concepts covered:  Use of PWM module to generate PWM for servo motor control

 Fire Bird V ARM7 LPC2148 microcontroller board has connection for a single RC servo motor.
 Servo motors move between 0 to 180 degrees proportional to the pulse train 
 with the on time of 1 to 2 ms with the frequency between 40 to 60 Hz. 50Hz is most recommended.

 We are using PWM1 to generate servo control waveform.
 In this mode servo motors can be controlled with the angular resolution of 2.25 degrees.
 Although angular resolution is less this is very simple method.
 
 There are better ways to produce very high resolution PWM but it involves interrupts at the frequency of the PWM.
 High resolution PWM is used for servo control in the Hexapod robot.
  
 Connection Details:	P0.0/TXD0  PWM1 --> Servo 1: Camera pod pan servo
 						
 					  	


 Note: 
 
 1. Make sure that in the Target options following settings are 
 	done for proper operation of the code

 	Microcontroller: LPC2148
 	Frequency: 12 Mhz
 	Create Hex File: Checked (For more information read section 4.3.1 "Setting up Project in Keil uVision" in the hardware manual)

 2. Ensure that following settings are done in Startup.s configuration wizard:

 	Clock Settings:
	
	PLL Steup	 >>	 MSEL=5, PSEL=2
	VPBDIV Setup >>  VPBCLK = CPU Clock/4
	
	For more details refer section 4.8 in the hardware manual.

 3. 5V supply to these motors is provided by separate low drop voltage regulator "5V Servo" which can
 	supply maximum of 800mA current. It is a good practice to move one servo at a time to reduce power surge 
	in the robot's supply lines. Also preferably take ADC readings while servo motor is not moving or stopped
	moving after giving desired position.

 4. The pin used by servo motor control is also used for loading hex file during ISP. After programming is finished remove DB9 cable from the RS-232 port
 	and then run the servo code	by pressing reset
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

/*******Function Prototypes***********************/

void DelaymSec(unsigned int j);
void Servo_1(unsigned char degrees);
void Init_Servo_PWM(void);
void Init_Peripherals(void);
void Init_Ports(void);
void UpdateServoPos(unsigned int pos);
void Servo_1_Free(void);
void Init_Buzzer_Pin(void);

/*************************************************/



void DelaymSec(unsigned int j)		  //Generates 1mSec delay
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

	Function 		: Init_Servo_PWM
	Return type		: None
	Parameters		: None
	Description 	: Initialises PWM1 for Servo motor Control
************************************************************/

void Init_Servo_PWM()
{
 PINSEL0&=0xFFFFFFFC;
 PINSEL0|=0x00000002;		//Enabling P0.0 as PWM1
 
 PWMPR	= 150;	//PWM Prescaler PCLK/150 = 100KHz
 PWMPC	= 0;	//PWMPC increments on every PCLK
 PWMTC	= 0;	//PWMTC increments on every PWMPC=PWMPR
 PWMMR0 = 2000;	//PWM base frequency 100KHz/2000=50Hz	 		
 PWMMR1 = 60;
 PWMMR2 = 0;
 PWMMR3 = 0;
 PWMMR4 = 0;
 PWMMR5 = 0;
 PWMMR6 = 0;
 PWMMCR = 0x00000002;
 PWMPCR	= 0x2600;
 PWMLER	= 0x7F;
 PWMTCR = 0x01;
}  


//Function to update Servo motor Position
void UpdateServoPos(unsigned int pos)
{
 PWMMR1 = pos;
 PWMLER = 0x02;
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.0 degree

/******************************************************************************

Servo Angle vs Duty cycle

	0' --> 0.6mSec
	180' --> 2.4mSec
******************************************************************************/
void Servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
 PositionPanServo = ((float)degrees / 1.0) + 60.0;
 UpdateServoPos((unsigned int)PositionPanServo);
}

void Servo_1_Free(void)	  //makes servo 1 free rotating
{
 PWMMR1 = 1999;
 PWMLER = 0x02;
}

void Init_Ports(void)
{
 Init_Buzzer_Pin();
}

void Init_Peripherals(void)
{
 Init_Ports();
 Init_Servo_PWM();
}

int main(void)
{ 
 unsigned int count=0; 
 PINSEL0 = 0x00000000;		// Initially set all pins as GPIO
 PINSEL1 = 0x00000000;
 PINSEL2 = 0x00000000;
 Init_Peripherals();
 DelaymSec(2000);
 for(count=0;count<180;count++)
 {
  Servo_1(count);
  DelaymSec(5);
 }
 DelaymSec(2000);
 Servo_1_Free();
 while(1);
}
