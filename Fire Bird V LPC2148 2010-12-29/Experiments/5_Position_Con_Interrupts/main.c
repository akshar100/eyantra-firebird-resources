 /********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe

 Date: 1st OCT 2010
 
 This experiment demonstrates use of position encoders.

 Concepts covered: External Interrupts, Position control
 
 Microcontroller pins used:
 P0.22, P1.21, P0.10, P0.11: Robot direction control
 P0.7/PWM2 and P0.21/PWM5: Robot velocity control. Currently set to 1 as PWM is not used
 P0.15 (EINT2): External interrupt for left motor position encoder 
 P0.16 (EINT0): External interrupt for the right position encoder

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

 3. Sudden change in the motor direction produces current surge up to 1.6Amps.
 	Battery can give current up to 3Amps. Auxiliary supply can give current up to 1.5Amp at peak load
	When robot is on Battery power we can do sudden change motion direction as battery can supply current up to 3Amps
    Motors are stopped for 0.5 seconds between consecutive motion commands to keep current surge below 1.5Amp.
	Stop motors for at least 0.3 seconds before reversing its direction if it is running at full speed.
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

/*********Functio Prototypes************/

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
void Angle_Rotate(unsigned int Degrees);
void Linear_Distance_mm(unsigned int DistanceInMM);
void Forward_mm(unsigned int DistanceInMM);
void Back_mm(unsigned int DistanceInMM);
void Left_Degrees(unsigned int Degrees);
void Soft_Left_Degrees(unsigned int Degrees);
void Soft_Left_2_Degrees(unsigned int Degrees);
void Right_Degrees(unsigned int Degrees);
void Soft_Right_Degrees(unsigned int Degrees);
void Soft_Right_2_Degrees(unsigned int Degrees);
void Init_Motion_Pin(void);
void Init_Buzzer_Pin(void);
void Init_Peripherals(void);
void Init_Ports(void);
void  __irq IRQ_Eint0(void);
void  __irq IRQ_Eint2(void);
void Ext_INT_Setup(void);

/******************************************/


/*************Global Variables*************/

volatile unsigned int Left_Shaft_Count=0;
volatile unsigned int Right_Shaft_Count=0;
/******************************************/


void DelaymSec(unsigned int j)
{  
 unsigned int  i;
 for(;j>0;j--)
 {
  for(i=0; i<10000; i++);
 } 
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

//Function used for turning robot by specified degrees
void Angle_Rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 Left_Shaft_Count = 0; 
 Right_Shaft_Count = 0; 
 while (1)
 {
  if((Right_Shaft_Count >= ReqdShaftCountInt) | (Left_Shaft_Count >= ReqdShaftCountInt))
  break;
 }
 Stop(); //Stop robot
}

//Function used for moving robot forward by specified distance
void Linear_Distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
  
 Right_Shaft_Count = 0;
 while(1)
 {
  if(Right_Shaft_Count > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 Stop(); //Stop robot
}

void Forward_mm(unsigned int DistanceInMM)
{
 Forward();
 Linear_Distance_mm(DistanceInMM);
}

void Back_mm(unsigned int DistanceInMM)
{
 Back();
 Linear_Distance_mm(DistanceInMM);
}

void Left_Degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 Left(); //Turn left
 Angle_Rotate(Degrees);
}



void Right_Degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 Right(); //Turn right
 Angle_Rotate(Degrees);
}


void Soft_Left_Degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 Soft_Left(); //Turn soft left
 Degrees=Degrees*2;
 Angle_Rotate(Degrees);
}

void Soft_Right_Degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 Soft_Right();  //Turn soft right
 Degrees=Degrees*2;
 Angle_Rotate(Degrees);
}

void Soft_Left_2_Degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 Soft_Left2(); //Turn reverse soft left
 Degrees=Degrees*2;
 Angle_Rotate(Degrees);
}

void Soft_Right_2_Degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 Soft_Right2();  //Turn reverse soft right
 Degrees=Degrees*2;
 Angle_Rotate(Degrees);
}


void Ext_INT_Setup(void)
{
 PINSEL0&= 0x3FFFFFFF;
 PINSEL0|= 0x80000000; //Enabling P0.15 as EINT2
 PINSEL1&= 0xFFFFFFFC;
 PINSEL1|= 0x00000001; //Enabling P0.16 as EINT0


 EXTMODE = 0x05;	// EINT2 and EINT0 is edge sensitive
 EXTPOLAR = 0x00;					// EINT2 and EINT0 in triggered on falling edge

 VICIntSelect = 0x00000000;		// Setting EINT2 and EINt0 as IRQ(Vectored)
 VICVectCntl0 = 0x20|16;		// Assigning Highest Priority Slot to EINT2 and enabling this slot
 VICVectAddr0 = (int)IRQ_Eint2; // Storing vector address of EINT2

 VICVectCntl1 = 0x20|14;		// Assigning second Highest Priority Slot to EINT0 and enabling this slot
 VICVectAddr1 = (int)IRQ_Eint0; // Storing vector address of EINT0
 EXTINT = 0x05;	//Clearing EINT2 & EINT0 interrupt flag
 	
 VICIntEnable = (1<<16) | (1<<14);	// Enable EINT2	& EINT0 flags
}


//ISR for EINT0
void  __irq IRQ_Eint0(void)
{  
   Right_Shaft_Count++;
   EXTINT = 0x01;				// Clear EINT0 flag
   VICVectAddr = 0x00;   		//Acknowledge Interrupt
}	


//ISR for EINT2
void  __irq IRQ_Eint2(void)
{  
   Left_Shaft_Count++;
   EXTINT = 0x04;				// Clear EINT2 flag
   VICVectAddr = 0x00;   		//Acknowledge Interrupt
}	

void Init_Ports(void)
{
 Init_Motion_Pin();
 Init_Buzzer_Pin();

}


void Init_Peripherals(void)
{
 Init_Ports();
 Ext_INT_Setup();
}
 

int  main(void)
{  
 PINSEL0 = 0x00000000;		// Enable GPIO on all pins
 PINSEL1 = 0x00000000;
 PINSEL2 = 0x00000000;
 Init_Peripherals();
 while(1)	
 { 
  	Forward_mm(100);
	Stop();
	DelaymSec(500);

	Back_mm(100);
	Stop();
	DelaymSec(500);

	Left_Degrees(90);
	Stop();
	DelaymSec(500);

	Right_Degrees(90);
	Stop();
	DelaymSec(500);

	Soft_Left_Degrees(90);
	Stop();
	DelaymSec(500);

	Soft_Right_Degrees(90);
	Stop();
	DelaymSec(500);

	Soft_Left_2_Degrees(90);
	Stop();
	DelaymSec(500);

	Soft_Right_2_Degrees(90);
	Stop();
	DelaymSec(500);
	
 }
}
