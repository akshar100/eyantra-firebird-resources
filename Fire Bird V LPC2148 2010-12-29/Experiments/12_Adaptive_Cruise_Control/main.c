/********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe

 Date: 1st OCT 2010
  
 Application example: White line following

 This experiment demostrates the application of a simple line follower robot. The 
 robot follows a white line over a black backround at the same time it maintains 
 safe distance with the robot ahead of it.
 
 Connection Details:  	L-1---->P1.21;		L-2---->P0.22;
   						R-1---->P0.10;		R-2---->P0.11;
   						P0.7 (PWM2) ----> Logic 1; 	P0.21 (PWM5) ----> Logic 1;

 Concepts covered:  ADC, LCD interfacing, motion control based on sensor data

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> P1.19
			  RW  --> P1.18
			  EN  --> P1.17
			  DB7 --> P1.25
			  DB6 --> P1.24
			  DB5 --> P1.23
			  DB4 --> P1.22

 ADC Connection:
 			  	Sensor									  ADC Channel No.

			  	Battery Voltage									AD1.4(P0.13)
			  	White line sensor 1								AD1.3(P0.12)
			  	White line sensor 2								AD0.1(P0.28)
			  	White line sensor 3								AD0.2(P0.29)
			  	IR Proximity analog sensor 2*****				AD0.6(P0.4)
			  	IR Proximity analog sensor 4*****				AD0.7(P0.5)
			  	Sharp IR range sensor 2							AD0.6(P0.4)
			  	Sharp IR range sensor 3							AD1.0(P0.6)
			  	Sharp IR range sensor 4							AD0.7(P0.5)
			  	

 ***** For using Analog IR proximity (2 and 4) sensors ensure that OE resistors are soldered and remove the respective sharp sensors. 
 	    
 
 LCD Display interpretation:
 ****************************************************************************
 *BLANK			         BLANK 			       BLANK     		TERY VOLTAGE*
 *LEFT WL SENSOR			  CENTER WL SENSOR				RIGHT WL SENSOR	*
 ****************************************************************************
 
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

 3. Make sure that you copy the lcd.c file in your folder

 4. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

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


#include <lpc214x.h>
#include "LCD.h"		//This header files defines LCD related prototype functions
#include <math.h>

/*************Macros******************/

#define BUZZER_OFF() IO0CLR=(1<<25)		   //Macro to turn OFF buzzer
#define BUZZER_ON() IO0SET=(1<<25)		   //Macro to turn ON buzzer

/************************************/


/*********Function Prototypes********/

void Init_Peripherals(void);
void Init_Ports(void);
void Init_Buzzer_Pin(void);
void Init_ADC_Pin(void);
void Init_ADC0(void);
void Init_ADC1(void);
unsigned int AD0_Conversion(unsigned char channel);
unsigned int AD1_Conversion(unsigned char channel);
unsigned int Sharp_GP2D12_Estimation(unsigned int Val);
unsigned int Batt_Voltage_Conversion(unsigned int Val);
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
void Init_PWM(void);
void UpdateLeftPWM(unsigned int vel);
void UpdateRightPWM(unsigned int vel);
void UpdateVelocity(unsigned int Left,unsigned int Right);


/*************************************/


/**********Global variables***********/

extern unsigned char String1[16];	//This variable is defined in LCD.c
extern unsigned char String2[16];	//This variable is defined in LCD.c
unsigned int ADC_Data[8];

/*************************************/

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
 //IO0SET = 0x00200080;	// Set PWM pins P0.7/PWM2 and P0.21/PWM5 to logic 1. This pins are set as PWM pins in Init_PWM fuction
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

/************************************************************

	Function 		: Init_ADC_Pin
	Return type		: None
	Parameters		: None
	Description 	: Initialises the required GPIOs as ADC pins
************************************************************/

void Init_ADC_Pin(void)
{
 PINSEL0&= 0xF0FFC0FF;
 PINSEL0|= 0x0F003F00;		//Set pins P0.4, P0.5, P0.6, P0.12, P0.13 as ADC pins
 PINSEL1&= 0xF0FFFFFF;		
 PINSEL1|= 0x05000000;	    //Set pins P0.28, P0.29 as ADC pins
}

/************************************************************

	Function 		: ADC0_Init
	Return type		: None
	Parameters		: None
	Description 	: This fuction initialises ADC 0
					  module of LPC2148 microcontroller. It also 
					  configures the required I/o pins to be used as 
					  ADC pins. 
************************************************************/
void Init_ADC0(void)
{
 AD0CR=0x00200E00;	// SEL = 1 	ADC0 channel 1	Channel 1
					// CLKDIV = Fpclk / 1000000 - 1 ;1MHz
					// BURST = 0 
					// CLKS = 0 
 					// PDN = 1 
 					// START = 1
  					// EDGE = 0 (CAP/MAT)
} 


/************************************************************

	Function 		: ADC1_Init
	Return type		: None
	Parameters		: None
	Description 	: This fuction initialises ADC 1
					  module of LPC2148 microcontroller. It also 
					  configures the required I/o pins to be used as 
					  ADC pins. 
************************************************************/
void Init_ADC1(void)
{
 AD1CR=0x00200E00;	// SEL = 1 	ADC0 channel 1	Channel 1
					// CLKDIV = Fpclk / 1000000 - 1 ;1MHz
					// BURST = 0 
					// CLKS = 0 
 					// PDN = 1 
 					// START = 1
  					// EDGE = 0 (CAP/MAT)
} 
                           

//This function converts ADC0 channels. Channel number is passed to this function as integer.
unsigned int AD0_Conversion(unsigned char channel)
{
 unsigned int Temp;
 if(channel!=0)
 {
  AD0CR = (AD0CR & 0xFFFFFF00) | (1<<channel);
 }
 else
 {
  AD0CR = (AD0CR & 0xFFFFFF00) | 0x01;
 }
 AD0CR|=(1 << 24);
 while((AD0GDR&0x80000000)==0);
 Temp = AD0GDR;						
 Temp = (Temp>>8) & 0xFF;
 return Temp;
}

//This function converts ADC1 channels. Channel number is passed to this function as integer.
unsigned int AD1_Conversion(unsigned char channel)
{
 unsigned int Temp;
 if(channel!=0)
 {
  AD1CR = (AD1CR & 0xFFFFFF00) | (1<<channel);
 }
 else
 {
  AD1CR = (AD1CR & 0xFFFFFF00) | 0x01;
 }
 AD1CR|=(1 << 24);
 while((AD1GDR&0x80000000)==0);
 Temp = AD1GDR;						
 Temp = (Temp>>8) & 0xFF;
 return Temp;
}

//This Function estimates the raw digital data of Sharp sensor in mm
unsigned int Sharp_GP2D12_Estimation(unsigned int Val)
{
 float Distance;
 unsigned int DistanceInt;
 Distance = (int)(10.00*(2799.6*(1.00/(pow(Val,1.1546)))));
 DistanceInt = (int)Distance;
 if(DistanceInt>800)
 {
  DistanceInt=800;
 }
 return DistanceInt;
}

//This function convetrs the raw digital data of battery to represent actual battery voltage
unsigned int Batt_Voltage_Conversion(unsigned int Val)
{
 float Batt_Voltage;
 unsigned int Batt_Voltage_Int=0;
 Batt_Voltage = ((float)Val* 0.05226 * 100.0) + 0.7;	//0.05226= (3.3/255)*(1/(3.3/13.3))
														//0.7 = Drop across diode
														//100.0 = Shifting decimal point by 2
 Batt_Voltage_Int = (unsigned int)Batt_Voltage;
 return Batt_Voltage_Int;
}

/************************************************************

	Function 		: Init_PWM
	Return type		: None
	Parameters		: None
	Description 	: Initialises PWM module for motion control
************************************************************/

void Init_PWM(void)
{
 PINSEL0&=0xFFFF3FFF;
 PINSEL0|=0x00008000;	//Enabling P0.7 as PWM2
 PINSEL1&=0xFFFFF3FF;		
 PINSEL1|=0x00000400;	//Enabling P0.22 as PWM5

 PWMPR	= 30;	//PWM Prescaler PCLK/30 = 500KHz
 PWMPC	= 0;	//PWMPC increments on every PCLK
 PWMTC	= 0;	//PWMTC increments on every PWMPC=PWMPR
 PWMMR0 = 500;	//PWM base frequency 500KHz/500=1KHz	 
 PWMMR1 = 0;
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


void UpdateLeftPWM(unsigned int vel)
{
 PWMMR2 = vel;
 PWMLER = 0x04;
}

void UpdateRightPWM(unsigned int vel)
{
 PWMMR5 = vel;
 PWMLER = 0x20;
}

void UpdateVelocity(unsigned int Left,unsigned int Right)
{
 UpdateLeftPWM(Left);
 UpdateRightPWM(Right);
}

void Init_Ports(void)
{
 Init_LCD_Pin();
 Init_Buzzer_Pin();
 Init_Motion_Pin();
 Init_ADC_Pin();
 
}

void Init_Peripherals(void)
{
 Init_Ports();
 Init_ADC0();
 Init_ADC1();
 Init_PWM();
}


int main(void)
{ 
 unsigned int Temp=0;
 unsigned int Front_Distance=0; 
 PINSEL0 = 0x00000000;		// Enable GPIO on all pins
 PINSEL1 = 0x00000000;
 PINSEL2 = 0x00000000;
 DelaymSec(40);
 Init_Peripherals();
 LCD_4Bit_Mode();
 LCD_Init();
 LCD_Command(0x01);
 DelaymSec(20);
   
 while(1)	
 { 
  ADC_Data[5] = AD1_Conversion(4);	  //Batt
  ADC_Data[1] = AD1_Conversion(0);	  //Front Sharp
  Front_Distance = Sharp_GP2D12_Estimation(ADC_Data[1]); 
  ADC_Data[3] = AD1_Conversion(3);	  //whiteline Left
  ADC_Data[4] = AD0_Conversion(1);	  //whiteline Center
  ADC_Data[6] = AD0_Conversion(2);	  //whiteline Right
   
  if(Front_Distance<130)
  {
   Forward();
   UpdateVelocity(1,1);
   BUZZER_ON();
  }
  else if(ADC_Data[4]<40)
  {
   Forward();
   UpdateVelocity(300,300);
   BUZZER_OFF();
  }
  else if(ADC_Data[3]<40)
  {
   Forward();
   UpdateVelocity(125,350);
   BUZZER_OFF();
  }
  else if(ADC_Data[6]<40)
  {
   Forward();
   UpdateVelocity(350,125);
   BUZZER_OFF();
  }
  else
  {
   Forward();
   UpdateVelocity(1,1);
   BUZZER_OFF();
  }

  Temp = Batt_Voltage_Conversion(ADC_Data[5]);
  LCD_Print(1,5,Front_Distance,3);
  LCD_Print(1,13,Temp,4);
  LCD_Print(2,1,ADC_Data[3],3);
  LCD_Print(2,7,ADC_Data[4],3);
  LCD_Print(2,13,ADC_Data[6],3);
  DelaymSec(100);
 }
 
}
