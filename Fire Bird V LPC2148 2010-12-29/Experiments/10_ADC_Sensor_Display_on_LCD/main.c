/********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe

 Date: 1st OCT 2010
 
 In this experiment ADC captures the analog sensor values and displayes it on the LCD

 Concepts covered:  ADC, LCD interfacing

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
 *BATTERY VOLTAGE	IR PROX.SENSOR 2	FRONT SHARP 2 DIS 	IR.PROX.SENSOR 4*
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


void Init_Ports(void)
{
 Init_LCD_Pin();
 Init_Buzzer_Pin();
 Init_ADC_Pin();
}

void Init_Peripherals(void)
{
 Init_Ports();
 Init_ADC0();
 Init_ADC1();
}


int main(void)
{ 
 unsigned int Temp=0; 
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
  ADC_Data[0] = AD0_Conversion(6);	  //Left IR
  LCD_Print(1,1,ADC_Data[0],3);

  ADC_Data[1] = AD1_Conversion(0);	  //Front Sharp
  Temp = Sharp_GP2D12_Estimation(ADC_Data[1]);
  LCD_Print(1,5,Temp,3);

  ADC_Data[2] = AD0_Conversion(7);	  //Right IR
  LCD_Print(1,9,ADC_Data[2],3);

  ADC_Data[5] = AD1_Conversion(4);	  //Batt
  Temp = Batt_Voltage_Conversion(ADC_Data[5]);
  LCD_Print(1,13,Temp,4);

  ADC_Data[3] = AD1_Conversion(3);	  //whiteline Left
  LCD_Print(2,1,ADC_Data[3],3);

  ADC_Data[4] = AD0_Conversion(1);	  //whiteline Center
  LCD_Print(2,7,ADC_Data[4],3);

  ADC_Data[6] = AD0_Conversion(2);	  //whiteline Right
  LCD_Print(2,13,ADC_Data[6],3);

  DelaymSec(100);
 }
 
}
