/********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe
 
 This experiment demostrates switching on and off of MOSFET switches to turn on and off the active part of the sensors.
  
 Connections:
 				P1.16 --> Power control of Sharp IR Range sensor 2, 3, 4 and red LEDs of the white line sensors
				
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

 3. Make sure that Jumper J2 is not inserted on the main board. J2 bypasses software control

 4. Logic 1 on pin P1.16 turns OFF the power to the sensor.
 	Logic 0 om pin P1.16 turns ON the power to the  sensor.

 

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

  Sotware relased under Creative Commance cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include  <lpc214x.h>


/**********Macros******************/

#define SENSOR_OFF() IO1SET=(1<<16)		//Macro to turn OFF Sensors
#define SENSOR_ON() IO1CLR=(1<<16)		//Macro to turn ON Sensors 
										 
#define BUZZER_OFF() IO0CLR=(1<<25)		   //Macro to turn OFF buzzer
#define BUZZER_ON() IO0SET=(1<<25)		   //Macro to turn ON buzzer

/**********************************/


/****Function Prototypes**********/

void DelaymSec(unsigned int j);
void Init_Sensor_Switch_Pin(void);
void Init_Buzzer_Pin(void);
void Init_Peripherals(void);
void Init_Ports(void);
/**********************************/


void DelaymSec(unsigned int j)	 //Generates msec delay
{  
 unsigned int  i;
 for(;j>0;j--)
 {
  for(i=0; i<10000; i++);
 } 
}

/************************************************************

	Function 		: Init_Sensor_Switch_Pin
	Return type		: None
	Parameters		: None
	Description 	: Initialises Sensor switching pin
************************************************************/
                               
void Init_Sensor_Switch_Pin(void)
{
 IO1DIR&=0xFFFEFFFF;
 IO1DIR|=(1<<16);   //Set P1.16 direction as Output
 SENSOR_OFF();		   //Initially turn OFF Sensor
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

void Init_Ports(void)
{
 Init_Buzzer_Pin();
 Init_Sensor_Switch_Pin();
}

void Init_Peripherals(void)
{
 Init_Ports();
}


int main(void)
{  
 PINSEL0 = 0x00000000;		// Reset all pins as GPIO
 PINSEL1 = 0x00000000;
 PINSEL2 = 0x00000000;
 Init_Peripherals();		
 while(1)	
 { 
  SENSOR_ON();
  DelaymSec(1000);
  SENSOR_OFF();
  DelaymSec(1000);
 }
}
