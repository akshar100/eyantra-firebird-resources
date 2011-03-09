/********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe

 Date: 1st OCT 2010
 
 This experiment demonstrates use of timer match interrupt

 Concepts covered:  Use of timer match interrupt to do tasks in a peirodic way

 In this example timer 0's Match0 interrupt is used to turn on and off buzzer with the time peirod of 1 second
 
 Connections: Buzzer is cinnected to the P0.25.

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

#include <LPC214x.h>

/**********Macros******************/

#define BUZZER_OFF() IO0CLR=(1<<25)		   //Macro to turn OFF buzzer
#define BUZZER_ON() IO0SET=(1<<25)		   //Macro to turn ON buzzer


/****Function Prototypes**********/

void __irq IRQ_Timer0(void);
void Init_Buzzer_Pin(void);
void Init_Peripherals(void);
void Init_Ports(void);
void Timer0_Init(void);

/********************************/

/*********Global Variables*******/

unsigned char Temp=0;
unsigned char Toggle=0;

/********************************/


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
 IO0DIR = (1<<25);  		//Set P0.25 as Output
 BUZZER_OFF();				//Initially turn OFF buzzer
}

void Timer0_Init(void)
{
 T0TCR = 0x02;		//Stop Timer
 T0CTCR = 0x00;		//Select Timer Mode
 T0PR = 15000;		//Timer Clock is running at 1KHz PCLK/T0PR = 1Khz
 T0TC = 0;			//Reset Timer Counting register to 0
 T0PC = 0;			//Reset Prescale counter register to 0
 T0MR0 = 500;		//Match Register0 triggers interrupt at every 500mSec 1Khz/T0MRO=2Hz=500msec
 T0MR1 = 0;
 T0MR2 = 0;
 T0MR3 = 0;
 T0MCR = 0x0001;	//Start Timer

 VICIntSelect = 0x00000000;		// Setting Timer 0 interrupt as IRQ(Vectored)
 VICVectCntl0 = 0x20|4;		// Assigning Highest Priority Slot to Timer 0 and enabling this slot
 VICVectAddr0 = (int)IRQ_Timer0; // Storing vector address Timer0 Interrupt
 VICIntEnable = (1<<4);	// Enabling Timer 0 interrupt

 T0TCR = 0x01;  //Start Timer
}


//ISR for Timer 0 interrupt
void __irq IRQ_Timer0(void)
{ 
  Temp=T0IR;
  if(Temp & 0x01==0x01)			//Mat 0 Interrupt
  {
   Toggle=~Toggle;
   if(Toggle)
   {
    BUZZER_ON();
   }
   else
   {
    BUZZER_OFF();
   }
   T0TC=0;
   T0IR=0x01;
  } 
  VICVectAddr = 0x00;   		//Acknowledge Interrupt
}	
 
void Init_Ports(void)
{
 Init_Buzzer_Pin();
}

void Init_Peripherals(void)
{
 Init_Ports();
 Timer0_Init();
}


int main()
{
 PINSEL0 = 0x00000000; //Reset all pins in GPIO mode
 PINSEL1 = 0x00000000;
 PINSEL2 = 0x00000000;
 Init_Peripherals();
 while(1);
}

