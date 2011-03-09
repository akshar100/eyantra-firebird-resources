/********************************************************************************
 Written by: Rohit Chauhan, NEX Robotics Pvt. Ltd.
 
 IDE: Keil uVision4
 Tool Chain: Realview MDK_ARM Ver 4.10
 C Compiler: ARMCC.exe

 Date: 1st OCT 2010
 
 This experiment demonstrates LCD interfacing in 4 bit mode

 Concepts covered:  LCD interfacing

 LCD Connections:
 			 LCD	  Microcontroller Pins
 			  RS  --> P1.19
			  RW  --> P1.18
			  EN  --> P1.17
			  DB7 --> P1.25
			  DB6 --> P1.24
			  DB5 --> P1.23
			  DB4 --> P1.22

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

#include  <lpc214x.h>

/*************Macros*******************/
#define DATA_PORT() IO1SET=(1<<19)	
#define READ_DATA() IO1SET=(1<<18)
#define EN_HI() IO1SET=(1<<17)

#define COMMAND_PORT() IO1CLR=(1<<19)	
#define WRITE_DATA() IO1CLR=(1<<18)
#define EN_LOW() IO1CLR=(1<<17)

#define BUZZER_OFF() IO0CLR=(1<<25)		   //Macro to turn OFF buzzer
#define BUZZER_ON() IO0SET=(1<<25)		   //Macro to turn ON buzzer

/**************************************/

/***********Prototypes****************/

void Init_LCD_Pin(void);
void DelaymSec(unsigned int j);
void LCD_Command(unsigned int data);
void LCD_4Bit_Mode(void);
void LCD_Init(void);
void LCD_String(unsigned char *data);
void LCD_Home(void);
void LCD_Cursor(unsigned char Row,unsigned char Col);
void LCD_Print(unsigned char Row, char Col,unsigned int Val, unsigned int Digits);
void Init_Peripherals(void);
void Init_Ports(void);
/**************************************/


/*************Global Variables*********/

unsigned char String1[16]={"FBV ARM7 LPC2148"};
unsigned char String2[16]={"  Nex Robotics  "};

/***************************************/



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

	Function 		: Init_LCD_Pin
	Return type		: None
	Parameters		: None
	Description 	: Initialises pins used for LCD interface
************************************************************/

void Init_LCD_Pin(void)
{
 IO1DIR&=0xFC31FFFF;
 IO1DIR|=(1<<25) | (1<<24) | (1<<23) | (1<<22) | (1<<19) | (1<<18) | (1<<17);		// Set P1.16, P1.17, P1.18, P1.19 as Output
}

//This Function provides delay of app. 1mSec
void DelaymSec(unsigned int j)		  //App 1mSec delay
{  
 unsigned int  i;
 for(;j>0;j--)
 {
  for(i=0; i<10000; i++);
 } 
}


//This function sends commands to LCD
void LCD_Command(unsigned int data)
{
 unsigned int temp=0;
 EN_LOW();
 COMMAND_PORT();
 WRITE_DATA();
 
 temp=data;
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp & 0xF0) << 18;

 EN_HI();
 DelaymSec(1);
 EN_LOW();
 
 temp=data & 0x0F;
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp) << 22;

 EN_HI();
 DelaymSec(1);
 EN_LOW();
 
} 

//This Functions switches LCD panel in 4-bit interfacing mode
void LCD_4Bit_Mode(void)
{
 unsigned int temp=0;
 
 EN_LOW();
 COMMAND_PORT();
 WRITE_DATA();
 
 temp=0x30;
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp & 0xF0) << 18;

 EN_HI();
 DelaymSec(5);
 EN_LOW();
 DelaymSec(5);

 temp=0x30;
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp & 0xF0) << 18;

 EN_HI();
 DelaymSec(5);
 EN_LOW();
 DelaymSec(5);

 temp=0x30;
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp & 0xF0) << 18;

 EN_HI();
 DelaymSec(5);
 EN_LOW();
 DelaymSec(5);

 temp=0x20;
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp & 0xF0) << 18;

 EN_HI();
 DelaymSec(5);
 EN_LOW();

 DelaymSec(5);
} 


//This function writes data on the LCD display
void LCD_Data(unsigned int data)
{
 unsigned int temp=0;
 EN_LOW();
 DATA_PORT();
 WRITE_DATA();
 
 temp=data;
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp & 0xF0) << 18;

 EN_HI();
 DelaymSec(1);
 EN_LOW();
 
 temp=data & 0x0F;
 
 IO1PIN&=0xFC3FFFFF;
 IO1PIN|=(temp) << 22;

 EN_HI();
 DelaymSec(1);
 EN_LOW();
}


//This function enables LCD's internal funcions
void LCD_Init(void)
{
 LCD_Command(0x28);
 LCD_Command(0x0C);
 LCD_Command(0x06);
}

//This function writes string on LCD display
void LCD_String(unsigned char *data)
{
 while(*data)
 {
  LCD_Data(*data);
  data++;
 } 
}  


//This Function puts LCD cursor to Home position
void LCD_Home(void)
{
 LCD_Command(0x80);
}

//This function sets cursor position
void LCD_Cursor(unsigned char Row,unsigned char Col)
{
 switch(Row)
 {
  case 1: LCD_Command(0x80+ Col -1);break;
  case 2: LCD_Command(0xC0+ Col -1);break;
  default: break;
 }
}


//This function displays any data upto 5 digits. It also requires row and column address
void LCD_Print(unsigned char Row, char Col,unsigned int Val, unsigned int Digits)
{
 unsigned char Flag=0;
 unsigned int Temp,Mi,Th,Hu,Te,Un=0;

 if(Row==0 || Col==0)
 {
  LCD_Home();
 }
 else
 {
  LCD_Cursor(Row,Col);
 }
 if(Digits==5 || Flag==1)
 {
  Mi=Val/10000+48;
  LCD_Data(Mi);
  Flag=1;
 }
 if(Digits==4 || Flag==1)
 {
  Temp = Val/1000;
  Th = (Temp % 10) + 48;
  LCD_Data(Th);
  Flag=1;
 }
 if(Digits==3 || Flag==1)
 {
  Temp = Val/100;
  Hu = (Temp%10) + 48;
  LCD_Data(Hu);
  Flag=1;
 }
 if(Digits==2 || Flag==1)
 {
  Temp = Val/10;
  Te = (Temp%10) + 48;
  LCD_Data(Te);
  Flag=1;
 }
 if(Digits==1 || Flag==1)
 {
  Un = (Val%10) + 48;
  LCD_Data(Un);
 }
 if(Digits>5)
 {
  LCD_Data('E');
 }
	
}

void Init_Ports(void)
{
 Init_LCD_Pin();
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
 DelaymSec(40);
 Init_Peripherals();
 LCD_4Bit_Mode();
 LCD_Init();
 LCD_Command(0x01);
 DelaymSec(20);
 LCD_Cursor(1,1);
 LCD_String(&String1[0]);
 LCD_Cursor(2,1);
 LCD_String(&String2[0]);
 while(1);	
}


