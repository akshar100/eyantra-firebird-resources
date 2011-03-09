#include <lpc214x.h>
#include "LCD.h"

/*************Global Variables*********/

unsigned char String1[16]={"FBV ARM7 LPC2148"};
unsigned char String2[16]={"  Nex Robotics  "};

/***************************************/


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

