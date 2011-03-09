/**************************************************************************************************************
		Platform: Fire Bird V P89V51RD2
		lcd_array_disp.h
		Written by: Omkar Pradhan, NEX Robotics Pvt. Ltd.
		Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
		Last Modification: 2009-12-08
		Used for displaying unsigned char array of 8 elements on the LCD
		Compiled with: uVision3 V3.90; C Compiler: C51.Exe, V8.18
**************************************************************************************************************/	

//program specific declarations
sbit RS = P2^6;             //P2^6 is connected to the RS line of LCD 
sbit RW = P2^5; 			//P2^5 is connected to the RW line of LCD
sbit E = P2^4;				//P2^4 is connected to the EN line of LCD
sbit BUSY = P2^3;           //P2^3 is connected to the DB3 pin of LCD

unsigned char A=0;			//variable used in the function swap(unsigned char tempdata) 

// function for giving a delay of ms milliseconds
void delay_ms(unsigned int ms)
{
unsigned int i,j;

for(i=0;i<ms;i++)
for(j=0;j<53;j++);
}

 void ready(void) // to check if LCD has finished executing previous command and is ready to accept next one
 {
  bit readybit;
  unsigned char buzzer_status = 0;
  buzzer_status = P2 & 0x80; //storing original buzzer status

  E = 0;
  RS = 0;			 //  to select comand register
  RW = 1;			 // to select read mode

  while(readybit == 1)	 // keep checking MSB bit till busy goes low
  {
   E = 1;		 // high to low transition on E pin requird to clock in data
   P2 = 0x7F | buzzer_status;		 // to set this pin in read mode to read busy signal from MSB
   readybit = P2^3;
   E = 0;
  }
}

/************************************************************************

	Function: swap(unsigned char tempdata)
	
	Description: This function swaps the data which is passed to it 
				 in such away that higher nibble is pushed in place of
				 lower nibble because lower nibble of Lcd is being used for data transfer.
				 

************************************************************************/


unsigned char swap(unsigned char tempdata)
{
A = tempdata;
A = _cror_(A,4);	// this uses an inbuilt rotate function to get MSB's in LSB's
tempdata = A & 0x0F;
return tempdata;
}	


/*********************************************************

	Function: commandsend(unsigned char command)
  
    Description: This function sends data to LCD as command by
				 selecting the instruction register
**********************************************************/


void commandsend(unsigned char command)
{ 
unsigned char temp;
unsigned char buzzer_status = 0;
  
buzzer_status = P2 & 0x80; //storing original buzzer status

ready();       // checking for busy bit to send next nibble
temp = swap(command);
P2 = temp  | buzzer_status; // to get higher nibble
RW = 0;        // to select write mode
RS = 0;        // to select command register
E = 1;         // to get the enable high to low transition
E = 0;         // in order to gate in the data


P2 = (command & 0x0F)  | buzzer_status; // to get the lower nibble
RW = 0;
RS = 0;        // repeat again all above steps    

E = 1;
E = 0;
delay_ms(2);	   // delay of 2 ms
}


/*********************************************************

	Function: datasend(unsigned char command)
  
    Description: This function sends data to LCD as data by
				 selecting the data register
**********************************************************/
void datasend(unsigned char lcddata)
{
unsigned char temp;
unsigned char buzzer_status = 0;
  
buzzer_status = P2 & 0x80; //storing original buzzer status

temp = swap(lcddata);
ready();	// checkin gfor busy bit to send next nibble
P2 = temp | buzzer_status;  // to get higher nibble
RW = 0;	 // to select write mode
RS = 1;  // to select data register
         
E = 1;
E = 0;


P2 = (lcddata & 0x0F) | buzzer_status;
RW = 0;
RS = 1;
E = 1;
E = 0;
delay_ms(2);	   // delay of 2 ms
}

/***************************************************************************

	Function: void lcdprint();

	Description: This function converts digital data into ASCII and displays 
				 sensor values on LCD display
				

***************************************************************************/

void lcdprint(unsigned char data_array[])
{
unsigned char data_array1[16]; //Array of 16 characters for Lcd line1
unsigned char data_array2[16]; //Array of 16 characters for Lcd line2
unsigned char temp=0;
unsigned char j=0;
unsigned char k=0;
 
 for (j=0;j<8;j++) 
{  
  temp=data_array[j];
  
  if(k > 15)//if this condition is true loop arranges data for display onto the 2nd line
  {
  data_array2[k - 16] = (temp / 100) + 48;           //Hundreds 
  data_array2[k - 15] = temp /10;
  data_array2[k - 15] = (data_array2[k - 15] % 10) + 48;	//Tens
  data_array2[k - 14] = (temp % 10) + 48;	//units
  data_array2[k - 13] = ' ';
  k = k + 4;
  }
  else //if the above condition is false loop arranges data for display onto the 1st line
  {
  data_array1[k] = (temp / 100) + 48;           //Hundreds 
  data_array1[k + 1] = temp /10;
  data_array1[k + 1]= (data_array1[k + 1] % 10) + 48;	//Tens
  data_array1[k + 2] = (temp % 10) + 48;	//units
  data_array1[k + 3] = ' ';
  k = k + 4;
  }
}

commandsend(0x80);  //Set cursor position to first line first character
for (j=0;j<k;j++)//j keeps track of the cursor position on the LCD screen
{
	if (j==16) //when j equals 16 it means cursor has to be shifted to 2nd line
	commandsend (0xC0);//when end of 1st line is reached cursor is shifted to first position on 2nd line

	if (j>15)//as long as j is greater than 15 it means that data has to be displayed on the 2nd line
	datasend(data_array2[j-16]);//dispaly data onto 2nd line of LCD
	else
	datasend(data_array1[j]);//display data onto 1st line of LCD
}


commandsend (0x80);
} 

/***********************************************************************
 Function: lcd_init(void) 
 Description: This function initialises LCD display in 4-bit mode.
              The intialisation functions are 8-bit wide, so we write
			  each inst into two seperate nibbles.
			  Following instructions are used for Lcd init.
			  1.>  0x30 8-bit mode
			  2.>  0x28 4-bit mode and 5x8 dot character font
 			  3.>  0x0E Turn ON lcd and cursor
			  4.>  0x06 Autoincrement cursor position
			  5.>  0x01 Clear Lcd display	
			  6.>  0x80 for setting cursor position
************************************************************************/

void lcd_init(void)
{
unsigned char buzzer_status = 0;

buzzer_status = P2 & 0x80; //storing original buzzer status
						
RS = 0;	 //  to select comand register
RW = 0;	// to select write mode
delay_ms(40);	   // on power ON we must allow a delay of 40ms for VCC to settle
 
E = 0;
P2 = 0x03 | buzzer_status; // send instruction to set FUNCTION SET to 8 bit mode though we have only connected 4 bit data bus because by default on power ON LCD is in 8 bit mode
E =  1;
E = 0;

delay_ms(4);	   // delay of 4ms
  
P2 = 0x03 | buzzer_status;	   // send the same instruction two times again. This way the LCD controller knows that we are operating in 4 bit mode
E =  1;
E = 0;
delay_ms(2);	   // delay of  2ms
P2 = 0x03 | buzzer_status;
E =  1;
E = 0;
delay_ms(2); // delay of  2ms

P2 = 0x02 | buzzer_status;		 // send FUNCTION SET instruction for 4 bit mode
E =  1;
E = 0;
ready();
P2 = 0x02 | buzzer_status; // again we send FUNCTION SET higher bits to keep 4 bit mode operation but main aim is to set the parameters in lower bits

E =  1;
E = 0;
P2 = 0x08 | buzzer_status; // these are the lower bits of FUNCTION SET. They set LCD to  1 display line and set font size to 5 x 8 dots

E =  1;
E = 0;
ready();
P2 = 0x00 | buzzer_status; // higher bits of DISPLAY OFF instruction
E =  1;
E = 0;
P2 = 0x08 | buzzer_status;  // lower bits of DISPLAY OFF instruction
E =  1;
E = 0;
ready();
P2 = 0x00 | buzzer_status;  // higher bits of CLEAR instruction
E =  1;
E = 0;
P2 = 0x01 | buzzer_status; // lower bits of CLEAR instruction
E =  1;
E = 0;
ready();
P2 = 0x00 | buzzer_status;// higher bits of ENTRY MODE instruction
E =  1;
E = 0;

P2 = 0x06 | buzzer_status;// lower bits of ENTRY MODE instruction to set increment i.e display to right side mode and no shift
E =  1;
E = 0;
ready();

commandsend(0x0F); // set LCD ON; CURSOR OFF; BLINK ON.
commandsend(0x01); // clear display.
commandsend(0x06); // to set cursor auto increment
commandsend(0x80); //Set cursor position to first line first character
}
