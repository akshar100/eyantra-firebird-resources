/**********************************************************************************************
 Platform: 6 Channel 2.4GHz XBee Remote Control
 Experiment: 6 Channel 2.4GHz XBee Remote Control firmware V1
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 26th Dec 2010
 AVR Studio Version 4.17, Build 666

 In this remote control application data packet as "NEXR" is received from master(Fire Bird-V Hexapod robot).
 Upon receiving the correct data packet, 2 byte header and 8-byte data packet containing information about 
 analog and digital logic states of the input devices is sent back at the baud rate of 115200 bps. 
 
 
 Receiving packet   : N - 1st byte of packet 
                      E - 2nd byte of packet
					  X - 3rd byte of packet
					  R - 4th byte of packet

 Transmitting packet: F     - 1st byte of packet ( 1st header byte )
                      B     - 2nd byte of packet ( 2nd header byte )	
					  Byte1 - 3rd byte of packet ( Vertical position of left joystick )  
					  Byte2 - 4th byte of packet ( horizontal position of left joystick )  
					  Byte3 - 5th byte of packet ( Vertical position of right joystick )  
					  Byte4 - 6th byte of packet ( horizontal position of right joystick )  
					  Byte5 - 7th byte of packet ( Accelerometer x axis position )  
					  Byte6 - 8th byte of packet ( Accelerometer y axis position )   
					  Byte7 - 9th byte of packet ( Battery Voltage)   
					  Byte8 - 10th byte of packet( Right and left switch positions in bitwise format
					                               in 10 byte as bit0 shows right switch and bit1 shows
												   left switch position)   

 Battery Voltage Calculation:

 The Battery voltage is Derived from the voltage divider circuit consisting of  2.2K and 4.4K resistor and connected to ADC6 of the ATMEGA8.
 
 analog value is calculated using the following formula
  
 A = (Digital value  * K ) + 0.7V
 
 Where 
 A = Equivalent value to Battery Voltage 
 K = (3.3/255) * ((2.2+4.7)/2.2) = 0.042
 0.7V = Voltage drop due to diode connected in series with the battrey voltage applied.
 
 UART is configured at baud rate is 115200.

 -----------------------------------------------------------------------------------------------
 
 connection details:         
  
 remote keys								      Atmega8
 
 left joystcik analog verctial position			    ADC2
 left joystcik  analog horizontal position    		ADC3
 right joystcik  analog verctial position			ADC1
 right joystcik  analog horizontal position    		ADC0
 Accelerometer y axis  analog position     		    ADC4
 Accelerometer x axis  analog position     			ADC5
 Battery Voltage		    						ADC6
 Spare ADC											ADC7

 Right switch (input)                              PORTB 0
 Left switch (input)                               PORTB 1

 Buzzer (Output)                                     PD2

------------------------------------------------------------------------------------------------- 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega8
 	Frequency: 7372800Hz
 	Optimization: -O0 

*************************************************************************************************/

/*************************************************************************************************

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

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>

#define		BATTERY_TRHRESHOLD		8        // Battery Low threshold value

unsigned char packet_valid;                  // to indicate complete packet received

unsigned char left_joystick_vertical_position = 0x00;         //Byte1
unsigned char left_joystick_horizontal_position = 0x00;       //Byte2
unsigned char right_joystick_verical_position = 0x00;         //Byte3
unsigned char right_joystick_horizontal_position = 0x00;      //Byte4
unsigned char Accelero_x_axis_position = 0x00;                //Byte5
unsigned char Accelero_y_axis_position = 0x00;                //Byte6
unsigned char battery_voltage = 0x00;                         //Byte7
unsigned char switchs_position = 0x00;                        //Byte8

unsigned char flag1 = 0;
unsigned char communication_proper_count = 0;
unsigned char count = 0;
unsigned char flag2 = 0;
//--------------------------------------------------------------------------------
// function to initialize Buzzer for debugging
//--------------------------------------------------------------------------------
void buzzer_pin_config (void)
{
 DDRD = DDRD | 0x04;   //Setting PORTD 2 as output
 PORTD = PORTD & 0xFB; //Setting PORTD 2 logic low to turnoff buzzer
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PIND;
 port_restore = port_restore | 0x04;
 PORTD = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PIND;
 port_restore = port_restore & 0xFB;
 PORTD = port_restore;
}

//--------------------------------------------------------------------------------
// UART initialisation
// desired baud rate: 115200
// actual: baud rate:115200 (0.0%)
// char size: 8 bit
// parity: Disabled
//--------------------------------------------------------------------------------
void uart_init(void)
{
 UCSRB = 0x00; //disable while setting baud rate
 UCSRA = 0x00;
 UCSRC = 0x86;
 UBRRL = 0x03; //set baud rate lo  //2F is for 7.3728MHz 9600 baudrate
 UBRRH = 0x00; //set baud rate hi
 UCSRB = 0x98; 
}

//--------------------------------------------------------------------------------
// atmega8 ADC initialisation
// Conversion time: 225uS (Prescale : 128)
//--------------------------------------------------------------------------------
void adc_init(void)
{
 ADCSRA = 0x00; //disable ADC
 ACSR  = 0x80;
 ADCSRA = 0x87; //Enable ADC
}

//--------------------------------------------------------------------------------
//TIMER1 initialize - prescale:256
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 1Hz
// actual value:  1.000Hz (0.0%)
//--------------------------------------------------------------------------------
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0x8F; //setup
 TCNT1L = 0x81;
 TCCR1A = 0x00;
 TCCR1B = 0x04; //start Timer
}

//--------------------------------------------------------------------------------
// Analog to digital convsersion 
//--------------------------------------------------------------------------------
unsigned char ADC_conversion()
{
 unsigned char adc_dataH=0;
  
 ADCSRA = ADCSRA | 0x40;       //SOC
 while((ADCSRA & 0x10)==0);    //Wait till conversion complete
 adc_dataH=ADCH;               //Higher Bytes
 return(adc_dataH);
}

//--------------------------------------------------------------------------------
// left and right switch position identification
//--------------------------------------------------------------------------------
unsigned char switch_pos()
{
 unsigned char switchs = 0;
 unsigned char temp;

//------------------------------------------------------------------------------
 {
  temp = PINB;						      //PB0 right switch
  temp = temp & 0x01;
  
  if (temp==0x01)
   {
     switchs = switchs | 0x01;
   }
  else
   {
	 switchs = switchs & 0xFE;
   }
 } 
//------------------------------------------------------------------------------
 {
  temp = PINB;						      //PB1  left switch
  temp = temp & 0x02;
  
  if (temp==0x02)
   {
	 switchs = switchs | 0x02;
   }
  else
   {
	 switchs = switchs & 0xFD;
   }
 } 
//------------------------------------------------------------------------------
 return(switchs);                   // return the switches position); 
}


//------------------------------------------------------------------------------
// Timer1 overflow isr is used to enable the know the battery status at every 10 sec.
// The uart0 is contineously receiving the data packet, which is disabled to know the
// battery status and enabled again after knowing the battery sataus 
//------------------------------------------------------------------------------
SIGNAL(SIG_OVERFLOW1) 
{
 //TIMER1 has overflowed
 TCNT1H = 0x8F; //reload counter high value
 TCNT1L = 0x81; //reload counter low value

 count++;
 if(count > 10)
  {
    flag2 = 1;
    UCSRB = 0x00; //disable communication when calculating battery volatge for balttery low indication
	count = 0;
  }
}

//--------------------------------------------------------------------------------
// uart isr for receive the command packet and send the 8 byte data packet
//--------------------------------------------------------------------------------
SIGNAL(SIG_UART_RECV) 
{
 unsigned char ser_data = 0x00;
 static unsigned char packet_data_count = 1;   // as packet byte counter

 ser_data = UDR;                         //uart has received a character in UDR

 // confirm the packet received
 if(packet_data_count == 4)              // 4th byte of packet
   {
     if(ser_data == 0x52)                // is 2nd byte is 'R'
	   {
	     packet_data_count = 5;          // allow for receving 3rd byte 
	     packet_valid = 1;
       }
     else 
	   {
	     packet_data_count = 1;          // allow for new packet to receive i.e 1st byte
       }   
   }  

 if(packet_data_count == 3)              // 3rd byte of packet
   {
     if(ser_data == 0x58)                // is 2nd byte is 'X'
	   {
	     packet_data_count = 4;          // allow for receving 3rd byte  
	   }
     else 
	   {
	     packet_data_count = 1;          // allow for new packet to receive i.e 1st byte		 
	   }                 // allow for receving 4th byte        
   }

 if(packet_data_count == 2)              // 2nd byte of packet
   {
     if(ser_data == 0x45)                // is 2nd byte is 'E'
	   {
	     packet_data_count = 3;          // allow for receving 3rd byte 		 
	   }
     else 
	   {
	     packet_data_count = 1;          // allow for new packet to receive i.e 1st byte		 
	   }   
   }

 if(packet_data_count == 1)              // 1st byte of packet
   {
     if(ser_data == 0x4E)                // is 1st byte is 'N'
	   {
	     packet_data_count = 2;          // allow for receving 2nd byte 
	   }
   }
  
 if(packet_valid)                        // is complete packet Rx'vd is ok 
  {  
     // Right joystick horizontal postion
     ADMUX = 0x20; //select ADC input 0 
     right_joystick_horizontal_position = ADC_conversion();
	 
	 // Right joystick vertical postion
	 ADMUX = 0x21; //select ADC input 1
	 right_joystick_verical_position = ADC_conversion();
     
	 // Left joystick vertical postion
	 ADMUX = 0x22; //select ADC input 2
     left_joystick_vertical_position = ADC_conversion();
	 
	 // Left joystick horizontal postion
     ADMUX = 0x23; //select ADC input 3
     left_joystick_horizontal_position = ADC_conversion();

	 // Accelerometer y axis postion
     ADMUX = 0x24; //select ADC input 4
     Accelero_y_axis_position = ADC_conversion();

	 // Accelerometer x axis postion
     ADMUX = 0x25; //select ADC input 5
     Accelero_x_axis_position = ADC_conversion();

	 // Battery Voltage
     ADMUX = 0x26; //select ADC input 6
     battery_voltage = ADC_conversion();

	 // Left and right switches position
	 switchs_position = switch_pos();                                
	              
    UDR = 0x46;                                    //1-Header 1st Byte as 'F'    
    while(!(UCSRA & (1<<UDRE)));
    UDR = 0x42;                                    //2-Header 2nd Byte as 'B'
    while(!(UCSRA & (1<<UDRE)));
    UDR = left_joystick_vertical_position;         //3-Byte1
    while(!(UCSRA & (1<<UDRE)));
    UDR = left_joystick_horizontal_position;       //4-Byte2
    while(!(UCSRA & (1<<UDRE)));
    UDR = right_joystick_verical_position;         //5-Byte3
    while(!(UCSRA & (1<<UDRE)));
    UDR = right_joystick_horizontal_position;      //6-Byte4
    while(!(UCSRA & (1<<UDRE)));
    UDR = Accelero_x_axis_position;                //7-Byte5
    while(!(UCSRA & (1<<UDRE)));
    UDR = Accelero_y_axis_position;                //8-Byte6
    while(!(UCSRA & (1<<UDRE)));
    UDR = battery_voltage;                         //9-Byte7
    while(!(UCSRA & (1<<UDRE)));
    UDR = switchs_position;                        //10-Byte8
    while(!(UCSRA & (1<<UDRE)));

 	packet_valid = 0;	
	packet_data_count = 1;                         // allow for receving New packet
    communication_proper_count++;
  }
}   //ISR

//--------------------------------------------------------------------------------
//call this routine to initialise all peripherals
//--------------------------------------------------------------------------------
void init_devices(void)
{
 //stop errant interrupts until set up
 cli();           //disable all interrupts
 
 buzzer_pin_config ();
 uart_init();
 adc_init();
 timer1_init();

 TIMSK = 0x04;    //timer interrupt sources

 sei();           //enable all interrupts
}

//--------------------------------------------------------------------------------
// start the main function here
//--------------------------------------------------------------------------------
int main(void)
{
 float battery_low = 0;

 init_devices();
  
 packet_valid = 0;// initialise valid packet variable to 0
 
 // initially calculate the battery voltage 
 UCSRB = 0x00; 
 ADMUX = 0x26; //select ADC input 6
 battery_voltage = ADC_conversion(); 
 UCSRB = 0x98; 
 flag2 = 0;

 while(1)
 {
   // flag2 will rise once in 10sec and calculates the Battery voltage
   // and enables the UART communication
   if(flag2 == 1)
    {  	
	  ADMUX = 0x26;                           //select ADC input 6
      battery_voltage = ADC_conversion();  
      UCSRB = 0x98;                           
      flag2 = 0;
    }

   // if battery voltage is less then battery threshold value ,buzzer beeps
   battery_low = (battery_voltage * 0.041) + 0.7;  // battery voltage caluculation
   if(battery_low < BATTERY_TRHRESHOLD)
   { buzzer_on();  _delay_ms(200); buzzer_off();_delay_ms(200);}
  
   // following function beeps the buzzer twice after sucessfully reciving 10 packets
   // from the master(FireBird-V Hexapod robot)
   if(flag1 == 0)
   { 
    if(communication_proper_count > 10)
     { 
	   flag1 = 1;
	   buzzer_on();  _delay_ms(400); buzzer_off();_delay_ms(400);
	   buzzer_on();  _delay_ms(400); buzzer_off();_delay_ms(400);
     }  
   }

 }

}
//--------------------------------------------------------------------------------

