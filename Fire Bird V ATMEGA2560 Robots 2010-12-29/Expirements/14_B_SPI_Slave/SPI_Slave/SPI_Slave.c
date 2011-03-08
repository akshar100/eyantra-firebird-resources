/********************************************************************************
 Written by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This program is for ATMEGA8 (slave) microcontroller
 This program is the default firmware for the ATMEGA8 (slave) microcontroller
 This program demonstrates SPI communication between master (ATMEGA2560) and slave (ATMEGA8) microcontroller.
 LCD displays analog values of the IR Proximity sensors 6, 7, 8.

 Concepts covered:  SPI communication
 
 Fire Bird V ATMEGA2560 have ATMEGA2560 (master) and ATMEGA8 (slave) microcontrollers.
 These two microcontrollers are connected with the SPI bus. Main function of the slave microcontroller is to 
 take analog readings from the sensors and send it back to the ATMEGA2560 (master) microcontroller.
 To get ADC data from channel number 'n' of the ATMEGA8 (slave) microcontroller send 'n' as parameter in 
 the function "spi_master_tx_and_rx"

 Returned ADC data from the ATMEGA8 (slave) microcontroller for 
 parameter 'n' given to the function "spi_master_tx_and_rx"
			
			Number sent		ADC channel data
				0			Analog value of white line sensor 4 (if connected)
				1			Analog value of white line sensor 5 (if connected)	or from servo pod		
				2			Analog value of white line sensor 6 (if connected)
				3			Analog value of white line sensor 7 (if connected)	or from servo pod		
				4			Analog value of sensed current by ACS712 current sensor (if installed)
				5			Analog value of IR proximity sensor 6
				6			Analog value of IR proximity sensor 7
				7			Analog value of IR proximity sensor 8

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega8
 	Frequency: 8000000
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. ATMEGA2560 (master) and ATMEGA8 (slave) microcontrollers use SPI bus for ISP as well as 
 	for the communication between them. Before doing ISP we need to disconnect the SPI bus between
	these two microcontrollers. Remove tree jumpers marked by J4 on the ATMEGA2560 microcontroller
	adaptor board before doing ISP.
 
 3. Connect 3 jumpers marked by J4 to connect SPI bus between the microcontrollers.

 4. Do not pass value more than 7 to the function "spi_master_tx_and_rx" else it will give back random value

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

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned char data;
unsigned char ADC_Value;

//Port initialization
void port_init(void)
{
 PORTB = 0x10;
 DDRB  = 0x10;
 PORTC = 0x3F; 
 DDRC  = 0x00;
 PORTD = 0x00;
 DDRD  = 0xFF;
}

//SPI initialize
void spi_init(void)
{
 SPCR = 0x41; //setup SPI
 SPSR = 0x00; //setup SPI
 SPDR = 0x00;
}

//ADC initialize
// Conversion time: 52uS
void adc_init(void)
{
 ADCSR = 0x00; //disable adc
 ADMUX = 0x00; //select adc input 0
 ACSR  = 0x80;
 ADCSR = 0xC5;
}


//Function for ADC conversion
unsigned char ADC_Conversion(unsigned char channel)
{
	unsigned char adc_data = 0;

	channel = channel & 0x07;			  //Store only 3 LSB bits
	ADMUX= 0x20 | channel;				  //Select the ADC channel with left adjust select
	ADCSRA = ADCSRA | 0x40;			  	  //Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	adc_data = ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	return adc_data; //return the ADC value
}

//Function To Iitialize All The Peripherals
void init_devices(void)
{
 cli(); 			//disable all interrupts
 port_init();
 spi_init();
 adc_init();
 sei();				//re-enable interrupts
}


//Main Function
int main(void)
{
 init_devices();
 
 while(1)
 {
 while((SPSR & 0x80) == 0x00); //wait for data reception to complete
 data = SPDR;
 //_delay_ms(1);
 //data = data + 1;
 ADC_Value = ADC_Conversion(data);
 SPDR = ADC_Value;
 while((SPSR & 0x80) == 0x00); //wait for data transmission to complete
 }
}
