/********************************************************************************
 Platform: SPARK V
 Experiment: 7B_ADC_URSWLBAT_Sensor_display_On_LCD
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 22nd September 2010
 AVR Studio Version 4.17, Build 666
 
 Concepts covered:  ADC, LCD. SENSOR interfacing

 In this experiment ADC captures data from the analog sensors such as Ultrasonic 
 Range Sensors, Whiteline sensors and Battery voltage and displayed it on the LCD.

 Ultrasonic sensors are used to detect proximity of any obstacles in the very short 
 range to long range from the robot. We are using EZ series of Ultrasonic Range Sensors 
 from Maxbotix. It can sense obsticals from from 6-inches to 254-inches with 1-inch 
 resolution. Output analog voltage has scaling factor of (Vcc/512) per inch. A supply 
 of 5V yields ~9.8mV/inch.

 White line sensors are used for detecting white line on the ground surface. White 
 lines are used to give robot sense of localization. White line sensor consists of 
 IR photo diode for line sensing and IRLED for illumination. White line cause increace 
 in reflected light resulting fall in voltage across the photodiode. 


 There are Three IR Proximity sensor, Three White Line Sensors on the SPARKV robot. 
 There are sockets for four Ultrasonic Range Sensors from Maxbotix. Due to limted 
 number of ADC channels either we can use IR Proximity sensors or Ultrasonic Range Sensors
 by jumper settings of J2,J3,J4 on the Robot. In this expirement three Ultrasonic Range
 Sensors are connected. You can also use less number of Ultrasonic Range Sensors. To do
 that you need to short some pads. For correct jumper setting and pad settings refer 
 to robot's hardware manual.

 
 IR Proximity Sensor interfacing is covered in "7A_ADC_IRWLBAT_Sensor_display_On_LCD"
 experiment.
  
 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PA0		Ultrasonic analog sensor Left
			  1			PA1		Ultrasonic analog sensor Center
			  2			PA2		Ultrasonic analog sensor Right
			  3			PA3		White line sensor Left
			  4			PA4		White line sensor Center
			  5			PA5		White line sensor Right
			  6			PA6		Battery Voltage
			  7         PA7     4th Ultrasonic Range Sensor Analog output is connected from 
			                    external connector (Refer Ultrasonic range sensor interfacing 
								in Manual)

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4


 Jumper settings:
 There are sockets for four Ultrasonic Range Sensors from Maxbotix. Due to 
 limted number of ADC channels either we can use IR Proximity sensors or 
 Ultrasonic Range Sensors by jumper settings of J2,J3,J4 on the Robot. For 
 correct jumper setting refer to robot's hardware manual.

 LCD Display interpretation:
 ***************************************************************************************************
 *ULTRASONIC SENSOR LEFT	ULTRASONIC SENSOR CENTER	ULTRASONIC SENSOR RIGHT                    *
 *LEFT WL SENSOR		    CENTER WL SENSOR	        RIGHT WL SENSOR		       BATTERY VOLTAGE *
 ***************************************************************************************************
 
 For more detail on hardware and theory refer the hardware manual. 

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800
 	Optimization: -O0 (For more information read section: Selecting proper optimization
	              options below figure 4.22 in the hardware manual)

 2. Make sure that you copy the lcd.c file in your folder

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
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned int BATT_Voltage;
unsigned char Left_ultrasonic_Sensor;
unsigned char Center_ultrasonic_Sensor;
unsigned char Right_ultrasonic_Sensor;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;  //set PORTF direction as input
 PORTA = 0x00; //set PORTF pins floating
}

void ultrsonic_trigger_config(void)
{
 DDRD = DDRD | 0x40;   //all the LCD pin's direction set as output
 PORTD = PORTD & 0x00; // all the LCD pins are set to logic 0 except PORTC 7
}

//Function to Initialize PORTs
void port_init()
{
 lcd_port_config();
 adc_pin_config();		
 ultrsonic_trigger_config();
}

//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;  			
 ADMUX= 0x20| Ch;	   		
 ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
 while((ADCSRA&0x10)==0);	    //Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;          //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 3);
}

// Ultrasonic sensor are connected in chaining mode. This function rise a 
// trigger pulse of >20usec to command ringing.     
void ultrasonic_trigger(void)                 
{
 PORTD = PORTD | 0x40;  // make high the Trigger input for Ultrasonic sensor
 _delay_us(50);         // Wait for >20usec
 PORTD = PORTD & 0xBF;  // make low the Trigger input for Ultrasonic sensor
}

void init_devices (void)
{
 cli();           //Clears the global interrupts
 port_init();
 adc_init();
 sei();           //Enables the global interrupts
}

//Main Function
int main(void)
{
 init_devices();

 lcd_set_4bit();
 lcd_init();

 while(1)
 {
	ultrasonic_trigger();      // call ultrasonic triggering after enery 150msec  
    _delay_ms(150);            
  
    Left_ultrasonic_Sensor = ADC_Conversion(0) * 2;         // In Inches
    Center_ultrasonic_Sensor = ADC_Conversion(1) * 2;       // In Inches
    Right_ultrasonic_Sensor = ADC_Conversion(2) * 2;        // In Inches
    
    lcd_print(1,1,Left_ultrasonic_Sensor,3);
	lcd_print(1,5,Center_ultrasonic_Sensor,3);
	lcd_print(1,9,Right_ultrasonic_Sensor,3);

	print_sensor(2,1,3);		//Prints value of White Line Sensor Left
	print_sensor(2,5,4);		//Prints value of White Line Sensor Center
	print_sensor(2,9,5);		//Prints Value of White Line Sensor Right

    //BATT_Voltage = (ADC_Conversion(6) * 0.03921) + 0.7;	//Prints Battery Voltage Status
    //lcd_print(2,13,BATT_Voltage,4);
    print_sensor(2,13,6);		//Prints Battery Voltage Status
 }
}
