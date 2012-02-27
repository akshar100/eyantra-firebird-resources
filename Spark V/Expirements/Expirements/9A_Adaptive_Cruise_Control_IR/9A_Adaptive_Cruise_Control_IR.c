/********************************************************************************
 Platform: SPARK V
 Experiment: 9A_Adaptive_Cruise_Control_IR
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 22nd September 2010
 AVR Studio Version 4.17, Build 666
 
 Concepts covered:  ADC, LCD interfacing, motion control based on sensor data

 This experiment demostrates the application of a simple line follower robot with 
 front obstical detection. Robot follows a white line over a black backround and
 stops when any obstacle comes in front of the robot. Robot will also stop if it
 does not detects whiteline.   
 
 Algorithum:
 1. Check the front IR Proximity Sensor reading, if it is not within the set threshold
    value then ther is a obstical in robot's path, stop the robot.
 
 2. Set the robot direction in the forward
 
 3. Read the Middle white-line sensor value, if it is in the threshold limit,set 
    the robot to maximum velocity.

 4. If the middle white-line sensor is not in threshold limit,and check the left whiteline
    sensor is not in the threshold limit then set the robot left motor with maximum set
	velocity and right motor with its minimum set velocity so that robot heads back 
	towards white line.

 5. If the middle white-line sensor is in threshold limit,and left white-line sensor 
    also in threshold limit and right white-line sensor is not in the 
	threshold limit then set the robot left motor with minimum set
	velocity and right motor with its maximum set velocity so that robot heads back 
	towards white line.
	
 6.If all white-line sensors are not detecting the whiteline then stop the robot.
       
 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  1			PA1		IR Proximity analog sensor Centre
              3			PA3		White line sensor Left
			  4			PA4		White line sensor Centre
			  5			PA5		White line sensor Right
  
 There are two components to the motion control:
 1. Direction control using pins PORTB0 to 	PORTB3
 2. Velocity control by PWM on pins PD4 and PD5 using OC1BL and OC1AL of timer 1.
                    	L-1---->PB0;		L-2---->PB1;
   						R-1---->PB2;		R-2---->PB3;
   						PD4 (OC1BL) ---->PWM left; 	PD5 (OC1AL) ----> PWM right; 

 LCD Display interpretation:
 ****************************************************************************************
 *LEFT WL SENSOR	   CENTRE WL SENSOR	       RIGHT WL SENSOR	   IR PROX.SENSOR CENTER*
 *LEFT MOTOR VELOCITY  RIGHTT MOTOR VELOCITY  									        *
 ****************************************************************************************
  		 
 For more detail on hardware and theory refer the hardware manual. 

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 4.22 in the hardware manual)

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
#include "lcd.c"

#define		IR_THRESHOLD	230
#define		WL_THRESHOLD	180
#define		VELOCITY_MAX	22
#define		VELOCITY_MIN	17
#define 	VELOCITY_LOW	0

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag1 = 0;
unsigned char flag2 = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char Front_ultrasonic_Sensor=0;
unsigned char Front_IR_Sensor=0;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;    //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0;  //set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;    //Setting PD5 and PD4 pins as output for PWM generation
 PORTD = PORTD | 0x30;  //PD5 and PD4 pins are for velocity control using PWM
}

//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;	//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;	//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();		
 motion_pin_config();
 buzzer_pin_config(); 
}

//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFF; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D; //start Timer
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
 ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
 while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 3);
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void stop (void) //hard stop
{
  motion_set(0x00);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = left_motor; 
 OCR1BH = 0x00;
 OCR1BL = right_motor;
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

void init_devices (void)
{
 cli();          //Clears the global interrupts
 port_init();
 timer1_init();
 adc_init();
 sei();          //Enables the global interrupts
}

//Main Function
int main(void)
{
 init_devices();

 lcd_set_4bit();
 lcd_init();

 velocity(VELOCITY_MAX,VELOCITY_MAX);    // Set the speed to max velocity
 forward();                              // start to move froward

 while(1)
 {
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(5);	//Getting data of Right WL Sensor
	Front_IR_Sensor = ADC_Conversion(1);    //Getting data of Center IR Proximity Sensor Sensor

	print_sensor(1,1,3);		//Prints value of White Line Sensor Left
	print_sensor(1,5,4);		//Prints value of White Line Sensor Center
	print_sensor(1,9,5);		//Prints value of White Line Sensor Right
	print_sensor(1,13,1);		//Prints value of Analog IR Proximity Sensor Center

	flag1 = 0;
	flag2 = 0;

	if(Front_IR_Sensor<IR_THRESHOLD)                    // Is there any obstacle at front
	{
		flag2=1;                                        
		velocity(VELOCITY_LOW,VELOCITY_LOW);            // stop the robot if obstacle is detected
		buzzer_on();                                    // indicate with buzzer
	}

	if((Center_white_line<WL_THRESHOLD) && (flag2==0))  // Is middle White-line is within threshold limit
	{                                             
		flag1=1;
		buzzer_off();
		velocity(VELOCITY_MAX,VELOCITY_MAX);            // Run robot at max velocity 
		lcd_print (2,1,VELOCITY_MAX,3);
		lcd_print (2,5,VELOCITY_MAX,3);
	}

	if((Left_white_line>WL_THRESHOLD) && (flag1==0) && (flag2==0))  // Is left White-line is not within threshold limit
	{                                             
		flag1=1;    
		buzzer_off();                                                   
		velocity(VELOCITY_MAX,VELOCITY_MIN);      // Run robot left wheel at max velocity and right wheel at min velocity
		lcd_print (2,1,VELOCITY_MAX,3);            
		lcd_print (2,5,VELOCITY_MIN,3);
	}

	if((Right_white_line>WL_THRESHOLD) && (flag1==0) && (flag2==0)) // Is right White-line is not within threshold limit
	{
		flag1=1;
		buzzer_off();
		velocity(VELOCITY_MIN,VELOCITY_MAX);      // Run robot right wheel at max velocity and left wheel at min velocity
		lcd_print (2,1,VELOCITY_MIN,3);           
		lcd_print (2,5,VELOCITY_MAX,3);
	}

	if(Center_white_line>WL_THRESHOLD && Left_white_line>WL_THRESHOLD && Right_white_line>WL_THRESHOLD && (flag2 == 0))
	                                              // if all White-line sensor are not within threshold limit    
	{
		flag1=1;
		buzzer_off();
		velocity(VELOCITY_LOW,VELOCITY_LOW);      // stop the robot
		lcd_print (2,1,VELOCITY_LOW,3);
		lcd_print (2,5,VELOCITY_LOW,3);
	}
 }
}
