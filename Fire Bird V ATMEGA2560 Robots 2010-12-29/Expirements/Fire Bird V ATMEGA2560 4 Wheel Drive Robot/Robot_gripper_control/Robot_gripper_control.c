/********************************************************************************
 Written by: Sachitanand Malewar NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 20th December 2010
 
 This application example demonstrates operation of two servo motor based small gripper which is fitted in front of 
 Fire Bird V ATMEGA2560 4 Wheel Drive Robot. In this application example robot will move forward, grip an object, 
 lift it up and move to other place and put it there.


 Concepts covered: Servo motor control, robot motion control

 Servo control:
  Fire Bird V ATMEGA2560 microcontroller board has connection for 3 servo motors (S1, S2, S3).
 Servo motors move between 0 to 180 degrees proportional to the pulse train 
 with the on time of 1 to 2 ms with the frequency between 40 to 60 Hz. 50Hz is most recommended.

 We are using Timer 1 at 10 bit fast PWM mode to generate servo control waveform.
 In this mode servo motors can be controlled with the angular resolution of 2.25 degrees.
 Although angular resolution is less this is very simple method.
 
 There are better ways to produce very high resolution PWM but it involves interrupts at the frequency of the PWM.
 High resolution PWM is used for servo control in the Hexapod robot.
  
 Connection Details:	PORTB 5 OC1A --> Servo 1: Camera pod pan servo
 						PORTB 6 OC1B --> Servo 2: Grip servo
						PORTB 7 OC1C --> Servo 3: Robot arm up/down servo 

 Motion control:
 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 
  
 Connection Details:  	L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 


 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 2.20 in the Software Manual)

 2. 5V supply to these motors is provided by separate low drop voltage regulator "5V Servo" which can
 	supply maximum of 800mA current. It is a good practice to move one servo at a time to reduce power surge 
	in the robot's supply lines. Also preferably take ADC readings while servo motor is not moving or stopped
	moving after giving desired position.

 3. Look carefully at the functions object_grip(); and object_ungrip();
    After gripping and ungripping the object it is making grip servo free to reduce current consumption.
	Never ever allow grip and arm up down servo to stall for more than 10 to 15 seconds in order to avoid
	damage to the servo motor because of over heating.

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

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Initialize the ports
void port_init(void)
{
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
 servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
 motion_pin_config(); //Configure pins required for motion control
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 42.187Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:64
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:674.988Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}


void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set(0x00);
}



//Function to initialize all the peripherals
void init_devices(void)
{
 cli(); //disable all interrupts
 port_init();
 timer1_init();
 timer5_init();
 sei(); //re-enable interrupts 
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
 PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 

void object_grip (void) // grip the object
{
 servo_2 (35);
 _delay_ms(1000);
}

void object_ungrip (void) // relase the object
{
 servo_2 (90);
 _delay_ms(1000);

}

void arm_lift (void)
{
 servo_3 (40);
 _delay_ms(1000);
}

void arm_down (void)
{
 servo_3 (90);
 _delay_ms(1000);
}



//Main function
void main(void)
{
 unsigned char i = 0;
 init_devices();
 	
 
 
 while (1)
 { 
  
 arm_down();
 object_ungrip(); 
 
 forward();
 _delay_ms(2500);
 stop();

 object_grip();
 arm_lift();

 back();
 _delay_ms(500);
 stop();
 _delay_ms(250);

 left();
 _delay_ms(3900);
 stop();
 _delay_ms(250);

 forward();
 _delay_ms(2500);
 stop();

 arm_down();
 object_ungrip();

 back();
 _delay_ms(500);
 stop();
 _delay_ms(250);

 left();
 _delay_ms(3900);
 stop();
 _delay_ms(250);
 }

}
