/********************************************************************************
 Written by: Sachitanand Malewar NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates Insect walking using Servo motor control in 10 bit fast 
 PWM mode using Timer 1.

 Concepts covered:  Use of timer to generate PWM for servo motor control, Simple insect motion using servo motors

 Fire Bird V ATMEGA2560 microcontroller board has connection for 3 servo motors (S1, S2, S3).
 Servo motors move between 0 to 180 degrees proportional to the pulse train 
 with the on time of 1 to 2 ms with the frequency between 40 to 60 Hz. 50Hz is most recommended.

 We are using Timer 1 at 10 bit fast PWM mode to generate servo control waveform.
 In this mode servo motors can be controlled with the angular resolution of 1.86 degrees.
 Although angular resolution is less this is very simple method.
 
 There are better ways to produce very high resolution PWM but it involves interrupts at the frequency of the PWM.
 High resolution PWM is used for servo control in the Hexapod robot.
  
 Connection Details:	PORTB 5 OC1A --> Servo 1: Left leg Servo motor
 						PORTB 6 OC1B --> Servo 2: Right leg Servo motor
						PORTB 7 OC1C --> Servo 3: Center leg Servo motor
 					  	


 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)
 
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

#define step_time 250 // defines speed of the robot
//set at 1000 to observe motion
//500 is optimal. 250 is fast. Robot will age faster at 250.
#define robot_stand_angle_left_servo 90 //left leg servo zero calibration
#define robot_stand_angle_right_servo 90 //right leg servo zero calibration
#define robot_stand_angle_centere_servo 90 //centere leg servo zero calibration

#define left_leg_forward_angle 120 //left leg forward angle
#define right_leg_forward_angle 60 //right leg forward angle

#define left_leg_back_angle 60 //left leg back angle
#define right_leg_back_angle 120 //right leg back angle

#define mid_leg_left_down_angle 75 //mid leg left side down angle
#define mid_leg_right_down_angle 105 //mid leg right side down angle


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

//Initialize the ports
void port_init(void)
{
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
 servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
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


//Function to initialize all the peripherals
void init_devices(void)
{
 cli(); //disable all interrupts
 port_init();
 timer1_init();
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

void all_leg_free (void) //makes all servo motors free to rotate while power is on.
						 //use this function to reduce power consumption and stress on the servo motors
						 //when robot is not moving
{
 void servo_1_free();
 void servo_2_free();
 void servo_3_free();
}

void stand_position (void) //Robot stands in its inital position. Use this function for calibration while 
							//assembling robot's leg
{
 servo_1(robot_stand_angle_left_servo);
 servo_2(robot_stand_angle_right_servo);
 servo_3(robot_stand_angle_centere_servo);
}

void left_leg_centere (void)
{
 servo_1(robot_stand_angle_left_servo);
}

void left_leg_forward (void)
{
 servo_1(left_leg_forward_angle);
}

void left_leg_back (void)
{
 servo_1(left_leg_back_angle);
}

void right_leg_centere (void)
{
 servo_2(robot_stand_angle_right_servo);
}

void right_leg_forward (void)
{
 servo_2(right_leg_forward_angle);
}

void right_leg_back (void)
{
 servo_2(right_leg_back_angle);
}

void mid_leg_centere (void)
{
 servo_3(robot_stand_angle_centere_servo);
}

void mid_leg_left_down (void)
{
 servo_3(mid_leg_left_down_angle);
}

void mid_leg_right_down (void)
{
 servo_3(mid_leg_right_down_angle);
}


void forward_init (void) //sets robot's legs in proper angle without skidding before starting forward walk
						 //if robot moves forward without calling this function it might drift a bit in left or right
						 //direction at the time of starting foreard motion
{
 stand_position();
 _delay_ms(250);

 mid_leg_right_down();
 _delay_ms(250);
 right_leg_back();
 _delay_ms(250);
}

void forward (void) //move forward
{
 mid_leg_left_down();
 _delay_ms(step_time);
 
 left_leg_forward();
 right_leg_back();
 _delay_ms(step_time);

 mid_leg_right_down();
 _delay_ms(step_time);
 
 left_leg_back();
 right_leg_forward();
 _delay_ms(step_time);
}

void back_init (void) //sets robot's legs in proper angle without skidding before starting back walk
					  //if robot moves back without calling this function it might drift a bit in left or right
					  //direction at the time of starting back motion
{
 stand_position();
 _delay_ms(250);

 mid_leg_right_down();
 _delay_ms(250);
 right_leg_forward();
 _delay_ms(250);
}

void back (void) //move back 
{
 mid_leg_left_down();
 _delay_ms(step_time);
 
 left_leg_back();
 right_leg_forward();
 _delay_ms(step_time);

 mid_leg_right_down();
 _delay_ms(step_time);
 
 left_leg_forward();
 right_leg_back();
 _delay_ms(step_time);
}

void left (void) //turn left by moving right pair of legs in the forward direction
{
 mid_leg_right_down();
 _delay_ms(step_time);

 right_leg_forward();
 _delay_ms(step_time);

 mid_leg_left_down();
 _delay_ms(step_time);

 right_leg_back();
 _delay_ms(step_time);
}

void right (void) //turn right by moving left pair of legs in the forward direction
{
 mid_leg_left_down();
 _delay_ms(step_time);

 left_leg_forward();
 _delay_ms(step_time);

 mid_leg_right_down();
 _delay_ms(step_time);

 left_leg_back();
 _delay_ms(step_time);
}

void left_back (void) //turn left by moving left pair of legs in the backward direction
{
 mid_leg_left_down();
 _delay_ms(step_time);

 left_leg_back();
 _delay_ms(step_time);

 mid_leg_right_down();
 _delay_ms(step_time);

 left_leg_forward();
 _delay_ms(step_time);
}

void right_back (void) //turn right by moving right pair of legs in the backward direction
{
 mid_leg_right_down();
 _delay_ms(step_time);

 right_leg_back();
 _delay_ms(step_time);

 mid_leg_left_down();
 _delay_ms(step_time);

 right_leg_forward();
 _delay_ms(step_time);
}

//Main function
void main(void)
{
 unsigned char i = 0;
 init_devices();

 stand_position();
 _delay_ms(3000);

 forward_init();

while(1)
{ 
 forward_init();
 for (i=0; i <5; i++)
 {
  forward();
 }

 back_init();
 for (i=0; i <5; i++)
 {
  back();
 }

 for (i=0; i <5; i++)
 {
  left();
 }

 for (i=0; i <5; i++)
 {
  right();
 }
 
}

}



