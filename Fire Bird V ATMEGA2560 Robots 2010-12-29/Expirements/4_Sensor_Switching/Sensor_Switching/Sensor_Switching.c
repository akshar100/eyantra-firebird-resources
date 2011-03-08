/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010

 This experiment demostrates switching on and off of MOSFET switches to turn on and off the active part of the sensors.
  
 Connections:
 				PORTG 2 --> Power control of Sharp IR Range sensor 2, 3, 4 and red LEDs of the white line sensors
				PORTH 2 --> Power control of Sharp IR Range sensor 1, 5
				PORTH 3 --> Power control of IR Proximity sensors 1 to 8
 
 
 

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Microcontroller: atmega2560
 Frequency: 14745600
 Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Make sure that Jumper 2, 3, 4 are open on the main board

 3. Logic 1 to any of the pin turns off the power to the corresponding sensor.
 Logic 0 to any of the above pin turns on the power to the corresponding sensor.

 4. To use IR proximity sensor as directional light intensity sensor, turn off IR proximity
 sensor. It will turn off IR LEDs.

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

  Sotware relased under Creative Commance cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


//MOSFET switch port configuration
void MOSFET_switch_config (void)
{
 DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
 PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

 DDRG = DDRG | 0x04; //make PORTG 2 pin as output
 PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

//Function to Initialize PORTS
void port_init()
{
 MOSFET_switch_config();	
}

void turn_on_sharp234_wl (void) //turn on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
  PORTG = PORTG & 0xFB;
}

void turn_off_sharp234_wl (void) //turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
 PORTG = PORTG | 0x04;
}

void turn_on_sharp15 (void) //turn on Sharp IR range sensors 1,5
{
  PORTH = PORTH & 0xFB;
}

void turn_off_sharp15 (void) //turn off Sharp IR range sensors 1,5
{
 PORTH = PORTH | 0x04;
}

void turn_on_ir_proxi_sensors (void) //turn on IR Proximity sensors
{
 PORTH = PORTH & 0xF7;
}

void turn_off_ir_proxi_sensors (void) //turn off IR Proximity sensors
{
 PORTH = PORTH | 0x08;
}

void turn_on_all_proxy_sensors (void) // turn on Sharp 2, 3, 4, red LED of the white line sensors
									  // Sharp 1, 5 and IR proximity sensor
{
 PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0
 PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

void turn_off_all_proxy_sensors (void) // turn off Sharp 2, 3, 4, red LED of the white line sensors
									  // Sharp 1, 5 and IR proximity sensor
{
 PORTH = PORTH | 0x0C; //set PORTH 3 and PORTH 1 pins to 1
 PORTG = PORTG | 0x04; //set PORTG 2 pin to 1
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 sei(); //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	
	turn_off_all_proxy_sensors();
	_delay_ms(1500);

	turn_on_sharp234_wl();
	_delay_ms(1500);

	turn_off_sharp234_wl();
	turn_on_sharp15();
	_delay_ms(1500);

	turn_off_sharp15();
	turn_on_ir_proxi_sensors();
	_delay_ms(1500);
	
	turn_off_ir_proxi_sensors();
	_delay_ms(1500);

	turn_on_all_proxy_sensors();

	while (1);
}

