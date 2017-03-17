/***************************************************************************
 *
 * Team Id: 	eYRC+#1575
 * Author List: Chirag Bafna, Mohit Khatri, Pranjali Borker, Namrata Sampat
 * Filename: 	eYRC+#1575_task6_atmel
 * Theme: 		Caretaker Robot
 * 

 Concepts covered:  serial communication and robot velocity control using PWM.

Use of timer to generate PWM for velocity control
Serial Port used: UART0

There are two components to the motion control:
1. Direction control using pins PORTA0 to PORTA3
2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

   
 Connection Details:  	L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 




In this experiment for the simplicity PL3 and PL4 are kept at logic 1.

Pins for PWM are kept at logic 1.

Connection Details:

Motion control:		L-1---->PA0;		L-2---->PA1;
R-1---->PA2;		R-2---->PA3;
PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1;


Serial Communication:	PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

PORTJ 0 --> RXD3 UART3 receive available on microcontroller expansion socket
PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expansion socket

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.

Commands:
Keyboard Key   ASCII value	Action
0				0x30	LED1 Red
1				0x31	LED1 Green
2				0x32	LED1 Blue
3				0x33	LED2 Red
4				0x34	LED2 Green
5				0x35	LED2 Blue
6				0x36	Stop
7				0x37	Buzzer on
8				0x38	Forward
9				0x39	Right (90 deg)

Note:

1. Make sure that in the configuration options following settings are
done for proper operation of the code

Microcontroller: atmega2560
Frequency: 14745600
Optimization: -O0 (For more information read section: Selecting proper optimization
options below figure 2.22 in the Software Manual)

2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
Rest of the things are the same.

3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

4. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to
2 Ampere. When both motors of the robot changes direction suddenly without stopping,
it produces large current surge. When robot is powered by Auxiliary power which can supply
only 1 Ampere of current, sudden direction change in both the motors will cause current
surge which can reset the microcontroller because of sudden fall in voltage.
It is a good practice to stop the motors for at least 0.5seconds before changing
the direction. This will also increase the useable time of the fully charged battery.
the life of the motor.

*********************************************************************************/
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

//Function to initialize ports
void init_ports()
{
 motion_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
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

unsigned char data; //to store received data from UDR1

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turn off buzzer
}

void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
	buzzer_pin_config();
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

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}


void led_pin_config (void)
{
	DDRJ = 0xFF;
	PORTJ = 0x00;
}

void red_on (void)
{
	PORTJ = 0x10;
}

void off (void)
{
	PORTJ = PORTJ & 0xF0;
}

void off_1 (void)
{
	PORTJ = PORTJ & 0x0F;
}
void off_2 (void)
{
	PORTA = 0x00;
}
void blue_on (void)
{
	PORTJ = 0x40;
}

void green_on (void)
{
	PORTJ = 0x80;
}

void red_on_1 (void)
{
	PORTJ = 0x01;
}

void blue_on_1 (void)
{
	PORTJ = 0x04;
}

void green_on_1 (void)
{
	PORTJ = 0x08;
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	UDR0 = data;				//echo data back to PC

	if(data == 0x38) //ASCII value of 8
	{
		PORTA=0x06;  //forward
	}

	//if(data == 0x32) //ASCII value of 2
	//{
		//PORTA=0x09; //back
//	}

	//if(data == 0x39) //ASCII value of 4
	//{
		//PORTA=0x05;  //left
	//}

	if(data == 0x39) //ASCII value of 6
	{
		PORTA=0x0A; //right
	}

	if(data == 0x36) //ASCII value of 5
	{
		PORTA=0x00; //stop
		PORTJ=0x00;
	}

	if(data == 0x37) //ASCII value of 7
	{
		buzzer_on();
	}

	//if(data == 0x39) //ASCII value of 9
	//{
		//buzzer_off();
		//PORTA=0x00;
	//}
}


//Function To Initialize all The Devices
void init_devices()
{
	cli(); //Clears the global interrupts
	port_init();  //Initializes all the ports
	uart0_init(); //Initailize UART1 for serial communiaction
	init_ports();
	timer5_init();

	sei();   //Enables the global interrupts
}
//Main Function
int main(void)
{
	init_devices();
	led_pin_config ();
	velocity (255, 255); //Set robot velocity here. Smaller the value lesser will be the velocity
	//Try different value between 0 to 255
	
	while(1)
	{
		//LED1
		if(data == 0x30)
		{
			red_on();
		}
		if(data == 0x31)
		{
			green_on();
		}
		if(data == 0x32)
		{
			blue_on();
		}	
		//LED2
		if(data == 0x33)
		{
			red_on_1();
		}
		if(data == 0x34)
		{
			green_on_1();
		}
		if(data == 0x35)
		{
			blue_on_1();
		}		
		
	}
}