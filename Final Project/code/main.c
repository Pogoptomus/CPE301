// this file is incomplete 
// currently does not cause motor to function
// still needs:
// adjust desired speed using input from usart
// timer to check encoder count and convert to speed
// compare found speed to desired speed via do_pid
// soft stop on usart command


#define F_CPU 8000000				// Clock Speed
#define BAUD 9600					// define BAUD
#define MYUBRR F_CPU/16/BAUD-1		// used to set UBRR0 to correct value

// ===========================INCLUDES===========================
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <util/atomic.h>
#include "ring_buffer.h"

// ===========================FUNCTION DECLARATIONS===========================
void encoder_update(encoder *e, int A, int B);
int do_pid(motor *m);
int encoder_velocity(encoder *e);
void USART_Init(unsigned int ubrr);
void USART_Transmit_string(char *data);
int array_check(char *data);

// ===========================STRUCT DEFINITIONS===========================

typedef struct encoder_data{
	int16_t direction;
	int32_t count;
} encoder;

typedef struct motor_data{
	int16_t velocity_setpoint;
	int16_t velocity;
	int16_t integral_error;
	int16_t previous_error;
	int16_t kp;
	int16_t ki;
	int16_t kd;
} motor;

// ===========================INTERUPTS===========================
ISR(INT1_vect){
	encoder_update (&DCencoder, (PIND & 8) >> 3, (PINC & 1));
}


// Recieve data via USART
// store data in a circular buffer to be read later
ISR(USART_RX_vect){
	RingBuffer_Insert(&buffer, UDR0);
	//	UDR0 = RingBuffer_Peek(&buffer);
}
// ===========================GLOBALS===========================
unsigned int stopFlag = 0;		// if this flag is set to 1, begin soft stop
volatile encoder DCencoder;
volatile RingBuffer_t buffer;

void main(void)
{
	motor DCmotor;
	motor_init(&DCmotor);
	uint8_t bufferdata[15];
	RingBuffer_InitBuffer(&buffer, bufferdata, sizeof(bufferdata));		// create a 15 char buffer.

//fast PWM initialization
TCCR0A = (1 << WGM00)|		// set timer/counter 0 waveform generation mode to fast pwm
(1 << WGM01)|
(1 << COM0A0)|		// set timer/counter 0 compare output mode
(1 <<COM0A1)|
(1 << CS00)&		// set timer/ counter 0 clock source
~(1 << CS01)&
~(1 << CS02);

// perform PID and set output for motor
OCR0A = motor_output(param, OCR0A, do_pid(DCmotor));
//encoder initializations
//pointer to right encoder structure
encoder pREncoder = &REncoder;
// right encoder A input
DDRC |= ~(1 << PC0);
//right encoder B input
DDRD |= ~(1 << PD3);
// right encoder A pullup resistor enabled
PORTC |= (1 << PC0);
// right encoder B pullup resistor enabled
PORTD |= (1 << PD3);
// Any logical change on right encoder
// output A generates an interrupt request
EICRA |= (1 << ISC10);
// enable the external interrupt
EIMSK |= (1 << INT1);

// check holds return value from array_check
// element is the current element in buffer removeal array
// ready is a flag denoting the array is ready to be checked for command
int check, element = 0, ready = 0;
char outs[10];
USART_Init(MYUBRR);
sei();
	while(1){
//		USART_Receive_string(outs);
		if(RingBuffer_GetCount(&buffer) > 0){
		  //take entry out of buffer and put into array[element] location to check for command.
		  // then check for return carriage or new line. if found send array array_check function.
			outs[element] = RingBuffer_Remove(&buffer);
			USART_Transmit(outs[element]);				// echo the data received from USART back
			//check if a return carriage or new line has been reached.
			if ((outs[element] == '\r') || ( outs[element] == '\n')){
				//if so add return, newline, and NULL to end of array and set flag to check array
				outs[element] = '\r';
				outs[element+1] = '\n';
				outs[element+2] = '\0';
				ready = 1;
			}
			element++;
		}		
		if(ready == 1){
			//based on the output from array_check perform the appropriate action
			// if >0, output error message with proper formating directions
			// if 0, print new desired motor speed and send command to change speed
			// if 1, print desired stop time and initiate slow stop
			if((check = array_check(outs)) < 0)		// 
				USART_Transmit_string("ERROR: expected format not given\r\nenter a number for desired speed or type s<x> to have motor soft stop in x seconds:\r\n");
			else if (check == 0){
				USART_Transmit_string("desired motor speed set to: ");
				USART_Transmit_string(outs);
				DCmotor->velocity_setpoint = atoi(outs);
			}
			else{
				USART_Transmit_string("slow stop motor time in seconds: ");
				 char *send = outs+1;
				USART_Transmit_string(send);
			}
			ready = 0;
			element = 0;
		}
		
	}
}

// ===========================FUNCTIONS===========================

// initialize a new motor parameters
void motor_init(motor *m){
		m->velocity_setpoint = 0;
		m->velocity = 0;
		m->integral_error = 0;
		m->previous_error = 0;
		m->kp = 5;
		m->ki = 2;
		m->kd = 2;
}

void encoder_update(encoder *e, int A, int B){
	//determine direction and update encoder count
	// from logic levels of the encoder's A and B outputs
	if (A == 1){
		// rising edge of A
		if (B == 1){
			e->direction = 1;
			e->count ++;
		}
		else{
			e->direction = 0;
			e->count --;
		}
	}
	else{
		// falling edge of A
		if(B == 1){
			e->direction = 0;
			e->count --;
		}
		else{
			e->direction = 1;
			e->count ++;
		}
	}
}

int encoder_velocity(encoder *e){
	//velocity in encoder counts per control loop.
	int velocity = abs(e->count);
	// reset encoder's count variable.
	e->count = 0;
	return velocity;
}

int do_pid(motor *m){
	// the difference between the desired velocity and current velocity
	float p_error;
	float i_error; // sum of errors over time.
	// difference between the previous proportional error
	// and the current proportional error
	float d_error;
	// the proportional component of the output
	float p_output;
	// integral component of the output
	//this term dampens the output to prevent overshoot and oscillation
	float i_output;
	//derivative component of the output
	// this term is responsible for accelerating the output if the error is large
	float d_output;
	// sum of the proportional, integral and derivative terms
	float output;
	//calculate the three errors
	p_error = m->velocity_setpoint - m->velocity;
	i_error = m->integral_error;
	d_error = p_error - m->previous_error;
	//calculate the three components of the PID output.
	p_output = m->kp * p_error;
	i_output = m->ki * i_error;
	d_output = m->kd * d_error;
	// sum the three components of the PID output
	output = p_output + i_output + d_output;
	// update the previous error and the integral error
	m->previous_error = p_error;
	m->integral_error += p_error;
	return (int) output;
}

void USART_Init(unsigned int ubrr)
{
	//Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	//Enable receiver and transmitter
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	// Set frame format: 8data, 1stop bit
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
}

//transmit a string of data via USART
void USART_Transmit_string(char *data)
{
	while(*data){
		while (!(UCSR0A & (1<<UDRE0)));
		// Wait for empty transmit buffer
		UDR0 = *data;
		data++;
	}
}

// this function is used to check if the data received from USART is an appropriate entry
// CHECK FOR:
// 1. all integer string, if found converts to int and returns.
// 2. S<number> or s<number> if found converts number string portion to int, begins stop protocol
// if neither case is found then returns error to transmit message detailing expected input
int array_check(char *data){
	//the return value is used to determine if the data is valid
	// and if so, what the proper course of action to take is
	// if -1 : ERROR expected format is not found
	// if 0: all number string. update desired speed
	// if 1: soft stop requested, second to nth elements contain numerical value determining time to stop
	//
	// element0 is a flag to determine if the checked element is the first one
	// if it is not, and is not a digit then return an error (expected format is not found)
	// sum is used to verify that the numerical value is valid.
	int element0 = 1, sum = 0;
	while((*data != '\r') && ( *data != '\n')){
		if((*data < 48) || (*data > 57)){		// check if element is a digit
			if(element0 == 1){				// check if this is the first element
				if((*data == 's') || (*data == 'S')){
					while((*++data != '\r') && ( *data != '\n')){ // since we have the letter we want, check for and convert found digits.
						if((*data < 48) || (*data > 57))
						return -1;		// if we get a non-digit char now return error (expected format is not found)
						sum = sum + (*data - '0');
					}
					if(sum == 0)
						return -1;	// if the value following "s" is not greater then 0 an invalid command was given
					// if sum is greater then 0 then the input format is valid
					return 1;
				}
			}
			// if this point is reached it is because the current element is a letter and not the first element
			// or not a number then return error, so combine both instances into one outside loop
			return -1;
		}
		// if the code gets to this point it is because the first element was a digit.
		// so make element0 is now "false"
		element0 = 0;
		data++;
	}
	if(element0 == 1)
		return -1;			// this case will only happen if the only data transmitted is enter
	// if this point is reached it is because all values in the array are digits
	return 0;
}
