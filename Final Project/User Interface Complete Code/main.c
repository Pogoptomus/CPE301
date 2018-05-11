#define F_CPU 8000000				// Clock Speed
#define BAUD 9600					// define BAUD
#define MYUBRR F_CPU/16/BAUD-1		// used to set UBRR0 to correct value

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <util/atomic.h>
#include "ring_buffer.h"

//function declarations
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void USART_Transmit_string(char *data);
unsigned char USART_Receive(void);
void USART_Receive_string(char *data);
int array_check(char *data);

unsigned int stopFlag = 0;		// if this flag is set to 1, begin soft stop 
volatile RingBuffer_t buffer;

ISR(USART_RX_vect){
	RingBuffer_Insert(&buffer, UDR0);
//	UDR0 = RingBuffer_Peek(&buffer);
}

void main(void)
{
	uint8_t bufferdata[15];
	RingBuffer_InitBuffer(&buffer, bufferdata, sizeof(bufferdata));		// create a 15 char buffer.
	// check holds return value from array_check
	// element is the current element in buffer removeal array
	// ready is a flag denoting the array is ready to be checked for command
	int check, element = 0, ready = 0;		
	char outs[10];
	USART_Init(MYUBRR);
	sei();
	//USART_Transmit_string("Set desired motor speed. use s<x> to have motor soft stop in x seconds:\r\n");
	while(1)
	{

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


//============================================USART functions=================================================
void USART_Init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
}

void USART_Transmit(unsigned char data)
{
		while (!(UCSR0A & (1<<UDRE0)));
		/* Wait for empty transmit buffer */
		UDR0 = data;
}

//transmit a string of data
void USART_Transmit_string(char *data)
{
	while(*data){
		while (!(UCSR0A & (1<<UDRE0)));
		// Wait for empty transmit buffer 
		UDR0 = *data;
		data++;
	}
}

/*
unsigned char USART_Receive(void)
{
	// Wait for data to be received 
	while ( !(UCSR0A & (1<<RXC0)) );
	return(UDR0);
}

void USART_Receive_string(char *data){
	// store recieved data byte in memory location, then check if new line is reached.
	// if not increment to the next memory location and repeat
	//	while ((*data = USART_Receive()) != '\n'){
	//		*data++;		// increment to the next memory location for the array
	//	}
	//once new line reached set last memory value to NULL
	//	*data = '\0';
	char send;
	while(((*data = USART_Receive()) != '\r' )&&(*data != '\n' )){
		send = *data;
		USART_Transmit(send);
		data++;
	}
//  add carriage return and new line and null pointer to end of string then perform a carriage return and new line.
	*data++ = '\r';
	*data++ = '\n';
	*data = '\0';
	USART_Transmit_string("\r\n");
}
*/
//============================================================================================================

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

