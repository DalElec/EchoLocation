/*
*
* main.c
*
* Created: 2/12/2018 2:16:36 PM
* Author: Carl Mattatall
*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//The CPU frequency
//It is required by delay.h library
#define F_CPU 14754600 
#include <util/delay.h>

//Initialize the time at the echo pulse edges 
volatile uint16_t Rising_E_time=0, Falling_E_time=0; 
volatile uint16_t time_elapsed=0; //The time between the falling and rising edge

volatile unsigned int button = 0; //External interrupt boolean

unsigned int object_distance = 0;

/************************ USART MODULE**********************************
 USART FILE IMPORTS AND FUNCTION DECLARATIONS
*/
int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);
FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE mystdin = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

int uart_putchar(char c, FILE *stream)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}
int uart_getchar(FILE *stream)
{
	/* Wait until data exists. */
	loop_until_bit_is_set(UCSR0A, RXC0);
	return UDR0;
}
void init_uart(void)
{
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	UBRR0 = 95;//7; // configures a baud rate of 115200
	stdout = &mystdout;
	stdin = &mystdin;
	printf("\n USART system booted\n");
}

/********************* END OF USART MODULE***************************/

/********************* ECHOLOCATION MODULE***************************
 Echolocation functions
*/

void trigger_echo()
{
	//trigger 10uS pulse
	PORTB |= (1<<PB1);
	_delay_us(10);
	PORTB &= ~(1<<PB1);
}

ISR(INT0_vect)
{
	button = 1;
}

ISR(TIMER1_CAPT_vect)
{
	//Capture the value of the ICR1 when the rising edge is found
	//Capture the value of the ICR1 when the falling edge is found
	if (TCCR1B & (1<<ICES1)){
		Rising_E_time = ICR1;
		TCCR1B &= ~(1<<ICES1); //Select the falling edge
		}else{
		Falling_E_time = ICR1;
		TCCR1B |= (1<<ICES1); //Select the rising edge
	}
	
}

int measure_echo()
{
	trigger_echo();
	
	time_elapsed = Falling_E_time-Rising_E_time;

	//CPU resolution is 1/F_CPU ~4.338uS (PRESCALER /64)
	//Formula: distance = uS/58 (HY-SRF05 Data sheet)
	
	return (int)((time_elapsed*4.338)/58);
}

/********************* END OF ECHOLOCATION MODULE*********************/

/********************* MAIN FUNCTION*********************************/

int main(void)
{	
	DDRB |= (1<<PB1);
	
	EICRA |= (1<<ISC00)|(1<<ISC01);//rising edge only on INT0 (PD2)
	EIMSK |= (1<<INT0);//enable external interrupt
	
	//start timer (PRESCALER /64)
	//Capture the rising edge
	TCCR1B =(1<<CS11) |(1<<CS10)|(1<<ICES1);
	
	TIMSK1 = (1<<ICIE1); //Enable input capture interrupt
	
	sei();
	
	init_uart();

	while(1){
		
		if(button){
			
			button = 0;
			
			object_distance = measure_echo();
			
			printf("The object distance is %d cm \n", object_distance);
		}
	}				
}
