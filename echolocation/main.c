/*
 * echolocation.c
 *
 * Created: 2018-03-01 9:06:42 PM
 * Author : Abdul
 */ 

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//The CPU frequency
//It is required by delay.h library
#define F_CPU 14754600
#include <util/delay.h>

volatile unsigned int button = 0; //External interrupt boolean
volatile unsigned int object_distance = 0;
volatile unsigned int angle =0;
volatile uint16_t t1=0, t2=0, pulse_width=0;

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
void init_echo(){
	
	TCCR1B |= (1<<CS10) | (1<<CS11)| (1<<ICES1);
	
	DDRB |= (1<<PB1);

}

void trigger_echo()
{
	//trigger 10uS pulse
	PORTB |= (1<<PB1);
	_delay_us(10);
	PORTB &= ~(1<<PB1);
	TCCR1B |= (1<<ICES1);
	 //make sure rising edge detection every time.
}

ISR(TIMER1_CAPT_vect)
{
	//Capture the value of the ICR1 when the rising edge is found
	//Capture the value of the ICR1 when the falling edge is found
	if (TCCR1B & (1<<ICES1)){
		t1 = ICR1;
		TCCR1B &= ~(1<<ICES1); //Select the falling edge
		}else{
		t2= ICR1;
		TCCR1B |= (1<<ICES1); //Select the rising edge
	}
	
}

int measure_echo()
{
	
	pulse_width =t2-t1;

	//CPU resolution is 1/(64*F_CPU) ~4.338uS (PRESCALER /64)
	//Formula: distance = uS/58 (HY-SRF05 Data sheet)
	
	return (int)((pulse_width*4.338)/58);
}

/********************* END OF ECHOLOCATION MODULE*********************/
/********************* SERVO MODULE **********************************

*/

//"90" (~0.615 ms pulse) is all the way to the left
//"90" (~2.45ms pulse) is all the way to the right
uint8_t intitial_duty=(int)((F_CPU*0.615e-3)/1024); //=9 5% duty cycle (0 deg angle)
uint8_t final_duty=(int)((F_CPU*2.425e-3)/1024); //=35 10% duty cycle (180 deg angle)
//uint8_t  increment_duty=(int)((F_CPU*0.15e-3)/1024); //duty cycle increments

//initialize duty cycle
 int duty_cycle = 0;

void init_servo(){
	// initialize timer0 in PWM mode
	TCCR0A |= (1<<WGM00)|(1<<WGM01); //fast PWM mode. TOP = 0xFF
	TCCR0A |= (1<<COM0B1)|(1<<COM0A1); //non-inverted PWM waveform
	TCCR0B |=(1<<CS00)|(1<<CS02); //PRESCALER /1024
	
	DDRB |=(1<<PB3)|(1<<PB4);
}

int rotate(void)
{
	if(duty_cycle >= intitial_duty && duty_cycle < final_duty)
	{
		//Update the duty cycle
		OCR0A = duty_cycle++;
		}else{
		duty_cycle = intitial_duty;
		OCR0A = duty_cycle;
	}
	
	//Signal Echo locator to start detecting objects
	trigger_echo();
	
	return (duty_cycle-intitial_duty)*7;
}
/********************* END OF SERVO *********************************/

/********************* RESET MODULE *********************************
*/
void configure_interrupts()
{
	EICRA |= (1<<ISC00)|(1<<ISC01);//rising edge only on INT0 (PD2)
	EIMSK |= (1<<INT0);//enable external interrupt
	
	TIMSK1 = (1<<ICIE1); //Enable input capture interrupt
	
	sei();
}

ISR(INT0_vect)
{
	//OCR0A = button;
}

/********************* END OF RESET**********************************/

/********************* MAIN FUNCTION*********************************
*/

int main(void)
{	
	// configure all modules
	init_uart();
	configure_interrupts();
	init_servo();
	init_echo();
	
	while(1)
	{
			//Rotate servo by 1 degree
			angle = rotate();
			//Find the distance of the object
			object_distance = measure_echo();	
			//if an object is detected, display its distance and angle on the screen 
			if(object_distance)
			{
				_delay_ms(1000);
				printf("%u,%u.",angle,object_distance);
				object_distance = 0;
			}
	}
	
}

/********************* END OF MAIN FUNCTION*******************************/
