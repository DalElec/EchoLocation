/*
*
* main.c
*
* Created: 2/25/2018 7:18:36 PM
* Author: Abdullahi Muse
*/

#include <avr/io.h>
//The CPU frequency
//It is required by delay.h and TIMERS
#define F_CPU 14754600
#include <util/delay.h>

void pwm_init()
{
	// initialize timer0 in PWM mode
	TCCR0A |= (1<<WGM00)|(1<<WGM01); //fast PWM mode. TOP = 0xFF
	TCCR0A |=(1<<COM0B1)|(1<<COM0A1); //non-inverted PWM waveform
	TCCR0B |=(1<<CS00)|(1<<CS02); //PRESCALER /1024

	//The OC0A pin (pin PB3)
	DDRB |= (1<<PB3);
}

int main(void)
{
	//initialize duty cycles
	//"90" (~0.615 ms pulse) is all the way to the left
	//"90" (~2.45ms pulse) is all the way to the right
	//
	uint8_t  intitial_duty=(int)((F_CPU*0.615e-3)/1024); //5% duty cycle (0 deg angle)
	uint8_t final_duty=(int)((F_CPU*2.425e-3)/1024); // 10% duty cycle (180 deg angle)
	uint8_t  increment_duty=(int)((F_CPU*0.15e-3)/1024); //duty cycle increments
	
	// initialize PWM
	pwm_init();
	while(1)
	{
		//start position of the servo motor is at far left (0 deg reference)
		for (int duty_cycle = intitial_duty; duty_cycle < final_duty; duty_cycle += increment_duty)
		{
			//Update the duty cycle
			OCR0A = duty_cycle;
			_delay_ms(1000);
		}
	}
}
