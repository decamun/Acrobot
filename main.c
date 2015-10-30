
#include "m_general.h"
#include "timer0.h"
#include "timer1.h"
#include "ADC.h"
#include "m_usb.h"
#include <math.h>

volatile uint16_t timer0_ticks = 0;
volatile uint16_t milliseconds = 0;
volatile uint16_t ADC_saved_value = 0;


volatile uint16_t intensity = 0;
uint16_t max_time = 200;
int shooting = 0;

float get_duty(uint16_t ADC_value);

int main() {

	//Initialize
	m_clockdivide(0); //16MHz
	start0(255); //timer prescaler 8 || 256 spots || 2048 clockcycles/interupt
	interupt0(1); //enable timer 0 interupts

	startADC(); //enable ADC
	interuptADC(1); //

	//set port B5 input, and enable pull up resistor
	clear(DDRB,5); 	//set input
	set(PORTB,5); 	//enable pull-up

	m_usb_init();


	while(1) {
		m_usb_tx_uint(ADC_saved_value);
		m_usb_tx_string("\n\r");
	}

	return 0;
}


float get_duty(uint16_t ADC_value) {
	 return 0.80 + 0.20 * (1.0 - (float)ADC_value / 960.0);
}



ISR(TIMER0_OVF_vect) {

	timer0_ticks++;

	if(timer0_ticks > 8) {

		milliseconds++; //approx (very approx) 1 millisecond
		timer0_ticks = 0;
	}

	if(shooting && milliseconds > max_time) {
		m_red(OFF);
		m_green(OFF);
		//stop shooting at the correct time
		stop1();
		shooting = 0;
	}

	//check if button is pressed
	if(!shooting && !check(PINB, 5)) {
		m_red(ON);
		shooting = 1;
		start_pwm1(1000, get_duty(ADC_saved_value));
		milliseconds = 0;
	}

}

ISR(ADC_vect) {
	ADC_saved_value = ADC; //do nothing
	m_green(ON);
}
