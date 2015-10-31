#include "m_general.h"
#include "timer1.h"


volatile uint16_t _max_value1 = 0;

void start_pwm1(uint16_t resolution, float duty_cycle) {
  _max_value1 = resolution; //save resolution

  OCR1A = resolution;  //frequency
  OCR1B = (uint16_t)(((float)resolution) * duty_cycle); //duty cycle

  //timer mode: 15 -> 1111
  set(TCCR1B, WGM13); //1
  set(TCCR1B, WGM12); //1
  set(TCCR1A, WGM11); //1
  set(TCCR1A, WGM10); //1

  //output options B6: 11
  set(TCCR1A, COM1B1);   //1
  clear(TCCR1A, COM1B0); //0

  //clock prescaler: /8 -> 010
  clear(TCCR1B, CS12);    //0
  set(TCCR1B, CS11);  //1
  clear(TCCR1B, CS10);  //0
  set(DDRB, 6); //set port B6 output
}

void set_duty1(float duty_cycle) {
  OCR1B = (uint16_t)(((float)_max_value1) * duty_cycle); //duty cycle
}

//stop timer1
void stop1() {
  //clock prescaler: 0 -> 000
  clear(TCCR1B, CS12);   //0
  clear(TCCR1B, CS11);   //0
  clear(TCCR1B, CS10);   //0
  clear(DDRB, 6);
}
