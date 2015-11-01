#include "m_general.h"
#include "timer0.h"


void start0(uint8_t frequency) {

  OCR0A = frequency;

  //timer mode: up to OCR0A
  set(TCCR0B, WGM02);
  set(TCCR0A, WGM01);
  set(TCCR0A, WGM00);

  //trigger mode: no change
  clear(TCCR0A, COM0A1);
  clear(TCCR0A, COM0A0);
  clear(TCCR0A, COM0B1);
  clear(TCCR0A, COM0B0);

  //clock prescaler /1024: 101
  set(TCCR0B, CS02);    //1
  clear(TCCR0B, CS01);  //0
  set(TCCR0B, CS00);    //1
}

void stop0() {
  clear(TCCR0B, CS02);     	//0
  clear(TCCR0B, CS01);   	  //0
  clear(TCCR0B, CS00);     	//0
}

void interupt0(bool on) {
  if(on) {
    set(TIMSK0, TOIE0); //enable timer interupt
    sei(); //turn on interupts
  } else {
    clear(TIMSK0, TOIE0);
  }
}
