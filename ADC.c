//default channel: F0
//must include ISR(ADC_vect) in main.c
#include "m_general.h"
#include "ADC.h"

void startADC() {
  //ADC Setup

  //Voltage Reference : Vcc
  clear(ADMUX, REFS1);
  set(ADMUX, REFS0);

  //ADC Prescaler : /128
  set(ADCSRA, ADPS2);
  set(ADCSRA, ADPS1);
  set(ADCSRA, ADPS0);

  //Disabling Digital Inputs
  set(DIDR0, ADC0D);

  //Free run enable
  set(ADCSRA, ADATE);

  //Channel Selection: F0
  clear(ADCSRB, MUX5);
  clear(ADMUX, MUX2);
  clear(ADMUX, MUX1);
  clear(ADMUX, MUX0);

  //start conversion process
  set(ADCSRA, ADEN); //enable
  set(ADCSRA, ADSC); //begin
}

void interuptADC(int interupt_on) {
  if(interupt_on) {
    //turn on interupts
    set(ADCSRA, ADIE);
    sei();
  } else {
    clear(ADCSRA, ADIE);
  }
}

void set_channelADC(uint8_t channel) {
  clear(ADCSRA, ADEN);
  ADMUX = (ADMUX & 0xF8) | (0x7 & channel); //set the ADC channel
  set(ADCSRA, ADEN);
}

uint8_t get_channelADC() {
  return ADMUX & 0x7; //mask and return ADC pin
}
