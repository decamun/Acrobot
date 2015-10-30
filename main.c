
#include "m_general.h"
#include "timer0.h"
#include "timer1.h"
#include "ADC.h"
#include "m_usb.h"
#include <math.h>

int main() {

	//Initialize
	m_clockdivide(0); //16MHz
	//code goes here

	return 0;
}

ISR(TIMER0_OVF_vect) {
//code also goes here

}

ISR(ADC_vect) {
//more code here

}
