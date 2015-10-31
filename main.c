/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "m_rf.h"
#include "m_imu.h"
#include "timer0.h"
#include "timer1.h"
#include "ADC.h"
#include <math.h>

#define accel_scale 0
#define gyro_scale 0
#define CHANNEL 1
#define RXADDRESS 0x6B //alto
#define PACKET_LENGTH 3

//unsigned char = m_imu_init(unsigned char accel_scale, unsigned char gyro_scale);

unsigned char buffer[PACKET_LENGTH] = {0,0,0};


int main(void)
{
		DDRE |= 1<<6;
		PORTE &= !(1<<6);
    /* insert your hardware initialization here */

		m_clockdivide(0); //16MHz
		m_usb_init();
		m_imu_init(accel_scale, gyro_scale);


    while(1){


        /* insert your main loop code here */
    }
    return 0;   /* never reached */
}


ISR(INT2_vect){
	m_rf_read(buffer, PACKET_LENGTH);
	m_green(TOGGLE);
}

ISR(TIMER0_OVF_vect) {
//code also goes here

}

ISR(ADC_vect) {
//more code here

}
