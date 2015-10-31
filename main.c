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

//program flags
volatile int _flag_recieved_IMU = 0;

<<<<<<< HEAD
int* data[9];
=======
volatile int data[9];
>>>>>>> origin/master


//unsigned char = m_imu_init(unsigned char accel_scale, unsigned char gyro_scale);

//volatile unsigned char buffer[PACKET_LENGTH] = {0,0,0};



int main(void)
{
		DDRE |= 1<<6;
		PORTE &= !(1<<6);
    /* insert your hardware initialization here */

		m_red(OFF);
		m_green(OFF);
		m_clockdivide(0); //16MHz
		m_usb_init();
		int imu_worked = m_imu_init((unsigned char)accel_scale, (unsigned char)gyro_scale);
		start0(250); //start the timer at 250 0CR0B
		interupt0(1); //enable timer interupt
		//m_rf_open(CHANNEL, RXADDRESS, PACKET_LENGTH);

		m_red(ON);
		int i = 0;
    while(1) {
		/*m_usb_tx_string("No Data. Timer value:");
		m_usb_tx_int(TCNT0);
		m_usb_tx_string("\n\r");*/

    	m_red(TOGGLE);
    	m_wait(500);
			if(_flag_recieved_IMU) {
				_flag_recieved_IMU = 0;
				for(i = 0; i < 9; i++) {
					m_usb_tx_int(*data[i]);
					m_usb_tx_string("\t");
				}
				m_usb_tx_string("\n\r");
			}
    }
    return 0;   /* never reached */
}


/*ISR(INT2_vect){
	m_rf_read(buffer, PACKET_LENGTH);
	m_green(TOGGLE);
}*/

ISR(TIMER0_OVF_vect) {
//code also goes here
	int worked = m_imu_raw(data);
	if(worked) {

		m_green(TOGGLE);
	}
}

ISR(ADC_vect) {
//more code here

}
