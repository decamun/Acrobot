/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include "m_general.h"
//#include "m_usb.h"
#include "m_bus.h"
//#include "m_rf.h"
#include "m_imu.h"
#include "timer0.h"
#include "timer1.h"
//#include "ADC.h"
#include <math.h>

#define accel_scale 0
#define gyro_scale 0
#define CHANNEL 1
#define RXADDRESS 0x6B
#define PACKET_LENGTH 3
#define TIMER1_RES 8000 //timer resolution -- set to 20kHz
#define LEFT 0
#define RIGHT 1
//gains
#define LPF_accel 0.05
#define p_gain 3
#define i_gain 0.05
#define d_gain 15

//program flags
volatile int _flag_recieved_IMU = 0;

//math values
volatile int data[9];
volatile float x_accel_filter = 0;
volatile float y_accel_filter = 0;
volatile float z_accel_filter = 0;
volatile float acc_angle;
volatile float prev_angle;
volatile float PID_p = 0;
volatile float PID_i = 0;
volatile float PID_d = 0;

void set_direction(int direction);
void PID();
float get_acc_angle();


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
		//m_usb_init();
		int imu_worked = m_imu_init((unsigned char)accel_scale, (unsigned char)gyro_scale);
		//start0(250); //start the timer at 250 0CR0B
		//interupt0(1); //enable timer interupt
		//m_rf_open(CHANNEL, RXADDRESS, PACKET_LENGTH);

		m_red(ON);
		int i = 0;

		start_pwm1(TIMER1_RES, 0.1); // output to B6
		set(DDRB, 0); //B0 to output, ENABLE LINE
		set(DDRB, 1); //B1 to output, Direction Motor 1
		set(DDRB, 2); //B2 to output Direction Motor 2

		set(DDRB, 3);
		set(PORTB, 3); //turn on B3 as logic supply

    while(1)
		{
			/*m_usb_tx_string("No Data. Timer value:");
			m_usb_tx_int(TCNT0);
			m_usb_tx_string("\n\r");*/

			int worked = m_imu_raw(data);
			if(worked)
			{
				_flag_recieved_IMU = 1;
				m_green(ON);
			}

			if(_flag_recieved_IMU)
			{
				_flag_recieved_IMU = 0;
				PID();
				//set_direction(data[0] > 0);
				//set_duty1(fabs(((float)data[0] / (float)18000)));
				m_green(OFF);

				//for(i = 2; i < 3; i++)
				//{
				//	m_usb_tx_int(data[i]);
				//	m_usb_tx_string("\t");
				//}
				//m_usb_tx_string("\n\r");
			}
    }
    return 0;   /* never reached */
}

float get_acc_angle() {
	float x_accel = (float)data[0];
	//float y_accel = (float)data[1];
	float z_accel = (float)data[2];
	//low pass filter accelerometer values
	x_accel_filter = x_accel_filter * (1 - LPF_accel) + x_accel * (LPF_accel);
	//y_accel_filter = y_accel_filter * (0.95) + y_accel * (0.05);
	z_accel_filter = z_accel_filter * (1 - LPF_accel) + z_accel * (LPF_accel);
	//get angle from accelorometer values
	if(z_accel_filter == 0)
	{
		z_accel_filter = 0.01;
	}
	prev_angle = acc_angle;
	acc_angle = atanf(x_accel_filter/z_accel_filter);
	return acc_angle;
}

void PID()
{
	//Proportional
	PID_p = get_acc_angle() * p_gain;

	//Integral
	PID_i = PID_i + acc_angle * i_gain;
	if(PID_i > 0.3) {
		PID_i = 0.3;
	} else if(PID_i < -0.3) {
		PID_i = -0.3;
	}

	//Derivitive
	PID_d = (acc_angle - prev_angle) * d_gain;



	float PID_out = PID_p + PID_i + PID_d;

	//check the sign of the output
	int PID_sign = (PID_out > 0);

	//output
	set_direction(PID_sign);
	set_duty1(fabs(PID_out/1.5708));
}

void set_direction(int direction)
{
	set(PORTB, 0); // enable line high
	if (direction)
	{
		set(PORTB, 1);
		set(PORTB, 2);
	}
	else
	{
		clear(PORTB, 1);
		clear(PORTB, 2);
	}
}

/*ISR(INT2_vect){
	m_rf_read(buffer, PACKET_LENGTH);
	m_green(TOGGLE);
}*/

ISR(TIMER0_OVF_vect)
{
//code also goes here
	int worked = m_imu_raw(data);
	if(worked)
	{
		_flag_recieved_IMU = 1;
		m_green(ON);
	}
}

//ISR(ADC_vect)
//{
