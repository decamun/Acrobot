/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
//#include "m_rf.h"
#include "m_imu.h"
#include "timer0.h"
#include "timer1.h"
//#include "ADC.h"
#include <math.h>



#define accel_scale 0
#define gyro_scale 1
#define CHANNEL 1
#define RXADDRESS 0x6B
#define PACKET_LENGTH 3
#define TIMER1_RES 8000 //timer resolution -- set to 20kHz
#define LEFT 0
#define RIGHT 1
//gains
#define LPF_accel 0.005
#define HPF_gyro 0.995
#define kp 10
#define ki 0
#define kd 0
#define GYRO_CONSTANT 0.015259
#define TIME_STEP 0.003968

//program flags
volatile int _flag_recieved_IMU = 0;

//math values
volatile int data[9];
volatile float prev_angle;

void set_direction(int direction);
void PID();
float get_acc_angle();
float get_gyro_angle();


//unsigned char = m_imu_init(unsigned char accel_scale, unsigned char gyro_scale);

//volatile unsigned char buffer[PACKET_LENGTH] = {0,0,0};



int main(void)
{
		DDRE |= 1<<6;
		PORTE &= !(1<<6);
    /* insert your hardware initialization here */

		m_usb_init();
		m_red(OFF);
		m_green(OFF);
		m_clockdivide(0); //16MHz
		int imu_worked = m_imu_init((unsigned char)accel_scale, (unsigned char)gyro_scale);
		start0(62); //start the timer at 250 0CR0B
		interupt0(1); //enable timer interupt\

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

			if(_flag_recieved_IMU)
			{
				PID();
				_flag_recieved_IMU = 0;
				//m_green(OFF);
			}
    }
    return 0;   /* never reached */
}

float get_acc_angle()
{
	static float x_accel_filter = 0;
	static float z_accel_filter = 0;
	static float angle_filter = 0;
	float x_accel = -(float)data[0];
	float z_accel = (float)data[2];

	x_accel_filter = x_accel_filter * (1 - LPF_accel) + x_accel * (LPF_accel);
	z_accel_filter = z_accel_filter * (1 - LPF_accel) + z_accel * (LPF_accel);

	if(z_accel_filter == 0)
	{
		z_accel_filter = 0.01;
	}

	angle_filter = atanf(x_accel_filter/z_accel_filter);

	//m_usb_tx_string("\t Acc Angle: ");
	//m_usb_tx_int((int)(atanf(x_accel/z_accel)*1000));
	m_usb_tx_string("\t Acc Angle Filter: ");
	m_usb_tx_int((int)(angle_filter * 1000));

	return angle_filter;
}

float get_gyro_angle()
{
	static float gyro_filter = 0;
	static int16_t prev_gyro = 0;
	static int16_t gyro = 0;
	static float gyro_rate = 0;
	static float gyro_angle_filter = 0;
	static float prev_gyro_angle = 0;
	static float gyro_angle = 0;

	gyro = data[4]; //y gyro
	prev_gyro_angle = gyro_angle;
	gyro_angle = gyro_angle + gyro * TIME_STEP * GYRO_CONSTANT;
	gyro_angle_filter = HPF_gyro * (gyro_angle_filter + (gyro_angle - prev_gyro_angle));

	m_usb_tx_string("\t Gyro Angle Filter: ");
	m_usb_tx_int((int)(gyro_angle_filter * 3.14 * 1000 / 180));
	//m_usb_tx_string("\t Gryro Rate:");
	//m_usb_tx_int(gyro);


	return gyro_angle_filter * 3.14 / 180;//gyro_angle_filter;
}

void PID()
{
	static float PID_p = 0;
	static float PID_i = 0;
	static float PID_d = 0;
	static float prev_angle = 0;
	static float angle_estimate = 0;
	static float angle_offset = 0;

	//save d/dt
	m_usb_tx_string("\t Angle Estimate Combination: ");
	m_usb_tx_int((int)(angle_estimate*1000));
	prev_angle = angle_estimate;
	angle_estimate = get_acc_angle() + get_gyro_angle() + angle_offset;
	m_usb_tx_string("\n\r");


	//Proportional
	PID_p = angle_estimate * kp;

	//Integral
	PID_i = PID_i + angle_estimate * ki;
	if(PID_i > 1) {
		PID_i = 1;
	} else if(PID_i < -1) {
		PID_i = -1;
	}

	//Derivitive
	PID_d = (angle_estimate - prev_angle) * kd;



	float PID_out = PID_p + PID_i + PID_d;

	//check the sign of the output
	int PID_sign = (PID_out > 0);
	if(fabs(PID_out) > 1) {
		PID_out = 1;
	}

	angle_offset = angle_offset + PID_out * 0.001;

	//output
	set_direction(PID_sign);
	set_duty1(fabs(PID_out));
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
	if(_flag_recieved_IMU) {
		m_red(ON);
	} else {
		m_red(OFF);
		m_imu_raw(data);
		_flag_recieved_IMU = 1;
		m_green(ON);
	}
}

//ISR(ADC_vect)
//{
