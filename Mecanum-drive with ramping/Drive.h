/* 
* Drive.h
*
* Created: 1/10/2018 8:15:49 PM
* Author: Subash Timilsina
*/


#ifndef __DRIVE_H__
#define __DRIVE_H__

#include "Motor.h"
#include "uart.h"
#include "Encoder.h"
#include "JOYSTICK.h"

//#define PI					3.141592
#define MAX_RPM					240						//In RPM	//for 12v
#define WHEEL_CIRCUMFERENCE		182					//In millimeters
#define OFFSET_SPEED_UP			40						//In RPM
#define OFFSET_SPEED_DOWN		40
//#define WHEEL_RADIUS			150						//In mm
//#define MAX_VEL      MAX_RPM/(2*PI*Wheel_Radius)		//In m/s

class Drive
{
	 private:
	 int i,j;
	 int manual_rpm;
	 int auto_rpm;
	 int velocity_motor[4];
	 int velocity_robot[4];
	 int ocr_motor[4];

	 long int destination;
	 long int dist_count;
	 float slope;
	 int val;
	 bool ramp_completeflag;
	 bool startflag ;
	 bool robot_mode;

	 Motor m[4];

	 public:
	 void init();
	 void reset_robvel();
	 void Move_the_Robot();
	 void Semi_automatic_mode();
	 void Manual_Mode();
	 void calculate_wheel_velocity();
	 void update_wheel_velocity();
	 void perform_ramping();
	 void reset_joystick_data();
}; 

#endif //__DRIVE_H__
