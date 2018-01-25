/* 
* Drive.cpp
*
* Created: 1/10/2018 8:15:49 PM
* Author: Subash Timilsina
*/


#include "Drive.h"

float coupling_matrix[4][3] = {{-0.707,0.707,-0.707},{0.707,0.707,0.707},{-0.707,0.707,0.707},{0.707,0.707,-0.707}};
Encoder e;
extern signed char rcvdata[8];


void Drive::init()
{
	
	auto_rpm = 180;
	manual_rpm = 80;
	for(int i=0;i<4;i++)
	{
		velocity_motor[i] = 0;
		velocity_robot[i] = 0;
		m[i].Initialise(i+1);
	}
	destination = 0;
	dist_count = 0;
	slope = 0;
	ramp_completeflag = false;
	robot_mode = false;
	startflag = false;
	reset_robvel();
	reset_joystick_data();
	e.Initialise(5);
}

void Drive::Move_the_Robot()
{
	if (GAMEBUTTONA == START_BUTTON)
		{startflag = (1^startflag);
		GAMEBUTTONA = 0;}
		
	 while(startflag && robot_mode)
	 {
		Semi_automatic_mode();
	 }
	while(startflag && (!robot_mode))
	{
		Manual_Mode();
	}
}

 void Drive::Semi_automatic_mode()
 {
	 if (GAMEBUTTONA == BUTTON_A)
	 {
		 velocity_robot[0] = 30;
		 velocity_robot[1] = auto_rpm;
		 velocity_robot[2] = 0;
		 destination = 3500;
		 dist_count = (destination/WHEEL_CIRCUMFERENCE)*PPR_DISTANCE;
		 val = 1;
		 e.Set_count(0);
		 calculate_wheel_velocity();
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == BUTTON_B)
	 {
		 velocity_robot[0] = 30;
		 velocity_robot[1] = -auto_rpm;
		 velocity_robot[2] = 0;
		 destination = 10000;
		 dist_count = (destination/WHEEL_CIRCUMFERENCE)*PPR_DISTANCE;
		 val = -1;
		 e.Set_count(0);
		 calculate_wheel_velocity();
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == BUTTON_X)
	 {
		 velocity_robot[0] = 30;
		 velocity_robot[1] = auto_rpm;
		 velocity_robot[2] = 0;
		 destination = 10000;
		 dist_count = (destination/WHEEL_CIRCUMFERENCE)*PPR_DISTANCE;
		 val = 1;
		 e.Set_count(0);
		 calculate_wheel_velocity();
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == BUTTON_Y)
	 {
		 velocity_robot[0] = 30;
		 velocity_robot[1] = -auto_rpm;
		 velocity_robot[2] = 0;
		 destination = 2000;
		 dist_count = (destination/WHEEL_CIRCUMFERENCE)*PPR_DISTANCE;
		 val = -1;
		 e.Set_count(0);
		 calculate_wheel_velocity();
		 GAMEBUTTONA = 0;
	 }
	 else if (GAMEBUTTONA == RIGHT_BUTTON)
	 {
		 velocity_robot[0] = 30;
		 velocity_robot[1] = auto_rpm;
		 velocity_robot[2] = 0;
		 destination = 11600;
		 dist_count = (destination/WHEEL_CIRCUMFERENCE)*PPR_DISTANCE;
		 val = 1;
		 e.Set_count(0);
		 calculate_wheel_velocity();
		 GAMEBUTTONA = 0;
	 }
	 
	 if (GAMEBUTTONB == LEFT_BUTTON)
	 {
		 velocity_robot[0] = 30;
		 velocity_robot[1] = -auto_rpm;
		 velocity_robot[2] = 0;
		 destination = 11600;
		 dist_count = (destination/WHEEL_CIRCUMFERENCE)*PPR_DISTANCE;
		 val = -1;
		 e.Set_count(0);
		 calculate_wheel_velocity();
		 GAMEBUTTONB = 0;
	 }
	 
	 else if (GAMEBUTTONB == BACK_BUTTON)
	 {
		 reset_robvel();
		 destination = 0;
		 dist_count = 0;
		 val = 0;
		 e.Set_count(0);
		 ramp_completeflag = false;
		 robot_mode = false;
		 GAMEBUTTONB = 0;
	 }
	 perform_ramping();
 }
 
 void Drive::Manual_Mode()
 {
	 if (GAMEBUTTONB == UP)
	 {
		 velocity_robot[0] = 0;
		 velocity_robot[1] = manual_rpm;
		 velocity_robot[2] = 0;
	 }
	 else if (GAMEBUTTONB == RIGHT)
	 {
		 velocity_robot[0] = 0;
		 velocity_robot[1] = -manual_rpm;
		 velocity_robot[2] = 0;
	 }
	 else if (GAMEBUTTONB == DOWN)
	 {
		 velocity_robot[0] = -manual_rpm;
		 velocity_robot[1] = 0;
		 velocity_robot[2] = 0;
	 }
	 else if (GAMEBUTTONB == LEFT)
	 {
		 velocity_robot[0] = manual_rpm;
		 velocity_robot[1] = 0;
		 velocity_robot[2] = 0;
	 }
	 
	 if (GAMEBUTTONA == BUTTON_A)
	 {
	 manual_rpm -= 20; 
	 GAMEBUTTONA = 0; 
	 }
	 else if (GAMEBUTTONA == BUTTON_B)
	 {
	 manual_rpm += 20; 
	 GAMEBUTTONA = 0;
	 }
	 if (abs(LEFTSTICKY) > 2)
	 {
		 velocity_robot[0] = 0;
		 velocity_robot[1] = ((LEFTSTICKY)/50.0)*200;
		 velocity_robot[2] = 0;
	 }
	 if (abs(LEFTSTICKX) > 2)
	  {
		  velocity_robot[0] = (-(LEFTSTICKX)/50.0)*200;
		  velocity_robot[1] = 0;
		  velocity_robot[2] = 0;
	  }
	  if (abs(RIGHTSTICKX)>2)
	  {
		  velocity_robot[0] = 0;
		  velocity_robot[1] = 0;
		  velocity_robot[2] = (-(RIGHTSTICKX)/50.0)*200;
		  
	  }
	
	 if (GAMEBUTTONB == BACK_BUTTON)
	 {
		 reset_robvel();
		 robot_mode = true;
		 GAMEBUTTONB = 0;
	 }
	 if (manual_rpm>MAX_RPM) manual_rpm = MAX_RPM;
	 else if (manual_rpm<0)	manual_rpm = 0;
	
	  calculate_wheel_velocity();
	  update_wheel_velocity();
	  reset_robvel();
	   
 }

void Drive::calculate_wheel_velocity()
{
	for(i=0;i<4;i++)
	{
		velocity_motor[i] = 0;
		for(j=0;j<3;j++)
		{
			velocity_motor[i] += velocity_robot[j] * coupling_matrix[i][j];
		}
		
	}
	
	for(i=0;i<4;i++)
	{
		ocr_motor[i] = (249.0*velocity_motor[i])/(MAX_RPM);;
	}
	
}

void Drive::update_wheel_velocity()
{
	for(i = 0; i<4 ; i++)
		m[i].SetOcrValue(ocr_motor[i]);
}

void Drive::perform_ramping()
{
	if(abs(e.Get_count()) >= dist_count)
	{
		//stop the robot
		if (ramp_completeflag)
		{
			//velocity_robot[0] = 50;
			//velocity_robot[1] = 0;
			//velocity_robot[2] = 0;
			//calculate_wheel_velocity();
			//update_wheel_velocity();
			//_delay_ms(400);
			ramp_completeflag = false;
		}

		for (i = 0;i < 4;i++ )
		{
			m[i].StopMotor();
		}
		dist_count = 0;
		reset_robvel();
	}
	else if (abs(e.Get_count()) >= (dist_count>>3) && (abs(e.Get_count()) <= 7*(dist_count>>3)))
	{
		//normal
		update_wheel_velocity();
			
	}
	else if (abs(e.Get_count()) <= (dist_count>>3))
	{
		//ramp up
		for (i = 0;i<4;i++)
		{
			slope = ((8.0*(ocr_motor[i]-val*OFFSET_SPEED_UP))/dist_count);
			m[i].SetOcrValue(val*((slope*e.Get_count())+OFFSET_SPEED_UP));
		}
	}
	
	else if ((abs(e.Get_count()) <= dist_count) && (abs(e.Get_count()) >= 7*(dist_count>>3)))
	{
		//ramp down
		for (i = 0;i < 4;i++ )
		{
			slope = -(8.0*(ocr_motor[i]-val*OFFSET_SPEED_DOWN)/dist_count);
			m[i].SetOcrValue((val*slope*e.Get_count())+8*ocr_motor[i]-7*val*OFFSET_SPEED_DOWN);
		}
		ramp_completeflag = true;
	}
	UART3TransmitData(e.Get_count());
	UART3Transmit('\t');
	UART3TransmitData((WHEEL_CIRCUMFERENCE*e.Get_count())/PPR_DISTANCE);
	UART3TransmitString("\n\r");

}

void Drive::reset_robvel()
{
	for(i=0;i<4;i++)
	{
		velocity_robot[i] = 0;
	}
}

void Drive::reset_joystick_data()
{
	for (j = 0 ; j<8 ;j++)
	rcvdata [j] = 0;
}

ISR(INT_VECT5)		//for calculating distance
{
	if(bit_is_set(ENCODER5_CHAPORTPIN,ENCODER5_CHBPIN))		//ENCODER_CHAPORTPIN,ENCODER_CHBPIN
	{
		e.dcrCount();
	}
	else
	e.incCount();

}
