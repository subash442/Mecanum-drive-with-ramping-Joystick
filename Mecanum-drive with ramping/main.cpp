/*
 * Mecanum-drive with ramping.cpp
 *
 * Created: 1/10/2018 8:12:58 PM
 * Author : Subash Timilsina
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Drive.h"
#include "headers.h"

int main(void)
{
	sei();
	initUART2();
	initUART3();
	Drive Manual;
	Manual.init();
	
	while(1)
	{
		Manual.Move_the_Robot();
	}

	return 0;
}