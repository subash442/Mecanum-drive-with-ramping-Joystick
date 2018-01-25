/* 
* Encoder.h
*
* Created: 10/15/2017 5:57:05 PM
* Author: Subash Timilsina
*/


#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <avr/io.h>
#include <avr/interrupt.h>
#include "headers.h"

#ifndef F_CPU	
#define F_CPU		16000000UL
#endif

#define	 CLOCKWISE			1
#define	 ANTICLOCKWISE		0

#define  DELAYTIMEMS		20.0

#define	 PPR				250.0
#define  PPR_DISTANCE		200.0

//encoder 1

#define  ENCODER1_CHANNELA		D,3			
#define	 ENCODER1_CHANNELB		A,3			
#define	 ENCODER1_INT			INT3		

#define	 ENCODER1_CHAPORTPIN	 PINA		
#define	 ENCODER1_CHBPIN		 PA3			
#define	 INT1_ISC1				ISC31		
#define	 INT1_INTF				INTF3		

#define  INT_VECT1				INT3_vect	

//encoder 2

#define  ENCODER2_CHANNELA			D,0
#define	 ENCODER2_CHANNELB			A,0
#define	 ENCODER2_INT				INT0

#define	 ENCODER2_CHAPORTPIN		 PINA
#define	 ENCODER2_CHBPIN			 PA0		
#define	 INT2_ISC1					ISC01
#define	 INT2_INTF					INTF0

#define  INT_VECT2					INT0_vect

//encoder 3

#define  ENCODER3_CHANNELA		D,1
#define	 ENCODER3_CHANNELB		A,1
#define	 ENCODER3_INT			INT1

#define	 ENCODER3_CHAPORTPIN	 PINA
#define	 ENCODER3_CHBPIN		 PA1
#define	 INT3_ISC1				ISC11
#define	 INT3_INTF				INTF1

#define  INT_VECT3			INT1_vect

//encoder 4

#define  ENCODER4_CHANNELA	D,2
#define	 ENCODER4_CHANNELB	A,2
#define	 ENCODER4_INT		INT2

#define	 ENCODER4_CHAPORTPIN	 PINA
#define	 ENCODER4_CHBPIN		 PA2
#define	 INT4_ISC1			ISC21
#define	 INT4_INTF			INTF2

#define  INT_VECT4			INT2_vect

//encoder 5

#define  ENCODER5_CHANNELA	E,4
#define	 ENCODER5_CHANNELB	A,4
#define	 ENCODER5_INT		INT4

#define	 ENCODER5_CHAPORTPIN	 PINA
#define	 ENCODER5_CHBPIN		 PA4
#define	 INT5_ISC1			ISC41
#define	 INT5_INTF			INTF4

#define  INT_VECT5			INT4_vect


class Encoder
{
	private:
		uint8_t no;
		long int speed;
		long int pprcount;
	public:
	Encoder():speed(0),pprcount(0){};
	void Initialise(uint8_t x);
	void Init_Interrupts();
	void Init_Timers();
	void Calc_Speed();
	long int Get_Speed(){return speed;};
	void Set_Speed(int val){speed = val;};
	void incCount(){pprcount++;};
	void dcrCount(){pprcount--;};
	long int Get_count(){return pprcount;};
	void Set_count(int val){pprcount = val;};
}; 

#endif //__ENCODER_H__
