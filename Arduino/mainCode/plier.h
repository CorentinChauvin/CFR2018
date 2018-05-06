/**
  Code for handling the plier
  Nantrobot, CFR 2018
**/

/*
  Wirings of the servomotor
  - yellow	pin 2
	- red 		+5V
	- brown		GND
*/


#ifndef PLIER_H
#define PLIER_H

#include <math.h>
#include <Servo.h>

// Pin
#define SERVO_PLIER	2  // yellow

// Servo constants
Servo plierServo;
const int minPlierAngle = 70;
const int maxPlierAngle = 140;


void setPlierAngle(int angle, int delayTime=15)
{
	/**
	  * Set the plier servomotor angle
		*
		* delayTime is the time (in ms) to wait after writing the servo angle
	**/
	angle = max(minPlierAngle, min(maxPlierAngle, angle));	// limit the angle in the good interval

	plierServo.write(angle);
	delay(delayTime);
}


void openPlier()
{
	setPlierAngle(70);
}


void closePlier()
{
	setPlierAngle(140);
}



#endif  // PLIER_H
