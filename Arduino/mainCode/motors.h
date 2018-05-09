/**
  Code for handling motors
  Nantrobot, CFR 2018
**/

/*
  Wirings of the L298 H bridge
  - E1 (purple)	pin 4
  - M1 (gray)		pin 6
  - E2 (White)	pin 3
  - A2 (blue)		pin 5
  - red wire		5V
*/

#ifndef MOTORS_H
#define MOTORS_H

#include "definitions.h"

#define ENABLE_LEFT 	3  // white
#define ENABLE_RIGHT 	4  // purple
#define MOTOR_LEFT 		5  // blue
#define MOTOR_RIGHT 	6  // gray

// Twist control
int Kp_v = 10;			// proportional gain for linear speed
int Kp_omega = 10;	// proportional gain for angular speed

void twistControl(Twist desiredTwist) {
	/** Control of the robot twist given a desired one
	**/

	// Computation of errors
	float e_v = desiredTwist.v - robotTwist.omega;
	float e_omega = desiredTwist.omega - robotTwist.omega;

	// Computation of motor speeds
	float omegaLeft  = ( Kp_v*e_v - (e*Kp_omega*e_omega)/2 ) / Rcl;
	float omegaRight = ( Kp_v*e_v + (e*Kp_omega*e_omega)/2 ) / Rcr;

	// Normalisation of motor speeds to avoid saturation
	float a = max(abs(omegaLeft), abs(omegaRight)) / 255;
	if (a < 1) a = 1;

	omegaLeft /= a;
	omegaRight /= a;

	// Find rotation direction of motors
	bool rotationDirectionLeft, rotationDirectionRight;

	if (omegaLeft < 0)
		rotationDirectionLeft = HIGH;
	else
		rotationDirectionLeft = LOW;

	if (omegaRight < 0)
		rotationDirectionRight = HIGH;
	else
		rotationDirectionRight = LOW;

	// Command of motors
	digitalWrite(MOTOR_LEFT, rotationDirectionLeft);
	digitalWrite(MOTOR_RIGHT, rotationDirectionRight);
	analogWrite(ENABLE_LEFT, omegaLeft);
	analogWrite(ENABLE_RIGHT, omegaRight);
}



#endif // MOTORS_H
