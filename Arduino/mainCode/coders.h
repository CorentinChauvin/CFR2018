/**
  Code for handling coders
  Nantrobot, CFR 2018
**/

/*
  Coders wiring description (inverted signal mode) :
   _______________________________________   _________________________
  |   X 	| Gray | Green  | Blue  | Brown | |  X  | B  | A  | 0  | 5V |
  | White | Pink | Yellow |  Red  |   X   | | GND | B/ | A/ | 0/ | X  |
  |---------------------------------------| |-------------------------|

  Coders wiring :
    - A left  18
    - B left  19
    - A right 20
    - B right 21

  Correspondance pin-interruption (Arduino Mega) :
  http://www.arduino.org/learning/reference/attachinterrupt
*/

#ifndef CODERS_H
#define CODERS_H

#include "definitions.h"

// ----------------------------------------------------------------------------------
// INITIALISATION

// Pin
#define CODER_A_LEFT 18
#define CODER_B_LEFT 19
#define CODER_A_RIGHT 20
#define CODER_B_RIGHT 21

// Geometrical dimensions
const double L = 0.1175;
const double e = 2*L;         // distance between the coders wheels (in m)
const double Rcl = 0.0505;	 // radius of the left coder wheel (in m)
const double Rcr = 0.0505;	 // radius of the right coder wheel (in m)
//const double R = 3; //rayon d'une roue
const int npas = 1024;	     // number of pulses for each turn of the coder wheel (in quadruple precision)(?)
//const double a = 10.0;	     // distance point asservi axe des roues (sur l'axe de symétrie) en cm

// Coders global variables
volatile long int k_leftEncoder = 0;
volatile long int k_rightEncoder = 0;
long int last_k_leftEncoder = 0;
long int last_k_rightEncoder = 0;

// Robot variables
Pose robotPose = {0, 0, 0};
Twist robotTwist = {0, 0};


// ----------------------------------------------------------------------------------
// CODERS INTERRUPT FUNCTIONS

void updateRightEncoder()
{
  if (digitalRead(CODER_B_RIGHT))
    k_rightEncoder++;
  else
    k_rightEncoder--;
}

void updateLeftEncoder()
{
  if (digitalRead(CODER_B_LEFT))
    k_leftEncoder--;
  else
    k_leftEncoder++;
}


// ----------------------------------------------------------------------------------
// UPDATE ROBOT VARIABLES

void updatePose()
{
  float deltaLeft = (k_leftEncoder - last_k_leftEncoder) * (2*PI*Rcl) / npas;
  float deltaRight = (k_rightEncoder - last_k_rightEncoder) * (2*PI*Rcr) / npas;

  float deltaX = (deltaRight + deltaLeft) / 2;
  float deltaTheta = (deltaRight - deltaLeft) / e;

  robotPose.x += deltaX * cos(robotPose.theta + deltaTheta/2);
  robotPose.y += deltaX * sin(robotPose.theta + deltaTheta/2);
  robotPose.theta += deltaTheta;
}

void updateTwist(long int lastTime)
{
  long int dt = millis() - lastTime;

  long int deltaLeft = k_leftEncoder - last_k_leftEncoder;
  long int deltaRight = k_rightEncoder - last_k_rightEncoder;

  float omegaLeft = float(deltaLeft) * (2*PI) / (npas*dt*0.001);  // speed of the left wheel (in rad/s)
  float omegaRight = float(deltaRight) * (2*PI) / (npas*dt*0.001);

  robotTwist.v = (Rcr*omegaRight + Rcl*omegaLeft) / 2;
  robotTwist.omega = (Rcr*omegaRight - Rcl*omegaLeft) / e;
}

void updateCoders(long int lastTime)
{
  updatePose();
  updateTwist(lastTime);

  last_k_leftEncoder = k_leftEncoder;
  last_k_rightEncoder = k_rightEncoder;
}


// ----------------------------------------------------------------------------------
// DISPLAY FUNCTIONS

void displayPose(Pose P)
{
  Serial.print("x=");
  Serial.print(P.x);
  Serial.print(" ; y=");
  Serial.print(P.y);
  Serial.print(" ; theta=");
  Serial.print(int(P.theta * 180 / PI) % 360);
  Serial.println("°");
}

void displayTwist(Twist T)
{
  Serial.print("v=");
  Serial.print(T.v);
  Serial.print(" ; omega=");
  Serial.print(int(T.omega * 180 / PI) % 360);
  Serial.println("°/s");
}





#endif
