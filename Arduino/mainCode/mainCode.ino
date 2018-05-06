/**
  Main code for the Arduino
  Nantrobot, CFR 2018
**/

#include "definitions.h"
#include "coders.h"
#include "motors.h"
#include "plier.h"

// Global variables
long int lastTime;
int k_display = 0;


void setup() {

  Serial.begin(9600);
  Serial.println("--- Start ---");

  // Opening pins
  pinMode(INPUT, CODER_A_LEFT); // coders
  pinMode(INPUT, CODER_B_LEFT);
  pinMode(INPUT, CODER_A_RIGHT);
  pinMode(INPUT, CODER_B_RIGHT);

  pinMode(MOTOR_RIGHT, OUTPUT);  // motors
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(ENABLE_LEFT, OUTPUT);

  plierServo.attach(SERVO_PLIER);	// servomotors

  // Interrupt declarations
  attachInterrupt(digitalPinToInterrupt(CODER_A_LEFT), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(CODER_A_RIGHT), updateRightEncoder, RISING);

  // Initialising variables
  lastTime = millis();
}


void loop() {
  openPlier();
  delay(5000);
  closePlier();
  delay(10000);

  /*updateCoders(lastTime);
  lastTime = millis();

  if (k_display >= 100) {
    displayTwist(robotTwist);
    k_display = 0;
  } else k_display++;

  delay(5);*/
}
