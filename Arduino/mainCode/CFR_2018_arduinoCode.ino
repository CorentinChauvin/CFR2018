/**
  Main code for the Arduino
  Nantrobot, CFR 2018
**/

#include "definitions.h"
#include "coders.h"
#include "motors.h"

// Global variables
long int lastTime;
int k_display = 0;

void setup() {

  Serial.begin(9600);
  Serial.println("--- Start ---");

  // Opening pins
  pinMode(INPUT, PIN_A_LEFT); // coders
  pinMode(INPUT, PIN_B_LEFT);
  pinMode(INPUT, PIN_A_RIGHT);
  pinMode(INPUT, PIN_B_RIGHT);

  pinMode(MOTOR_RIGHT, OUTPUT);  // motors
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(ENABLE_LEFT, OUTPUT);

  // Interrupt declarations
  attachInterrupt(digitalPinToInterrupt(PIN_A_LEFT), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_A_RIGHT), updateRightEncoder, RISING);

  // Initilising variables
  lastTime = millis();
}

void loop() {

  updateCoders(lastTime);
  lastTime = millis();

  if (k_display >= 100) {
    displayTwist(robotTwist);
    k_display = 0;
  } else k_display++;

  delay(5);
}
