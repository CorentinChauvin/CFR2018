/*
  Headers which define usual constants and structures
*/

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define PI 3.14159265359


// Structure representing the pose of the robot
struct Pose {
  float x;		// in cm
  float y;		// in cm
  float theta;	// in degree, anticlockwise
};

// Structure representing the twist of the robot
struct Twist {
  float v;      // linear speed (in m/s)
  float omega;  // angular speed (in rad/s)
};




#endif
