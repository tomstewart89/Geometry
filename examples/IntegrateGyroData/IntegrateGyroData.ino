#include <Geometry.h>

/*
 * This example sketch will show you how to integrate the rate measurements from a gyro sensor without having to worry about gimbal lock or
 * truncating the integrated value back to +-pi.
 */

// This is analogous to making a Z, followed by a Y and then an X axis rotation
Rotation orientation;
float xdot, ydot, zdot, dt = 1.0;

// Just a stub to gather some gyro-ish data
void GetGyroData(float &xdot, float &ydot, float &zdot)
{
  xdot = random(-1.5,1.5);
  ydot = random(-1.5,1.5);
  zdot = random(-1.5,1.5);
}

void setup() 
{
  randomSeed(analogRead(0)); // seed the random sequence used in GetGyroData
  Serial.begin(115200);
}

void loop()
{
  GetGyroData(xdot, ydot, zdot);

  // Apply the rotations in accordance with the rotation order for Euler angles
  orientation.RotateX(xdot * dt);
  orientation.RotateY(ydot * dt);
  orientation.RotateZ(zdot * dt);

  // Print the orientation to the console
  Serial << orientation << "\n";

  delay(1000);
}
