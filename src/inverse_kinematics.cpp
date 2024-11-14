#include <cmath>
#include <iostream>
using namespace std;
#include <Wire.h>



//Calculation Variables
double nmag, nz, nx_b, nx_c;  //magnitude and z component of the normal vector
double x, y, z;   //generic variables for the components of legs A, B, and C
double mag;       //generic magnitude of the leg vector
double angle;     //generic angle for legs A, B, and C
double PI = 3.141592653589793;
double phi_a, phi_b, phi_c;

//constants
#define A 0
#define B 1
#define C 2

//all dimensions are in cm
double R = 15; //platform radius
double RM = 15; //radial distance from base centre to motors
double L1 = 6.719; //bottom link length
double L2 = 9.4592; //top link length

double* unitVec(double propX, double propY) {
  double magn = sqrt(pow(propX, 2) + pow(propY, 2) + 1); //magnitude

  double* vec = new double[3];
  vec[0] = propX/magn;
  vec[1] = propY/magn;
  vec[2] = 1/magn;

  //double check = pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2);
  //cout << "Check unit vector magnitude: " << check << endl;

  return vec;
}

double platformAngle(int leg, double vec[]) { 
  //take normal vector components
  double nx = vec[0], ny = vec[1], nz = vec[2];
  
  //determine normal radial component based on chosen leg
  double nr;
  switch (leg) {
    case A:
      nr = nx;
      break;
    case B:
      nr = sqrt(0.75)*ny-0.5*nx;
      break;
    case C:
      nr = -sqrt(0.75)*ny-0.5*nx;
      break;
  }

  //return platform angle of incline for chosen leg, in radians
  return atan(-nr/nz);
}

double motorAngle(double inclineAngle, double height) {
  //determine position of sphere joint in (r,z) coordinates
  double platform_r = R * cos(inclineAngle);
  double platform_z = R * sin(inclineAngle) + height;

  //perform some intermediate calculations for reused variables
  double dSq = pow(platform_r - RM, 2) + pow(platform_z, 2);
  double D = 1 + (pow(L1, 2) - pow(L2, 2))/dSq;
  double E = sqrt(pow(L1, 2)/dSq - 0.25*pow(D, 2));

  //determine position of pin joint in (r,z) coordinates
  double joint_r = RM + 0.5*D*(platform_r - RM) + platform_z*E;
  double joint_z = 0.5*D*platform_z + (RM - platform_r)*E;

  //determine angle of motor based on pin joint position, in radians
  return atan(joint_z / (joint_r - RM));
}

// double leg_theta(int leg, double nx, double ny) { 
//   phi_a = platform_theta(0,nx,ny);
//   phi_b = platform_theta(1,nx,ny);
//   phi_c = platform_theta(2,nx,ny);
//   
// }

void setup() {
  Serial.begin(9600);
  Wire.begin(0x8);  // Initialize I2C with address 0x8
  Wire.onReceive(receiveEvent);  // Register the I2C receive event
}

// I2C receive event function
void receiveEvent(int howMany) {
  if (howMany >= 8) {  // Expect 8 bytes: 4 bytes for x and 4 bytes for y (float)
    byte x_bytes[4], y_bytes[4];

    // Read the 4 bytes for x and 4 bytes for y
    for (int i = 0; i < 4; i++) {
      x_bytes[i] = Wire.read();
      y_bytes[i] = Wire.read();
    }

    // Convert bytes to floats
    float* x_ptr = (float*)x_bytes;
    float* y_ptr = (float*)y_bytes;

    // Get the x and y values as floats
    x = *x_ptr;
    y = *y_ptr;

    // Calculate angles based on received x and y
    double* vec = unitVec(x, y);
    double platformA = platformAngle(0, vec);
    double platformB = platformAngle(1, vec);
    double platformC = platformAngle(2, vec);

    double motorA = motorAngle(platformA, hz) * 180 / PI;
    double motorB = motorAngle(platformB, hz) * 180 / PI;
    double motorC = motorAngle(platformC, hz) * 180 / PI;

    // Send motor angles to Serial Monitor
    Serial.print("Motor Angles: ");
    Serial.print(motorA); Serial.print(", ");
    Serial.print(motorB); Serial.print(", ");
    Serial.println(motorC);

    delete[] vec;  // Clean up dynamic memory
  }
}

// Main loop function
void loop() {
  // Idle, waiting for data
}