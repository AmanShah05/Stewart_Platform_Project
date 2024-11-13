#include <cmath>
#include <iostream>
using namespace std;

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

int main(){
  double hz = 11;
  double nx = 0.342;
  double ny = 0.342;

  double pos[3] = {0, 0, 0};  // Define pos as an array with three elements
  double* vec = unitVec(0.3, 0);

  double testA = platformAngle(A , vec) * 180/PI;
  double testB = platformAngle(B , vec) * 180/PI;
  double testC = platformAngle(C , vec) * 180/PI;

  cout << "Platform Angles: " << testA << ", " << testB << ", " << testC << endl;

  double testA2 = motorAngle(platformAngle(A, vec), hz) * 180/PI;
  double testB2 = motorAngle(platformAngle(B, vec), hz) * 180/PI;
  double testC2 = motorAngle(platformAngle(C, vec), hz) * 180/PI;

  cout << "Motor Angles: " << testA2 << ", " << testB2 << ", " << testC2 << endl;
}