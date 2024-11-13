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

double* unitVec(double propX, double propY) {
  double magn = sqrt(pow(propX, 2) + pow(propY, 2) + 1); //magnitude

  double* vec = new double[3];
  vec[0] = propX/magn;
  vec[1] = propY/magn;
  vec[2] = 1/magn;

  double check = pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2);
  cout << "Check unit vector magnitude: " << check << endl;

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

  //return platform angle of incline for chosen leg, in degrees
  return atan(-nr/nz) * (180/PI);
}


// double leg_theta(int leg, double nx, double ny) { 
//   phi_a = platform_theta(0,nx,ny);
//   phi_b = platform_theta(1,nx,ny);
//   phi_c = platform_theta(2,nx,ny);
//   
// }

int main(){
  double hz = 105;
  double nx = 0.342;
  double ny = 0.342;

  double pos[3] = {0, 0, 0};  // Define pos as an array with three elements
  double* vec = unitVec(0.3, 0.3);

  double testA = platformAngle(A , vec);
  double testB = platformAngle(B , vec);
  double testC = platformAngle(C , vec);

  cout << testA << " " << testB << " " << testC ;
}


