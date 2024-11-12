#include <cmath>
#include <iostream>
using namespace std;

//Calculation Variables
double nmag, nz, nx_b, nx_c;  //magnitude and z component of the normal vector
double x, y, z;   //generic variables for the components of legs A, B, and C
double mag;       //generic magnitude of the leg vector
double platform_angle;
double angle;     //generic angle for legs A, B, and C
double PI = 3.141592653589793;
double phi_a, phi_b, phi_c;

//constants
#define A 0
#define B 1
#define C 2

double platform_theta(int leg, double nx, double ny) { 
  //create unit normal vector
  nmag = sqrt(pow(nx, 2) + pow(ny, 2) + 1);  //magnitude of the normal vector
  nx /= nmag;
  ny /= nmag;
  nz = 1 / nmag;
  double check = pow(nx, 2) + pow(ny, 2) + pow(nz, 2);
  cout << "Check nmag :" << check << endl;

  switch (leg) {
    case A:
      platform_angle = atan(-nx/nz);
      break;
    case B:
      nx_b = (sqrt(3)/2)*ny-(1/2)*nx;
      platform_angle = atan(-nx_b/nz);
      break;
    case C:
      nx_c = (-sqrt(3)/2)*ny-(1/2)*nx;
      platform_angle = atan(-nx_c/nz);
      break;
  }
  return platform_angle * ( 180 / PI);
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
  double testA = platform_theta(A , 0.3, 0);
  double testB = platform_theta(B , 0.3, 0);
  double testC = platform_theta(C , 0.3, 0);

  cout << testA << " " << testB << " " << testC ;
  cout << "hi does committing work";
}


