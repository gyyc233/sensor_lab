#include "dubins_curve.h"

Dubins::Dubins(){}

Dubins::~Dubins(){}

double Dubins::PI2PI(double angle) {
    return fmod(angle+PI,2*PI)-PI;
}

vector<double> Dubins::polar(double x, double y){
  double r=sqrt(x*x+y*y);
  double theta=atan2(y,x);

  return {r,theta};
}

double Dubins::mod2Pi(double theta){
  return theta - 2.0*PI* floor(theta/2.0/PI);
}

Path Dubins::left_straight_left(double alpha, double beta, double d){
  double s_a,s_b,c_a,c_b,,c_a_b;
  s_a=sin(alpha);
  s_b=sin(beta);
  c_a=cos(alpha);
  c_b=cos(beta);
  c_a_b=cos(alpha-beta);

  double tmp=atan2((c_b-c_a)/(d+s_a-s_b));
  double tmp_sqrt=

  Path path;
  path.mode="LSL";

  path.t=mod2Pi(-1*alpha+tmp);
  

}
