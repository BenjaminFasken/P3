/*
    Dynamics.h - Library for performing dynamics on a robot model
    Made by group 369

*/
#ifndef Dynamics_h
#define Dynamics_h

#include "Arduino.h"

class Dynamics
{
  public:
    Dynamics();
    ~Dynamics();
    void eulerXYZ2T(double homo_trans[][4], double X, double Y, double Z, double rotZ, double rotY);
    void getInvKinJoints(double joints[4], double x, double y, double z, double pitch);
    void forwardKin(double joints[4], double cart_pos[4]);
  private:
    double pi = 3.1415926;
    // Pi divided by 2
    double piO2 = 1.5707963;
    double d1 = 0.23500;   
    double d2 = 0.21675;   
    double d3 = 0.10875;   
    double d4 = 0.03000;
    double d1S = d1*d1;
    double d2S = d2*d2;
    double d3S = d3*d3;
    //double t4W_inverse[] = {{1,0,0,0},{0,1,0,0},{0,0,1,-0.03},{0,0,0,1}};
};

#endif