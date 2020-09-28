#include "Arduino.h"
#include "Dynamics.h"

Dynamics::Dynamics()
{

}

Dynamics::~Dynamics()
{
}

void Dynamics::getInvKinJoints(double joints[4], double x, double y, double z, double pitch){
        /*yaw is θ1*/
        double xS = x*x;
        double yS = y*y;
        double h = (z-1);
        double hS = h*h;
        double yaw = atan2(y,x);
        double Target04[4][4];
        Dynamics::eulerXYZ2T(Target04, x, y, z, yaw, pitch);

        // dont know whether it is the right way round
        x = Target04[1][4];
        y = Target04[2][4];
        z = Target04[3][4];

        double rS = xS + yS + hS;
        double r = sqrt(rS);
        double theta_2 = atan(h/sqrt(xS+yS)) + acos((d2S + rS - d3S)/(2*d2*r));
        double theta_3 = acos((d2S + d3S - rS) / (2 * d2 * d3)) - pi;
        double theta_4 = theta_2 + theta_3 - pitch + piO2;
        double joints_temp[4] = {yaw, theta_2, theta_3, theta_4};
        joints = joints_temp;

    }

void Dynamics::eulerXYZ2T(double homo_trans[][4], double X, double Y, double Z, double rotZ, double rotY){
        double homo_trans_temp[4][4] = {
        {cos(rotZ)*cos(rotY), 0, 0, X},
        {sin(rotZ)*cos(rotY), 0, 0, Y},
        {-sin(rotY)         , 0, 0, Z-0.03},
        {0                  , 0, 0, 1}};
        homo_trans = homo_trans_temp;
    }

void Dynamics::forwardKin(double joints[4], double cart_pos[4]){
    /*Insert denavit hartenberg params*/
}