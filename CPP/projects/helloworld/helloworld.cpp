#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>

using namespace std;

void eulerXYZ2T(double homo_trans[4][4], double cart_pos[6]){
    double X = cart_pos[0],Y = cart_pos[1],Z = cart_pos[2],r = cart_pos[3],p = cart_pos[4],y = cart_pos[5];
        
    // double temp[4][4] = {
    //     {cos(y)*cos(p), 0, 0, X},
    //     {sin(y)*cos(p), 0, 0, Y},
    //     {-sin(y)      , 0, 0, Z},
    //     {0            , 0, 0, 1}};
    double temp[4][4] = {
        {cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r), X},
        {sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r), Y},
        {-sin(p),       cos(p)*sin(r),                      cos(p)*cos(r),                      Z},
        { 0, 0, 0, 1}};
    copy(&temp[0][0], &temp[0][0]+16, &homo_trans[0][0]);
}
void print_matrix(double matrix[4][4]){
    for(int x=0;x<4;x++)  // loop 3 times for three lines
    {
        for(int y=0;y<4;y++)  // loop for the three elements on the line
        {

            cout<<matrix[x][y] << " ";  // display the current element out of the array
        }
    cout<<endl;  // when the inner loop is done, go to a new line
    }
}

void mult_matrix(double m1[][4], double m2[][4], double out[][4]){
    double temp[4][4];
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            double q = 0;
            for (int k = 0; k < 4; k++){
                double t = m1[i][k] * m2[k][j];
                q += t;
                // if (t == 0){t = 0;}
            }
            temp[i][j] =q;
        }
    }
    copy(&temp[0][0], &temp[0][0]+16, &out[0][0]);
}

void T2EULERXYZ(double t[4][4], double cart_pos[6]){
    double pi = 3.14159625;

    
    double X = t[0][3];
    double Y = t[1][3];
    double Z = t[2][3];

    double r, p, y;
    p =atan2(-t[2][0],sqrt(t[0][0]*t[0][0]+t[1][0]*t[1][0]));
    if (abs(abs(p)-pi/2)<0.00001){
        y=0;
        r=p/abs(p)*atan2(t[0][1],t[1][1]);
    }else{
        y=atan2(t[1][0]/cos(p),t[0][0]/cos(p));
        r=atan2(t[2][1]/cos(p),t[2][2]/cos(p));
    }
    double temp[6] = {X,Y,Z,r,p,y};
    copy( temp, temp+6, cart_pos );
}

void inv_kin(double joints[4], double cart_pos[6]){
        /*yaw is θ1*/
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
        double X = cart_pos[0],Y = cart_pos[1],Z = cart_pos[2],r = cart_pos[3],p = cart_pos[4],y = cart_pos[5];
        double Target0W[4][4];
        eulerXYZ2T(Target0W, cart_pos);
        double inv_4W[4][4] = {
            {1, 0, 0, -d4},
            {0, 0, -1, 0},
            {0, 1, 0, 0},
            {0, 0, 0, 1}};
        
        double Target04[4][4];
        mult_matrix(Target0W, inv_4W, Target04);
        // dont know whether it is the right way round
        X = Target04[0][3];
        Y = Target04[1][3];
        Z = Target04[2][3];

        double xS = X*X, yS = Y*Y, h = (Z-d1), hS = h*h;
        double dS = X*X + Y*Y + (Z-d1)*(Z-d1);
        double d = sqrt(dS);
        double theta_1 = atan2(Y,X);
        double theta_2 = atan2(Z-d1, sqrt(xS+yS)) + acos((d2S + dS - d3S)/(2*d2*d));
        double theta_3 = acos((d2S + d3S - dS) / (2 * d2 * d3)) - pi;
        double theta_4 = theta_2 + theta_3 - p;
        double temp[4] = {theta_1, theta_2, theta_3, theta_4};
        copy( temp, temp+4, joints );

    }

void fwd_kin(double joints[4], double cart_pos[6]){
    double theta_1 = joints[0], theta_2 = joints[1], theta_3 = joints[2], theta_4 = joints[3];
    double d_1 = 0.235, d_2 = 0.21675, d_3 = 0.10875, d_4 = 0.03;
    /*      David hartenberg params
    *   Joint   A       α       D       θ
    *   1       0       0       d1      θ1
    *   2       0       90      0       θ2
    *   3       d2      0       0       θ3
    *   4       d3      0       0       θ4
    *   5       d4      -90     0       0       DEN HER SKAL LAVES OM!!!!!
    */
    double T[4][4] = {
	{(cos(theta_1)*cos(theta_2)*cos(theta_3) - cos(theta_1)*sin(theta_2)*sin(theta_3))*cos(theta_4) + (-cos(theta_1)*cos(theta_2)*sin(theta_3) - cos(theta_1)*sin(theta_2)*cos(theta_3))*sin(theta_4), -sin(theta_1), -(cos(theta_1)*cos(theta_2)*cos(theta_3) - cos(theta_1)*sin(theta_2)*sin(theta_3))*sin(theta_4) + (-cos(theta_1)*cos(theta_2)*sin(theta_3) - cos(theta_1)*sin(theta_2)*cos(theta_3))*cos(theta_4), ((cos(theta_1)*cos(theta_2)*cos(theta_3) - cos(theta_1)*sin(theta_2)*sin(theta_3))*cos(theta_4) + (-cos(theta_1)*cos(theta_2)*sin(theta_3) - cos(theta_1)*sin(theta_2)*cos(theta_3))*sin(theta_4))*d_4 + (cos(theta_1)*cos(theta_2)*cos(theta_3) - cos(theta_1)*sin(theta_2)*sin(theta_3))*d_3 + cos(theta_1)*cos(theta_2)*d_2},
	{(sin(theta_1)*cos(theta_2)*cos(theta_3) - sin(theta_1)*sin(theta_2)*sin(theta_3))*cos(theta_4) + (-sin(theta_1)*cos(theta_2)*sin(theta_3) - sin(theta_1)*sin(theta_2)*cos(theta_3))*sin(theta_4), cos(theta_1), -(sin(theta_1)*cos(theta_2)*cos(theta_3) - sin(theta_1)*sin(theta_2)*sin(theta_3))*sin(theta_4) + (-sin(theta_1)*cos(theta_2)*sin(theta_3) - sin(theta_1)*sin(theta_2)*cos(theta_3))*cos(theta_4), ((sin(theta_1)*cos(theta_2)*cos(theta_3) - sin(theta_1)*sin(theta_2)*sin(theta_3))*cos(theta_4) + (-sin(theta_1)*cos(theta_2)*sin(theta_3) - sin(theta_1)*sin(theta_2)*cos(theta_3))*sin(theta_4))*d_4 + (sin(theta_1)*cos(theta_2)*cos(theta_3) - sin(theta_1)*sin(theta_2)*sin(theta_3))*d_3 + sin(theta_1)*cos(theta_2)*d_2},
	{(cos(theta_2)*sin(theta_3) + cos(theta_3)*sin(theta_2))*cos(theta_4) + sin(theta_4)*(cos(theta_2)*cos(theta_3) - sin(theta_2)*sin(theta_3)), 0, -sin(theta_4)*(cos(theta_2)*sin(theta_3) + cos(theta_3)*sin(theta_2)) + cos(theta_4)*(cos(theta_2)*cos(theta_3) - sin(theta_2)*sin(theta_3)), ((cos(theta_2)*sin(theta_3) + cos(theta_3)*sin(theta_2))*cos(theta_4) + sin(theta_4)*(cos(theta_2)*cos(theta_3) - sin(theta_2)*sin(theta_3)))*d_4 + (cos(theta_2)*sin(theta_3) + cos(theta_3)*sin(theta_2))*d_3 + sin(theta_2)*d_2 + d_1},
	{0, 0, 0, 1}};
    print_matrix(T);
    double temp[6] = {0,0,0,0,0,0};
    T2EULERXYZ(T, temp);
    copy( temp, temp+6, cart_pos );

}

int main()
{

    double pi = 3.14159265;

    double joints[4] = {pi/4, pi/4, -pi/4, pi/4}, cart_pos[6] = {0,0,0,1,0,0};
    fwd_kin(joints, cart_pos);
    for (int i = 0; i < 6; i++){
        cout << cart_pos[i] << "    ";
    } cout << endl;
    
    double ijoints[4] = {1,1,1,1}, icart_pos[6] = {0.200273,    0.200273,    0.409479,    0,    -0.785398,    0.785398};
    inv_kin(ijoints, icart_pos);
    for (int i = 0; i < 4; i++){
        cout << ijoints[i] << "    ";
    } cout << endl;
}