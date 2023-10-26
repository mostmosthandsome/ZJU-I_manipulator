#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include "HIKsolver.h"
using namespace std;
using namespace Eigen;

namespace handsome
{
    IKSolver::IKSolver()
    {
        for(int i = 0; i < 6; ++i)  theta[i] = 0;
    }

    Matrix3d IKSolver::Eular2Matrix(Vector3d e_angle)
    {
        Eigen::AngleAxisd rollAngle(AngleAxisd(e_angle(0),Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(AngleAxisd(e_angle(1),Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(AngleAxisd(e_angle(2),Vector3d::UnitZ()));
        Matrix3d rotation_matrix(rollAngle*pitchAngle*yawAngle);
        return rotation_matrix;
    }

    inline void IKSolver::init_argment()
    {
        Matrix3d T = Eular2Matrix(pose);
        n = T.block(0,0,3,1),o = T.block(0,1,3,1),g = T.block(0,2,3,1);
    }

    void IKSolver::computeTheta1()
    {
        double m = p[1] - y[7] - d[7] * g[1],n = y[7] * o[0] + d[7] * g[0] - p[0], k = d[5] + y[2];
        if(sqrt(m * m + n * n) < fabs(k) && fabs(k) - sqrt(m * m + n * n) > 1e-4)
        {
            cout << "Invalid pose!\n";
            return;
        }
        theta[0] = atan2(k,sqrt(fabs(m * m + n * n - k * k))) - atan2(m,n);
        computeTheta6();
        theta[0] = atan2(k,-sqrt(fabs(m * m + n * n - k * k))) - atan2(m,n);
        computeTheta6();
    }

    void IKSolver::computeTheta6()
    {
        oxy = o[0] * cos(theta[0]) + o[1] * sin(theta[0]),nxy = n[0] * cos(theta[0]) + n[1] * sin(theta[0]);
        gxy = g[0] * cos(theta[0]) + g[1] * sin(theta[0]),pxy = p[0] * cos(theta[0]) + p[1] * sin(theta[0]);
        lx = p[0] * cos(theta[0]) - y[7] * oxy - d[7] * gxy + p[1] * sin(theta[0]),ly = p[2] - d[1] - d[7] * g[2] - o[2] * y[7] - d[2];
        double kx = n[1] * cos(theta[0]) - n[0] * sin(theta[0]), ky = o[1] * cos(theta[0])  - o[0] * sin(theta[0]);
        theta[5] = - atan2(ky,kx);
        computeTheta5();
        theta[5] += M_PI;
        computeTheta5();
    }

    void IKSolver::computeTheta5()
    {
        double s5 = g[1] * cos(theta[0]) - g[0] * sin(theta[0]),c5;
        double c234 = o[2] * cos(theta[5]) + n[2] * sin(theta[5]),s234 = sin(theta[5]) * nxy + cos(theta[5]) * oxy;
        if(fabs(c234) < 1e-5)   c5 = - g[2] / s234;
        else    c5 = gxy / c234;
        theta[4] = atan2(s5,c5); 
        computeTheta23();
    }

    void IKSolver::computeTheta23()
    {
        double a = lx - d[6] * (cos(theta[5]) * oxy + sin(theta[5]) * nxy),b = ly - d[6] * (o[2] * cos(theta[5]) + n[2] * sin(theta[5]));
        double cos3 = (a * a + b * b - y[3] * y[3] - y[4] * y[4]) / (2 * y[3] * y[4]);
        if(fabs(cos3) > 1 + 1e-4)   return;
        else if(fabs(cos3) > 1)
        {
            if(cos3 > 0)    cos3 = 1;
            else    cos3 = -1;
        }
        theta[2] = acos(cos3);
        double ax = y[4] * cos(theta[2]) + y[3],ay = y[4] * sin(theta[2]),A = ax * ax + ay * ay;
        double sin2 = (ax * a - ay * b) / A,cos2 = (ay * a + ax * b) / A;
        theta[1] = atan2(sin2,cos2);
        computeTheta4();
        theta[2] = -theta[2];
        ax = y[4] * cos(theta[2]) + y[3],ay = y[4] * sin(theta[2]),A = ax * ax + ay * ay;
        sin2 = (ax * a - ay * b) / A,cos2 = (ay * a + ax * b) / A;
        theta[1] = atan2(sin2,cos2);
        computeTheta4();
    }

    void IKSolver::computeTheta4()
    {
        double pp = o[2] * cos(theta[5]) + n[2] * sin(theta[5]),qq = sin(theta[5]) * nxy + cos(theta[5]) * oxy;
        theta[3] = atan2(qq,pp) - theta[1] - theta[2];
        checkAns();
    }

    void IKSolver::checkAns()
    {
        for(int i = 0; i < 6; ++i)
        {
            theta[i] -= (int)((theta[i] + M_PI) / 2 / M_PI) * 2 * M_PI;;//ensure that all the angle is in range -pi to pi
            if(fabs(theta[i] / M_PI * 180) > angleLimit[i])  return;
        }
        sol.push_back(Solution(theta));
        
    }

    void IKSolver::printAns()
    {
        // for(auto s : sol)   printSolution(s);
    }

    SolutionVec IKSolver::solve(double a[])
    {
        sol.clear();
        for(int i = 0; i < 3; ++i)  p[i] = a[i],pose[i] = a[i + 3];
        // cout << "position = " << p.transpose() << ", pose = " << pose.transpose() << endl;
        init_argment();
        computeTheta1();
        return sol;
    }
}