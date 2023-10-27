#pragma once
#include <eigen3/Eigen/Dense>
#include <pybind11/pybind11.h>
#include <vector>
#include "Hmath.h"

namespace handsome
{
    class IKSolver
    {
    public:
        IKSolver();
        void printAns();
        //@brief for a given pose(x,y,z,Eular angles), compute the joint angle solutions, all the angles are in Radian value
        SolutionVec solve(double a[]);

    private:
        Eigen::Vector3d p;
        Eigen::Vector3d pose;
        Eigen::Vector3d n,o,g;
        double theta[6];
        SolutionVec sol;
        double angleLimit[6] = {200,90,120,150,150,180};
        double d[8] = {0,0.116,0.114,0,0,0.077,0.077,0.0855};
        double y[8] = {0,0,-0.054,0.185,0.17,0,0,-0.00002};
        double oxy,nxy,pxy,gxy,lx,ly;
        
    private:
        inline void init_argment();
        //@brief calculate theta1, also the entrance of computation
        void computeTheta1();
        void computeTheta6();
        void computeTheta5();
        void computeTheta23();
        void computeTheta4();
        //@brief transform eular angle to rotation matrix
        Eigen::Matrix3d Eular2Matrix(Eigen::Vector3d e_angle);
        void checkAns();
    };
}
