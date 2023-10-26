#pragma once
#include <cmath>
#include <vector>
#include <string.h>

namespace handsome
{
    class Solution
    {
    public:
        double theta[6];
        Solution()
        {
            for(int i = 0; i < 6; ++i)  theta[i] = 0;
        }
        Solution(double a[])
        {
            for(int i = 0; i < 6; ++i)  theta[i] = a[i];
        }
    };
    using SolutionVec = std::vector<Solution>;
    //@brief calculate the distance 
    double dist(Solution &s1,Solution& s2);
    double min(double x,double y);
    double max(double x,double y);
    void printSolution(Solution &x);

    class Double_S
    {
    public:
        Double_S(double q0,double q1);
        Double_S();

    public:
        void setDestinyPos(double q0, double q1);
        double getPosAt(double t);
        double getVelocityAt(double t);

    public:
        double Tj1,Tj2,Ta,Td;
        double Tv,T;

    private:
        double q0,q1,v0,v1,a_m,v_m;
        double jerk;
        double a_lim1,v_lim,a_lim2;
    private:
        double qj1,vj1,qj2,vj2;
        double Taa,qa,va;
        double qv; 
        double qj3,vj3;
        double Tdd,qd,vd;
    private:
        void calc();
        bool check(double a_max);
    };
}