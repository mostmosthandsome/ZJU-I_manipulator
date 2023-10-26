#include "Hmath.h"
#include <iostream>

namespace handsome
{
    double dist(Solution &s1,Solution &s2)
    {
        double s = 0,e = 0;
        for(int i = 0; i < 6; ++i)   e = fabs(s1.theta[i] - s2.theta[i]),s += e * e;
        return sqrt(s);
    }

    void printSolution(Solution &x)
    {
        for(int i = 0; i < 6; ++i)
        {
            if(i)   putchar(',');
            std::cout << x.theta[i];
        }
        putchar(10);
    }

    double min(double x,double y)
    {
        return x > y ? y : x;
    }

    double max(double x,double y)
    {
        return x > y ? x : y;
    }

    Double_S::Double_S():v0(0),v1(0)
    {
        a_m = 500.0 / 180 * M_PI,v_m = 100.0 / 180 * M_PI, jerk = 5 * a_m;
    }

    Double_S::Double_S(double x,double y):q0(x),q1(y),v0(0),v1(0)
    {
        a_m = 500.0 / 180 * M_PI,v_m = 100.0 / 180 * M_PI;
        jerk = 20 * a_m;
        if(q0 > q1) v_m = -v_m,jerk = -jerk,a_m = -a_m;
        calc();
    }

    void Double_S::setDestinyPos(double x,double y)
    {
        q0 = x,q1 = y;
        a_m = 500.0 / 180 * M_PI,v_m = 100.0 / 180 * M_PI,jerk = 20 * a_m;
        if(q0 > q1) v_m = -fabs(v_m),jerk = -fabs(jerk),a_m = -fabs(a_m);
        calc();
    }

    bool Double_S::check(double a_max)
    {
        a_m = a_max, Tj1 = Tj2 = a_m / jerk;
        double delta = pow(a_m,4) / jerk / jerk + 2 * (v0 * v0 + v1 * v1) + a_m * (4 * (q1 - q0) - 2 * a_m / fabs(jerk) * (v0 + v1));
        Ta = (a_m * a_m / fabs(jerk) - 2 * v0 + sqrt(delta)) / 2 / fabs(a_m),Td = (a_m * a_m / fabs(jerk) - 2 * v1 + sqrt(delta)) / 2 / fabs(a_m);
        if(min(Ta,Td) < 2*Tj1)  return false;
        return true;
    }

    void Double_S::calc()
    {   
        // std::cout << (v_m - v0) * jerk << "," << a_m * a_m << std::endl;
        if((v_m - v0) * jerk < a_m * a_m)   Tj1 = sqrt((v_m - v0) / jerk),Ta = 2 * Tj1,a_lim1 = jerk * Tj1;
        else    Tj1 = a_m / jerk, Ta = Tj1 + (v_m - v0) / a_m,a_lim1 = a_m;
        // std::cout << Tj1 << "," << Ta << std::endl;
        if((v_m - v1) * jerk < a_m * a_m)   Tj2 = sqrt((v_m - v1) / jerk),Td = 2 * Tj2,a_lim2 = -jerk * Tj2;
        else    Tj2 = a_m / jerk,Td = Tj2 + (v_m - v1) / a_m,a_lim2 = -a_m;
        // std::cout << Tj2 << "," << Td << std::endl;
        Tv = (q1 - q0) / v_m - Ta / 2 * (1 + v0 / v_m) - Td / 2 * (1 + v1 / v_m);
        // std::cout << "q0 = " << q0 << ",q1 = " << q1 << std::endl;
        // std::cout << "Tv = " << Tv << std::endl;
        v_lim = v0 + a_lim1 * (Ta - Tj1);
        if(Tv < 0)
        {
            double l = 0,r = a_m,mid = (l + r) / 2;
            while(fabs(r - l) > 1e-6)
            {
                if(check(mid))  l = mid;
                else    r = mid;
                mid = (l + r) / 2;
            }
            a_lim1 = jerk * Tj1,a_lim2 = -jerk * Tj2,Tv = 0;
        }
        //calculate the speed in every part
        //acceleration increase from 0
        qj1 = q0 + v0 * Tj1 + jerk * Tj1 * Tj1 * Tj1 / 6,vj1 = v0 + a_lim1 * Tj1 / 2,Taa = Ta - 2 * Tj1;
        //constant acceleration a_lim1
        qa = qj1 + vj1 * Taa + a_lim1 * Taa * Taa / 2,va = vj1 + a_lim1 * Taa;
        //acceleration decrease to 0
        vj2 = va + a_lim1 * Tj1 / 2,qj2 = qa + vj2 * Tj1 - jerk * Tj1 * Tj1 * Tj1 / 6;
        //constant velocity v_lim
        v_lim = vj2,qv = qj2 + vj2 * Tv;
        //acceleration decrease
        qj3 = qv + v_lim * Tj2 - jerk * Tj2 * Tj2 * Tj2 / 6,vj3 = v_lim + a_lim2 * Tj2 / 2,Tdd = Td - 2 * Tj2;
        //constant acceleration a_lim2
        qd = qj3 + vj3 * Tdd + a_lim2 * Tdd * Tdd / 2,vd = vj3 + a_lim2 * Tdd;
        T = Ta + Tv + Td;
    }
    double Double_S::getPosAt(double t)
    {
        //acceleration increase from 0
        if(t < Tj1) return q0 + v0 * t + jerk * t * t * t / 6;
        t -= Tj1;
        //acceleration = a_lim1, jerk = 0
        if(t < Taa)    return qj1 + vj1 * t + a_lim1 * t * t / 2;
        t -= Taa;
        //acceleration decrease to 0
        if(t < Tj1)  return qa + va * t + a_lim1 * t * t / 2 - jerk * t * t * t / 6;
        t -= Tj1;
        if(t < Tv)  return qj2 + vj2 * t;
        t -= Tv;
        if(t < Tj2) return qv + vj2 * t - jerk * t * t * t / 6;
        t -= Tj2;
        if(t < Tdd) return qj3 + vj3 * t + a_lim2 * t * t / 2;
        t -= Tdd;
        if(t < Tj2) return qd + vd * t + a_lim2 * t * t / 2 + jerk * t * t * t / 6;
        return q1;
    }
    double Double_S::getVelocityAt(double t)
    {
        if(t < Tj1) return v0 + jerk * t * t / 2;
        t -= Tj1;
        //acceleration = a_lim1, jerk = 0
        if(t < Taa)    return vj1 + a_lim1 * t;
        t -= Taa;
        //acceleration decrease to 0
        if(t < Tj1)  return va + a_lim1 * t - jerk * t * t / 2;
        t -= Tj1;
        if(t < Tv)  return v_lim;
        t -= Tv;
        if(t < Tj2) return v_lim - jerk * t * t / 2;
        t -= Tj2;
        if(t < Tdd) return vj3 + a_lim2 * t;
        t -= Tdd;
        if(t < Tj2) return vd + a_lim2 * t + jerk * t * t / 2;
        return v1;
    }
}
