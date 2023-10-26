#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include "HIKsolver.h"
#include "Hmath.h"
#include "Htrajectory.h"
using namespace handsome;

#define TIME_STEP 0.05

IKSolver solver;
std::vector< SolutionVec > a;
SolutionVec path;
std::vector <double> v[6];

void readData(std::string filename)
{
    std::ifstream f(filename,std::ios::in);
    if(!f)
    {
        std::cout << "打开文件失败\n";
        return;
    }  
    std::string s;
    while(getline(f,s))
    {
        double tmp_pose[6];
        int p1 = s.find(','),p2 = s.find(',',p1 + 1);
        for(int i = 0; i < 6; ++i,p1 = p2,p2 = s.find(',',p1 + 1))
        {
            if(p2 == std::string::npos) p2 = s.length();
            double d = stod(s.substr(p1 + 1,p2 - p1 - 1));
            tmp_pose[i] = d;
            if(i >= 3)   tmp_pose[i] = tmp_pose[i] / 180 * M_PI;
        }
        a.push_back(solver.solve(tmp_pose));
    }
    f.close();
}

int main(int argc, char *argv[])
{
    readData("resource/path.txt");
    path = trajectory::choose_one_solution(a);
    path.push_back(Solution());//give the initial position
    double T = 0;
    Double_S s[6];
    Solution lstSolution;
    for(int i = 0; i < path.size(); ++i,T = 0)
    {
        for(int j = 0; j < 6; ++j)
        {
            s[j].setDestinyPos(lstSolution.theta[j],path[i].theta[j]);
            if(s[j].T > T) T = s[j].T;
        }
        lstSolution = path[i];
        for(int t = 0; t < T / TIME_STEP; ++t)
        {
            float angle[6];
            for(int i = 0; i < 6; ++i)  angle[i] = s[i].getPosAt(0.05 * t);
        }
        std::cout << "路径点" << i << "已达到\n";
    }
}