#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include "HIKsolver.h"
#include "Hmath.h"
#include "Htrajectory.h"


#define TIME_STEP 0.05

namespace handsome
{
    class HSolver
    {
    public:
        HSolver():T_total(0)
        {
            
        }
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
    public:
        double T_total;
    public:
        void process()
        {
            path = trajectory::choose_one_solution(a);
            path.push_back(Solution());
            Solution lstSolution;
            double T = 0;
            for(int i = 0; i < path.size(); ++i,T = 0)
            {
                for(int j = 0; j < 6; ++j)
                {
                    curveVec[j].push_back(Double_S(lstSolution.theta[j],path[i].theta[j]));
                    if(curveVec[j][i].T > T) T = curveVec[j][i].T;
                }
                T_total += T,Times.push_back(T),lstSolution = path[i];
                std::cout << "路径点" << i << "已达到\n";
            }
        }

        private:
        IKSolver solver;
        std::vector< SolutionVec > a;
        SolutionVec path;
        std::vector <double> v[6];
        std::vector<Double_S> curveVec[6];
        std::vector<double> Times;
    };
}




PYBIND11_MODULE(handsome,m)
{
    m.doc() = "the developer is handsome";
    pybind11::class_<handsome::Double_S>(m,"Double_S")
    .def(pybind11::init<double, double>())
    .def("getPosAt", &handsome::Double_S::getPosAt,"get the position of double S trajectory at time t")
    .def("setDestinyPos",&handsome::Double_S::setDestinyPos,"set the position of Double_S curve");
    pybind11::class_<handsome::HSolver>(m,"HSolver")
    .def(pybind11::init())
    .def("readData",&handsome::HSolver::readData,"read pose data from file")
    .def("process",&handsome::HSolver::process,"process the data");
}