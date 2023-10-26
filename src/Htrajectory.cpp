#include "Htrajectory.h"
#include <iostream>

namespace handsome
{
    namespace trajectory
    {
        SolutionVec choose_one_solution(std::vector< SolutionVec > &v)
        {
            int l = v.size();
            double f[101][10];
            int rec[101][10],ans_order[101];
            SolutionVec ans;
            //initialize all the value to -1, add an extra -1 to the end of every f[i] for the convienience of check
            for(int i = 0; i < l; ++i)
                for(int j = 0; j <= v[i].size(); ++j)
                    f[i][j] = -1;
            for(int j = 0; j < v[0].size(); ++j)    f[0][j] = 0;

            //DP
            for(int i = 1; i < l; ++i)
            {
                for(int j = 0; j < v[i].size(); ++j)
                {
                    for(int k = 0; k < v[i - 1].size(); ++k)
                    {
                        double tmp_value = f[i - 1][k] + dist(v[i][j],v[i - 1][k]);
                        if(f[i][j] == -1)   f[i][j] = tmp_value,rec[i][j] = k;// if f[i][j] hasn't been updated    
                        else if(f[i][j] > tmp_value)    f[i][j] = tmp_value,rec[i][j] = k;// choose a better solution
                    }
                }
            }

            //check final value and record
            double final_value = f[l - 1][0];   ans_order[l - 1] = 0;
            for(int i = 0; i < v[l - 1].size(); ++i)
                if(final_value > f[l - 1][i])
                    final_value = min(final_value,f[l - 1][i]),ans_order[l - 1] = i;
            if(final_value == -1)   throw "no solution found";

            //traceback the solution
            for(int i = l - 1; i > 0; --i)    ans_order[i - 1] = rec[i][ans_order[i]];
            for(int i = 0; i < l; ++i)    ans.push_back(v[i][ans_order[i]]);
            return ans;
        }
    }
}