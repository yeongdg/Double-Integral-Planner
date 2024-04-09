#include "double_integral_planner.hpp"
#include <matplotlibcpp.h>
#include <vector>
using std::vector;

namespace plt = matplotlibcpp;

int main() {
    double p_goal = 5.0;
    double a_max = 20.0;
    double v_max = 10.0;
    
    DoubleIntegralPlanner DIP;

    DIP.setGoalPos(p_goal);

    DIP.setAcc_max(a_max);
    DIP.setVel_max(v_max);
    
    DIP.TimeCalc();

    vector<double> t_vec;
    vector<double> pos_vec;
    vector<double> vel_vec;
    vector<double> acc_vec;
    double N = 1000;
    double t = 0;
    double dt = 0.01;
    
    for (int i = 0; i < N; i++) {
        t += dt;
        t_vec.push_back(t);
        DIP.goalTraj(t_vec[i]);
        pos_vec.push_back(DIP.getPos());
        vel_vec.push_back(DIP.getVel());
        acc_vec.push_back(DIP.getAcc());

        // cout<<vel_vec[i]<<endl;
    }

    plt::plot(t_vec,pos_vec);
    plt::plot(t_vec,vel_vec);
    plt::plot(t_vec,acc_vec);
    plt::grid(true);
    plt::xlabel("time [sec]");
    plt::ylabel("pos vel acc");
    plt::legend();
    // plt::save("fig1.png",640);
    plt::show();
}