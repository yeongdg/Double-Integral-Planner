#include "../include/doubleIntegral.h"

int main() {
    double p_goal, a_goal, t1_goal;
    cin >> p_goal;
    cin >> a_goal;
    cin >> t1_goal;
    doubleIntegral DI(p_goal,a_goal,t1_goal);
    DI.TimeCalc();

    vector<double> x;
    vector<double> pos;
    vector<double> vel;
    vector<double> acc;
    double dt = 100;
    
    for (int i=0; i<20*dt; i++) {
        x.push_back(i/dt);
        DI.goalTraj(x[i]);
        pos.push_back(DI.getPos());
        vel.push_back(DI.getVel());
        acc.push_back(DI.getAcc());
    }

    plt::plot(x,pos);
    plt::plot(x,vel);
    plt::plot(x,acc);
    plt::grid(true);
    plt::xlabel("time [sec]");
    plt::ylabel("Position/Velocity/Acceleration");
    plt::legend();
    plt::save("fig1.png",640);
    plt::show();
}