#include <iostream>
#include <vector>
#include <cmath>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;
using namespace std;

class doubleIntegral
{
    public:
        doubleIntegral(double goal, double a_goal, double t1_goal);
        void TimeCalc();
        void goalTraj(double t);
        double getPos();
        double getVel();
        double getAcc();

    private:
        double a_max = 3;
        double v_max = 0;

        double p_goal = 30;

        double t_1 = 2;
        double t_2 = 0;
        double t_f = 0;

        double a_curr = 0;
        double v_curr = 0;
        double p_curr = 0;
};