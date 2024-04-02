#include "doubleIntegral.h"

doubleIntegral::doubleIntegral(double p_goal, double a_goal, double t1_goal)
:p_goal(p_goal), a_max(a_goal), t_1(t1_goal) {}


void doubleIntegral::TimeCalc() {
    v_max = a_max*t_1;
    t_2 = (p_goal/v_max);
    t_f = t_1 + t_2;

    if (t_1 >= t_2) {
        t_1 = sqrt(p_goal/a_max);
        t_2 = t_1;
        t_f = 2*t_1;
        v_max = a_max*t_1;
        cout << "Double Integral:: Bang-Bang" << endl;
    }
    else {cout << "Double Integral:: Bang-off-Bang" << endl;}
}
void doubleIntegral::goalTraj(double t) {
    if (t < t_1) {
        // cout << "Range:: t_0 ~ t_1" << endl;
        a_curr = a_max;
        v_curr = a_curr*t;
        p_curr = 0.5*v_curr*t;
    }
    else if (t < t_2) {
        // cout << "Range:: t_1 ~ t_2" << endl;
        a_curr = 0;
        v_curr = a_max*t_1;
        p_curr = 0.5*v_max*t_1 + v_curr*(t-t_1);
    }
    else if (t < t_f) {
        // cout << "Range:: t_2 ~ t_f" << endl;
        a_curr = -a_max;
        v_curr = a_max*t_1 + a_curr*(t-t_2);
        p_curr = 0.5*v_max*t_1 + v_max*(t_2-t_1) + 0.5*(v_max+v_curr)*(t-t_2);
    }
    else {
        a_curr = 0;
        v_curr = 0;
        p_curr = 0.5*v_max*t_1 + v_max*(t_2-t_1) + 0.5*v_max*(t_f-t_2);
    }
}

double doubleIntegral::getPos() {return p_curr;}
double doubleIntegral::getVel() {return v_curr;}
double doubleIntegral::getAcc() {return a_curr;}