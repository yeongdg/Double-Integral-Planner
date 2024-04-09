#include "wheel_speed_mode.hpp"
#include <matplotlibcpp.h>
#include <vector>
using std::vector;
using std::cout;
using std::endl;
using std::to_string;

namespace plt = matplotlibcpp;

int main() {
    string file_name = "../config/test.yaml";
    TRAJ_GEN *traj_gen_ptr = new TRAJ_GEN(file_name);
    WHEEL_SPEED_MODE *panWheel_mode_ptr;
    WHEEL_SPEED_MODE *liftWheel_mode_ptr;

    vector<double> t_vec;
    vector<double> pos_vec[6];
    vector<double> vel_vec[6];
    vector<double> acc_vec[6];
    vector<double> panWheel_vec[3];
    vector<double> liftWheel_vec[3];
    
    TRJ_DATA traj_data;

    int N = 800;
    double t = 0;
    double dt = 0.01;

    // goal_pos[0:2]::(PAN)leg123
    // goal_pos[3:6]::(LIFT)leg123
    double goal_pos[6] = {5,15,25,35,45,55};

    LIFT_SENSOR qLift_sensorVal;

    for(int i=0; i<6; i++) {
        traj_gen_ptr->setGoalPos(goal_pos[i],i);
    }

    for (int i=0; i<N; i++) {
        t_vec.push_back(t);

        for (int j=0; j<6; j++) {
            traj_gen_ptr->getTraj(traj_data,j,t_vec[i]);
            pos_vec[j].push_back(traj_data.p_curr[j]);
            vel_vec[j].push_back(traj_data.v_curr[j]);
            acc_vec[j].push_back(traj_data.a_curr[j]);
        }

        panWheel_mode_ptr = new WHEEL_SPEED_MODE(PAN_WHEEL,traj_data,qLift_sensorVal);
        liftWheel_mode_ptr = new WHEEL_SPEED_MODE(LIFT_WHEEL,traj_data,qLift_sensorVal);
        for (int j=0; j<3; j++) {
            panWheel_vec[j].push_back(panWheel_mode_ptr->returnOmegaWheel(j));
            liftWheel_vec[j].push_back(liftWheel_mode_ptr->returnOmegaWheel(j));
        }
        delete panWheel_mode_ptr;
        delete liftWheel_mode_ptr;

        t += dt;
    }
    
    // PAN WHEEL

    for (int i=0; i<3; i++) {
        plt::subplot(3,1,i+1);
        plt::named_plot({"leg"+to_string(i+1)},t_vec,panWheel_vec[i]);
        if(i==2) {plt::xlabel("time [sec]");}
        if(i==1) {plt::ylabel("omega_wheel [rad/sec]");}
        if(i==0) {plt::title("PAN_WHEEL Mode");}
        plt::grid(true);
        plt::legend();
    }
    plt::save("../fig/pan_wheel.png",640);
    plt::show();
    

    // LIFT_WHEEL
    for (int i=0; i<3; i++) {
        plt::subplot(3,1,i+1);
        plt::named_plot({"leg"+to_string(i+1)},t_vec,liftWheel_vec[i]);
        if(i==2) {plt::xlabel("time [sec]");}
        if(i==1) {plt::ylabel("omega_wheel [rad/sec]");}
        if(i==0) {plt::title("LIFT_WHEEL Mode");}
        plt::grid(true);
        plt::legend();
    }
    plt::save("../fig/lift_wheel.png",640);
    plt::show();

    // plt::subplot(2,1,1);
    // for (int i=0; i<6; i++)
    //     plt::named_plot({"leg"+to_string(i+1)},t_vec,vel_vec[i]);
    // plt::legend();
    // plt::grid(true);
    // plt::subplot(2,1,2);
    // for (int i=0; i<6; i++)
    //     plt::named_plot({"leg"+to_string(i+1)},t_vec,pos_vec[i]);
    // plt::legend();
    // plt::grid(true);
    // plt::show();

    delete traj_gen_ptr;

    return 0;
}