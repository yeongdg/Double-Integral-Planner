#include "traj_gen.hpp"
#include <matplotlibcpp.h>
#include <vector>
#include <map>
#include <filesystem>

using std::vector;

namespace plt = matplotlibcpp;

int main()
{
    string file_name = "../config/test.yaml";
    TRAJ_GEN *traj_gen_ptr;
    traj_gen_ptr = new TRAJ_GEN(file_name);


    vector<double> t_vec;
    vector<double> pos_vec[6];
    vector<double> vel_vec[6];
    vector<double> acc_vec[6];
    double N = 1000;
    double t = 0;
    double dt = 0.01;

    double goal_pos[6] = {10,20,30,40,50,60};

    string legend_name[6];

    std::map<string, string> label_keywords;


    TRJ_DATA traj_data;

    for(int i = 0; i < 6; i++)
        traj_gen_ptr->setGoalPos(goal_pos[i],i);


    for (int i = 0; i < N; i++){
        t += dt;
        t_vec.push_back(t);

        for(int j = 0; j < 6; j++)
        {
            traj_gen_ptr->getTraj(traj_data, j ,t_vec[i]);
            pos_vec[j].push_back(traj_data.p_curr[j]);
        }

    }

    plt::figure_size(1960,1000);
    for(int i = 0; i < 6; i++)
    {
        legend_name[i] = "motor ";

        label_keywords.insert(
            std::pair<string, string>
            ("label",legend_name[i]+std::to_string(i))
            );

        plt::plot(t_vec,pos_vec[i],label_keywords);
        label_keywords.erase("label");
    }



    plt::xlabel("time [sec]");
    plt::ylabel("pos");
    plt::grid(true);
    plt::legend();

    // plt::save("fig1.png",640);
    plt::show();
    

    return 0;
}