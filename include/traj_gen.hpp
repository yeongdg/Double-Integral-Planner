#ifndef TRAJ_GEN_HPP
#define TRAJ_GEN_HPP
#include "yaml_read.hpp"
#include "double_integral_planner.hpp"

typedef struct TRJ_DATA_{
    double a_curr[6];
    double v_curr[6];
    double p_curr[6];
} TRJ_DATA;

typedef struct GOAL_DATA_{
    double goal_pos[6];
} GOAL_DATA;

class TRAJ_GEN
{
    public:
        TRAJ_GEN();

        TRAJ_GEN(string file_name_);

        void setup();

        void setGoalPos(double goal_pos, int idx);

        void getTraj(TRJ_DATA &traj, int idx, double time);

        ~TRAJ_GEN();

    private:

        DoubleIntegralPlanner *dip_ptr[6];
        YAML_READ *yaml_read_ptr;

        TRJ_constraint traj_constraint[2];
        TRJ_DATA traj_data;

        string file_name;


};

#endif