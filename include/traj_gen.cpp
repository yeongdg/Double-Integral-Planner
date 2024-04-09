#include "traj_gen.hpp"

TRAJ_GEN::TRAJ_GEN()
{
    cout<<"Constructor is called.\n";

    yaml_read_ptr = new YAML_READ();

    setup();

    for(int i = 0; i < 6; i++)
        dip_ptr[i] = new DoubleIntegralPlanner(
            traj_constraint[i].a_max, 
            traj_constraint[i].v_max);

}

TRAJ_GEN::TRAJ_GEN(string file_name_)
{
    cout<<"Constructor is called.\n";

    yaml_read_ptr = new YAML_READ(file_name_);

    setup();

    for(int i = 0; i < 6; i++)
        dip_ptr[i] = new DoubleIntegralPlanner(
            traj_constraint[i/3].a_max, 
            traj_constraint[i/3].v_max);
}


void TRAJ_GEN::setup()
{
    yaml_read_ptr->YamlLoadFile();
    for(int i = 0; i < 2; i++)
        traj_constraint[i] = yaml_read_ptr->traj_constraint[i];
    
    for(int i = 0; i < 2; i++)
    {
        cout<<"Name: "<<traj_constraint[i].name<<endl;
        cout<<"a_max: "<<traj_constraint[i].a_max<<endl;
        cout<<"v_max: "<<traj_constraint[i].v_max<<endl;
    }

}

void TRAJ_GEN::setGoalPos(double goal_pos, int idx)
{
    dip_ptr[idx]->setGoalPos(goal_pos);
    dip_ptr[idx]->TimeCalc();
}

void TRAJ_GEN::getTraj(TRJ_DATA &traj, int idx, double time)
{
    dip_ptr[idx]->goalTraj(time);
    traj.p_curr[idx] = dip_ptr[idx]->getPos();
    traj.v_curr[idx] = dip_ptr[idx]->getVel();
    traj.a_curr[idx] = dip_ptr[idx]->getAcc();
}


TRAJ_GEN::~TRAJ_GEN()
{
    for(int i = 0; i < 6; i++)
        delete dip_ptr[i];

    delete yaml_read_ptr;

    cout<<"Destructor is called.\n";
}