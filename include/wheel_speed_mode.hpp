#ifndef WHEEL_SPEED_MODE_HPP
#define WHEEL_SPEED_MODE_HPP

#include<cmath>
#include "traj_gen.hpp"

typedef struct LIFT_SENSOR_{
    double qLift_sensor[3] = {5,10,15};
} LIFT_SENSOR;

enum WHEEL_MODE {
    PAN_WHEEL,
    LIFT_WHEEL
};

class WHEEL_SPEED_MODE {

    public:
        WHEEL_SPEED_MODE();
        ~WHEEL_SPEED_MODE();
        WHEEL_SPEED_MODE(WHEEL_MODE wheelMode, TRJ_DATA &traj, LIFT_SENSOR &sensor);
        inline double returnOmegaWheel(int idx) {return omegaWheel[idx];};

    private:
        double R_wheel = 16;
        double l_leg = 4;

        double omegaWheel[3];
};

#endif