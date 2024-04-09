#include "wheel_speed_mode.hpp"

WHEEL_SPEED_MODE::WHEEL_SPEED_MODE() {
    std::cout << "WHEEL_SPEED_MODE::WHEEL_MODE(LIFT_WHEEL/PAN_WHEEL)" << std::endl;
    std::cout << "Enter a Leg Number in the range of 0~2" << std::endl;
}

WHEEL_SPEED_MODE::~WHEEL_SPEED_MODE() {}

WHEEL_SPEED_MODE::WHEEL_SPEED_MODE(WHEEL_MODE wheelMode, TRJ_DATA &traj, LIFT_SENSOR &sensor) {
    switch (wheelMode)
    {
        case PAN_WHEEL:
            for (int legNum=0; legNum<3; legNum++)
                omegaWheel[legNum] = (l_leg*traj.v_curr[legNum + 3*wheelMode]*sin(sensor.qLift_sensor[legNum]))/R_wheel;
            break;

        case LIFT_WHEEL:
            for (int legNum=0; legNum<3; legNum++)
                omegaWheel[legNum] = (l_leg*traj.v_curr[legNum + 3*wheelMode]*cos(traj.p_curr[legNum + 3*wheelMode]))/R_wheel;
            break;

        default:
            wheelMode = LIFT_WHEEL;
            break;
    }
}