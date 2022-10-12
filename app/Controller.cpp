#include <stdio.h>
#include <vector>

#include "../include/Controller.hpp"

bool Controller::set_gains(double kp, double ki, double kd) {
    return true;
}

std::vector<double> Controller::get_gains() {
    std::vector<double> gain;
    return gain;
}

double Controller::control_action(std::vector<double> present_pose,
            std::vector<double> target_pose) {
                return 0.0;
}

double Controller::saturation(double min_vel, double max_vel) {
    return 0.0;
}
