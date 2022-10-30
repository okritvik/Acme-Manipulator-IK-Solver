/**
 * @file Controller.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Implementation of Acme's Robot Manipulator's PID controller class.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdio.h>
#include <vector>
#include <NumCpp/Functions/norm.hpp>
#include "../include/Controller.hpp"

Controller::Controller() {
    // Assigning the initial values to the private data members
    m_prev_error = 0;
    m_sum_error = 0;
    m_kd = 0;
    m_ki = 0;
    m_kp = 0;
}

bool Controller::set_gains(double *kp, double *ki, double *kd) {
    // Assigning the given gain values to the class attributes
    m_kd = *kd;
    m_ki = *ki;
    m_kp = *kp;
    return true;
}

std::vector<double> Controller::get_gains() {
    // Push the gains to a vector and return
    std::vector<double> gain;
    gain.push_back(m_kp);
    gain.push_back(m_ki);
    gain.push_back(m_kd);
    return gain;
}

double Controller::control_action(std::vector<double> *present_pose,
            std::vector<double> *target_pose) {
                // Using the given poses, calculate the error and control action
                double curr_pose = present_pose->at(0);
                double tgt_pose = target_pose->at(0);
                double error = tgt_pose - curr_pose;
                double control = m_kp * error + m_kd * m_sum_error +
                        m_ki * m_prev_error;
                return control;
}

double Controller::saturation(double *min_vel, double *max_vel, double *val) {
    // Compare the given value minimum and maximum velocities and return 0 if
    // the condition is not satisfied.
    if ((*min_vel < *val) && (*val< *max_vel)) {
        return *val;
    }
    return 0.0;
}
