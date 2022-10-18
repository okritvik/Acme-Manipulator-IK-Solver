/**
 * @file Controller.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator's PID controller.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <stdio.h>
#include <vector>

/**
 * @brief Defines the PID controller class.
 * 
 */
class Controller {
 public:
    /**
     * @brief Mutator function to set the values of controller gains (Kp, Ki, Kd) 
     * 
     * @param kp  Proportional Gain
     * @param ki  Integral Gain
     * @param kd  Derivative Gain
     * @return true Gain values are valid and set 
     * @return false Gain values are invalid 
     */
    bool set_gains(double kp, double ki, double kd);

    /**
     * @brief Accessor function to get the values of controller gains (Kp, Ki, Kd)
     * 
     * @return std::vector<double> Controller gains (Kp, Ki, Kd)
     */
    std::vector<double> get_gains();

    /**
     * @brief Computes the error in pose (between present and target) and applies the PID control action on the error to drive it to zero. 
     * 
     * @param present_pose Current position of the robot's end-effector w.r.t. the base (m)
     * @param target_pose Desired position of the robot's end-effector w.r.t. the base (m)
     * @return double Control action (sec) to change the numerical integration time-step 
     */
    double control_action(std::vector<double> present_pose,
            std::vector<double> target_pose);

    /**
     * @brief Saturates the controller output to within {min_vel, max_vel} for safety reasons.
     * 
     * @param min_vel Minimum possible value of controller output
     * @param max_vel Maximum possible value of controller output
     * @return double Saturated controller output
     */
    double saturation(double min_vel, double max_vel);

 private:
    double m_kp;    // Proportional gain
    double m_kd;    // Integral gain
    double m_ki;    // Derivative gain
    double m_sum_error;    // Accumulated errors
    double m_prev_error;    // Previous error
};

#endif  // CONTROLLER_HPP_
