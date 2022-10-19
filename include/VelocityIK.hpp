/**
 * @file VelocityIK.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator's Inverse Kinematics (Velocity) solver.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <matplot/matplot.h>
#include <vector>
#include <NumCpp.hpp>

/**
 * @brief Defines the Velocity Inverse Kinematics solver class using the Jacobian method (with one joint locked).
 * 
 */
class VelocityIK {
 public:
    /**
     * @brief Construct a new Velocity IK object
     * 
     */
    VelocityIK();

    /**
     * @brief Defines the cartesian velocity trajectory of the robot's end-effector, assuming angular speed to be zero.
     * 
     * @return std::vector<double> Cartesian velocity of the robot's end effector.
     */
    std::vector<double> cartesian_velocity();

    /**
     * @brief Performs numerical integration of the joint speed to obtain joint position for a small time-step
     *        using the update equation, q_next = q_curr + q_dot * dt
     * 
     * @param dt Time-step (sec)
     * @param present_joint_angle Current value of joint angle (rad) 
     * @return std::vector<double> Next value of joint angle (rad)
     */
    std::vector<double> update_joint_angles(double &dt,
                std::vector<double> &present_joint_angle);

    /**
     * @brief Computes the Jacobian matrix of the robot
     * 
     */
    void compute_jacobian();

    /**
     * @brief Accessor function to get the Jacobian matrix of the robot.
     * 
     * @return nc::NdArray<double> Jacobian matrix (6x6)
     */
    nc::NdArray<double> get_jacobian();

 private:
    nc::NdArray<double> m_jacobian;    // Jacobian matrix (6x6) of the robot
};
