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
     * @param theta Cylindrical co-ordinate system angle for the trajectory. 
     * @return nc::NdArray<double> Cartesian velocity of the robot's end effector.
     */
    nc::NdArray<double> cartesian_velocity(double *theta);

    /**
     * @brief Performs numerical integration of the joint speed to obtain joint position for a small time-step
     *        using the update equation, q_next = q_curr + q_dot * dt
     * 
     * @param dt Time-step (sec)
     * @param present_joint_angle Current value of joint angle (rad)
     * @param joint_angle_dot Joint angular speed from inverse kinematics (rad/s) 
     * @return std::vector<double> Next value of joint angle (rad)
     */
    std::vector<double> update_joint_angles(double *dt,
                std::vector<double> *present_joint_angle,
                std::vector<double> *joint_angle_dot);

    /**
     * @brief Computes the Jacobian matrix of the robot
     * 
     * @param joint_angle Current joint angles of the robot
     * @return true If the Jacobian exists and is computed.
     * @return false If the Jacobian is singular.
     */
    bool compute_jacobian(std::vector<double> *joint_angle);

    /**
     * @brief Accessor function to get the Jacobian matrix of the robot.
     * 
     * @return nc::NdArray<double> Jacobian matrix (6x6)
     */
    nc::NdArray<double> get_jacobian();

 private:
    nc::NdArray<double> m_jacobian;    // Jacobian matrix (6x6) of the robot
};
