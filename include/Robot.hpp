/**
 * @file Robot.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu), Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator, to be used in the assembly of automobiles.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <vector>

#include "../include/Controller.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Simulator.hpp"

/**
 * @brief Defines the Robot class containing 3 children classes - Controller, Kinematics and Simulator.
 * 
 */
class Robot {
 public:
    /**
     * @brief Generates and executes a trajectory for the robot's path from the initial pose to the final pose.
     * 
     * @return true When a valid path exists and the robot executes it.
     * @return false When the path is invalid.
     */
    bool execute_path();

    /**
     * @brief Mutator function to set the initial pose of the robot.
     * 
     * @param pose Initial position of the robot's end-effector w.r.t. the base frame (m)
     * @return true Initial position value is valid and set 
     * @return false Initial position value is invalid 
     */
    bool set_initial_pose(std::vector<double> pose);

    /**
     * @brief Mutator function to set the desired final pose of the robot.
     * 
     * @param pose Final position of the robot's end-effector w.r.t. the base frame (m)
     * @return true Final position value is valid and set 
     * @return false Final position value is invalid
     */
    bool set_final_pose(std::vector<double> pose);

    /**
     * @brief Mutator function to set the robot's joint angles.
     * 
     * @param joint_angles Joint angles of the robot (rad)
     */
    void set_joint_angles(std::vector<double> joint_angles);

 private:
    Controller m_control;
    Kinematics m_kinematics;
    Simulator m_sim;

    unsigned int m_dof;    // Degrees of Freedom of the robot
    double m_max_vel;    // Max permissible joint velocity of the robot
    double m_min_vel;    // Min permissible joint velocity of the robot
    std::vector<double> m_initial_pose;    // Initial pose of the robot
    std::vector<double> m_final_pose;    // Final pose of the robot
    std::vector<double> m_current_angle;    // Current pose of the robot
};

#endif  // ROBOT_HPP_
