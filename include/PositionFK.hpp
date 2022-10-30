/**
 * @file PositionFK.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator's Forward Kinematics (Position) solver.
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
 * @brief Defines the Position Forward Kinematics solver class using the standard form of the Denavit-Hartenberg convention.
 * 
 */
class PositionFK {
 public:
    /**
     * @brief Constructor for PositionFK to initialize the robot's D-H table matrix.
     * 
     */
    PositionFK();

    /**
     * @brief Accessor function to get the D-H table of the robot.
     * 
     * @return nc::NdArray<double> D-H table matrix of the robot (theta, a , d, alpha).
     */
    nc::NdArray<double> get_dh();

    /**
     * @brief Computes all of the homogeneous transformation matrices between successive links on the robot.
     * 
     * @return std::vector<nc::NdArray<double>> Transformation matrices between links 'i-1' and 'i'
     */
    std::vector<nc::NdArray<double>> link_transformation();

    /**
     * @brief Mutator function to set the robot's joint angles.
     * 
     * @param joint_angles Joint angles of the robot (rad)
     *
     * @return true If joint angles are set correctly, 
     * @return false If the joint angles are incorrectly set.
     */
    bool set_joint_angles(std::vector<double> *joint_angles);

 private:
    std::vector<double> m_joint_angles;    // Joint angles of the robot
    nc::NdArray<double> m_dh_table;    // D-H table matrix
    std::vector<std::vector<double>> m_joint_positions;
};
