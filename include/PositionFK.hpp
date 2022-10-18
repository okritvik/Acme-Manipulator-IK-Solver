/**
 * @file PositionFK.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu), Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator's Forward Kinematics (Position) solver.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef POSITIONFK_HPP_
#define POSITIONFK_HPP_

#include <vector>

#include "NumCpp.hpp"

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
     * @brief Computes the homogeneous transformation matrix between successive links on the robot.
     * 
     * @return nc::NdArray<double> Transformation matrix between links 'i-1' and 'i'
     */
    nc::NdArray<double> link_transformation();

    /**
     * @brief Mutator function to set the robot's joint angles.
     * 
     * @param joint_angles Joint angles of the robot (rad)
     */
    void set_joint_angles(std::vector<double> joint_angles);

 private:
    std::vector<double> m_joint_angles;    // Joint angles of the robot
    nc::NdArray<double> m_dh_table;    // D-H table matrix
    std::vector<std::vector<double>> m_joint_positions;
};

#endif  // POSITIONFK_HPP_
