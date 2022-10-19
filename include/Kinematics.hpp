/**
 * @file Kinematics.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator's Kinematics solver.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "../include/PositionFK.hpp"
#include "../include/VelocityIK.hpp"

/**
 * @brief Defines the Kinematics class containing 2 children classes - PositionFK and VelocityIK.
 * 
 */
class Kinematics {
 public:
    PositionFK fk_solver;
    VelocityIK ik_solver;
};
