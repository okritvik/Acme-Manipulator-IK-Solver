/**
 * @file Simulator.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Implementation of Acme's Robot Manipulator's Simulator class.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <vector>

#include "../include/Simulator.hpp"


void Simulator::simulate_robot(std::vector<double> config) {
}

void Simulator::set_axes(double xlim, double ylim, double zlim) {
    m_xlim = xlim;
    m_ylim = ylim;
    m_zlim = zlim;
}
