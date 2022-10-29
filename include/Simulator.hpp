/**
 * @file Simulator.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator's Simulator, to simulate the robot in action.
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
 * @brief Defines the Simulator class.
 * 
 */
class Simulator {
 public:
    /**
     * @brief Construct a new Simulator object
     * 
     */
    Simulator();

    /**
     * @brief Simulates the robot in a 3D environment for the given robot configuration.
     * 
     * @param config Robot configuration
     */
    void simulate_robot(const std::vector<double> *x_0p,
        const std::vector<double> *y_0p, const std::vector<double> *z_0p,
        const std::vector<nc::NdArray<double>> *tr);

    /**
     * @brief Mutator function to set the axes limits of the simulation window. 
     * 
     * @param xlim X-axis limit
     * @param ylim Y-axis limit
     * @param zlim Z-axis limit
     */
};
