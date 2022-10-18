/**
 * @file Simulator.hpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu), Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Class definition for Acme's Robot Manipulator's Simulator, to simulate the robot in action.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include <vector>

/**
 * @brief Defines the Simulator class.
 * 
 */
class Simulator {
 public:
    /**
     * @brief Simulates the robot in a 3D environment for the given robot configuration.
     * 
     * @param config Robot configuration
     */
    void simulate_robot(std::vector<double> config);

    /**
     * @brief Mutator function to set the axes limits of the simulation window. 
     * 
     * @param xlim X-axis limit
     * @param ylim Y-axis limit
     * @param zlim Z-axis limit
     */
    void set_axes(double xlim, double ylim, double zlim);

 private:
    double m_xlim;    // Simulation x-axis limit
    double m_ylim;    // Simulation y-axis limit
    double m_zlim;    // Simulation z-axis limit
};

#endif  // SIMULATOR_HPP_
