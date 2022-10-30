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

Simulator::Simulator() {
}

void Simulator::simulate_robot(const std::vector<double> *x_0p,
        const std::vector<double> *y_0p, const std::vector<double> *z_0p,
        const std::vector<nc::NdArray<double>> *tr) {
    // Declare an identity matrix to compute the base to n link transformation
    nc::NdArray<double> tr_0_i = nc::identity<double>(4);
    std::vector<std::vector<double>> origins;

#if !defined (DONT_SHOW_PLOT)
    // Plot the end-effector position
    matplot::scatter3(*x_0p, *y_0p, *z_0p);
    matplot::hold(matplot::on);
#endif

    // Base link position
    origins.push_back({0, 0, 0});

    for (auto tr_i_j : *tr) {
        tr_0_i = tr_0_i.dot(tr_i_j);
        std::vector<double> pos;
        // i-th link position from base
        pos.push_back(tr_0_i.at(0, 3));
        pos.push_back(tr_0_i.at(1, 3));
        pos.push_back(tr_0_i.at(2, 3));
        origins.push_back(pos);
    }

    for (size_t i = 0; i < origins.size()-1; i++) {
        // get the present and next origins of links and plot a line to
        // visualize as a robot link. Note that joint 3 is fixed.
        auto o1 = origins.at(i);
        auto o2 = origins.at(i+1);
        auto ox = {o1.at(0), o2.at(0)};
        auto oy = {o1.at(1), o2.at(1)};
        auto oz = {o1.at(2), o2.at(2)};
#if !defined (DONT_SHOW_PLOT)
        auto l = matplot::plot3(ox, oy, oz);
        l->line_width(6);
#endif
    }
#if !defined (DONT_SHOW_PLOT)
    matplot::xlim({m_xlim.at(0), m_xlim.at(1)});
    matplot::ylim({m_ylim.at(0), m_ylim.at(1)});
    matplot::zlim({m_zlim.at(0), m_zlim.at(1)});
    // Hold is off to clear the links visualization for next iteration
    matplot::hold(matplot::off);
#endif
}

bool Simulator::set_axes(std::vector<double> *xlim, std::vector<double> *ylim,
         std::vector<double> *zlim) {
        // Set the axis limit attributes of the class.
        m_xlim = *xlim;
        m_ylim = *ylim;
        m_zlim = *zlim;

        return true;
}
