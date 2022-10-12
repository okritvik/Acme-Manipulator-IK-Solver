#include <vector>

#include "../include/Simulator.hpp"


void Simulator::simulate_robot(std::vector<double> config) {
}

void Simulator::set_axes(double xlim, double ylim, double zlim) {
    m_xlim = xlim;
    m_ylim = ylim;
    m_zlim = zlim;
}
