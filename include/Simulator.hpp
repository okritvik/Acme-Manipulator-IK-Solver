#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include <vector>

class Simulator {
 public:
    void simulate_robot(std::vector<double> config);
    void set_axes(double xlim, double ylim, double zlim);

 private:
    double m_xlim;
    double m_ylim;
    double m_zlim;
};

#endif  // SIMULATOR_HPP_
