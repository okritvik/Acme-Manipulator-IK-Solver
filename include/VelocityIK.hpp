#ifndef VELOCITYIK_HPP_
#define VELOCITYIK_HPP_

#include <vector>
#include <matplot/matplot.h>
#include "NumCpp.hpp"

class VelocityIK{
 public:
    std::vector<double> cartesian_velocity();
    std::vector<double> update_joint_angles(double dt,
                std::vector<double> present_joint_angle);
    void compute_jacobian();
    nc::NdArray<double> get_jacobian();

 private:
    nc::NdArray<double> m_jacobian;
};

#endif  // VELOCITYIK_HPP_
