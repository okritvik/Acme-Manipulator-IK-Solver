#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include "../include/PositionFK.hpp"
#include "../include/VelocityIK.hpp"

class Kinematics{
 public:
    PositionFK fk_solver;
    VelocityIK ik_solver;
};

#endif  // KINEMATICS_HPP_
