#ifndef POSITIONFK_HPP_
#define POSITIONFK_HPP_

#include <vector>

#include "NumCpp.hpp"

class PositionFK{
 public:
    PositionFK();
    nc::NdArray<double> get_dh();
    nc::NdArray<double> link_transformation();
    void set_joint_angles(std::vector<double> joint_angles);

 private:
    std::vector<double> m_joint_angles;
    nc::NdArray<double> m_dh_table;
    std::vector<std::vector<double>> m_joint_positions;
};

#endif  // POSITIONFK_HPP_
