#include <vector>

#include "../include/PositionFK.hpp"
#include "NumCpp.hpp"

PositionFK::PositionFK() {
    m_dh_table = 0;
}

nc::NdArray<double> PositionFK::get_dh() {
    nc::NdArray<double> dh;
    return dh;
}

nc::NdArray<double> PositionFK::link_transformation() {
    nc::NdArray<double> tr;
    return tr;
}

void PositionFK::set_joint_angles(std::vector<double> joint_angles) {
}
