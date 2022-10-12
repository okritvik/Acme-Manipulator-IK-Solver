#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <vector>

#include "../include/Controller.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Simulator.hpp"

class Robot {
 public:
    bool execute_path();
    bool set_initial_pose(std::vector<double> pose);
    bool set_final_pose(std::vector<double> pose);
    std::vector<double> get_joint_angles();
    void set_joint_angles(std::vector<double> joint_angles);

 private:
    Controller m_control;
    Kinematics m_kinematics;
    Simulator m_sim;

    unsigned int m_dof;
    double m_max_vel;
    double m_min_vel;
    std::vector<double> m_initial_pose;
    std::vector<double> m_final_pose;
    std::vector<double> m_current_angle;
};

#endif  // ROBOT_HPP_
