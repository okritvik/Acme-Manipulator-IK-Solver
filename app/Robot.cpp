/**
 * @file Robot.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Implementation of Acme's Robot Manipulator's Robot class.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <NumCpp/Core/Types.hpp>
#include <NumCpp/Functions/identity.hpp>
#include <NumCpp/Functions/linspace.hpp>
#include <NumCpp/Linalg/inv.hpp>
#include <NumCpp/NdArray/NdArrayCore.hpp>
#include <matplot/freestanding/plot.h>
#include <matplot/util/handle_types.h>
#include <matplot/matplot.h>
#include <vector>
// #include <matplot/matplot.h>

#include "../include/Controller.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Simulator.hpp"
#include "../include/Robot.hpp"

#define PI nc::constants::pi

using namespace nc;
using namespace matplot;

Robot::Robot() {
    m_dof = 6;
    m_max_vel = 100;  // stub
    m_min_vel = 0;  // stub
    m_current_angle =  {PI/2, 0, -PI/2, 0, 0.00001, 0};
}

bool Robot::execute_path() {
    // Debug the Transformation Matrix
    // m_kinematics.fk_solver.set_joint_angles(&m_current_angle);
    // std::vector<nc::NdArray<double>> tr;
    // tr = m_kinematics.fk_solver.link_transformation();
    // nc::NdArray<double> tr_0_n = nc::identity<double>(4);
    // for (auto tr_i_j : tr) {
    //     tr_0_n = tr_0_n.dot(tr_i_j);
    //     // tr_i_j.print();
    //     // std::cout << "\n\n";
    // }
    // tr_0_n.print();
    nc::uint32 N = 40;  // Number of data points
    double dt = 5.0/N;  // Time interval between successive data points

    std::vector<nc::NdArray<double>> tr;
    nc::NdArray<double> x_dot;
    std::vector<double> next_j_angle;
    double x_0p, y_0p, z_0p;

    for (auto theta : nc::linspace(PI/2, 5*PI/2, N)) {
        nc::NdArray<double> tr_0_n = nc::identity<double>(4);
        // std::cout << theta << "\n";
        x_dot = m_kinematics.ik_solver.cartesian_velocity(&theta);
        x_dot = x_dot.transpose();
        // for (auto i : x_dot) {
        //     std::cout << i <<" ";
        // }
        // std::cout << "\n";

        m_kinematics.ik_solver.compute_jacobian(&m_current_angle);
        nc::NdArray<double> J = m_kinematics.ik_solver.get_jacobian();
        auto q_dot = nc::linalg::inv(J).dot(x_dot).toStlVector();
        // nc::linalg::inv(J).print();
        // std::cout << "\n";
        // for (auto i : q_dot) {
        //     std::cout << i <<" ";
        // }
        // std::cout << "\n";

        next_j_angle = m_kinematics.ik_solver.update_joint_angles(&dt,
                &m_current_angle, &q_dot);
        Robot::set_joint_angles(&next_j_angle);

        // for (auto ang : next_j_angle) {
        //     std::cout << ang << " ";
        // }
        // std::cout << "\n";

        m_kinematics.fk_solver.set_joint_angles(&next_j_angle);
        tr = m_kinematics.fk_solver.link_transformation();

        for (auto tr_i_j : tr) {
            tr_0_n = tr_0_n.dot(tr_i_j);
        }

        x_0p = tr_0_n.at(0, 3);
        y_0p = tr_0_n.at(1, 3);
        z_0p = tr_0_n.at(2, 3);

        scatter(x_0p, z_0p);
    }
    matplot::show();
    return true;
}

bool Robot::set_initial_pose(std::vector<double> *pose) {
    if (pose != NULL) {
        m_initial_pose = *pose;
        return true;
    } else {
        return false;
    }
}

bool Robot::set_final_pose(std::vector<double> *pose) {
    if (pose != NULL) {
        m_final_pose = *pose;
        return true;
    } else {
        return false;
    }
}

std::vector<double> Robot::get_joint_angles() {
    return m_current_angle;
}

bool Robot::set_joint_angles(std::vector<double> *joint_angles) {
    if (joint_angles != NULL) {
        m_current_angle = *joint_angles;
        return true;
    } else {
        return false;
    }
}
