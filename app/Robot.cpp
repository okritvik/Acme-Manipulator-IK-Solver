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

#include <matplot/core/figure_registry.h>
#include <matplot/freestanding/axes_functions.h>
#include <matplot/freestanding/axes_lim.h>
#include <matplot/freestanding/plot.h>
#include <matplot/util/handle_types.h>
#include <unistd.h>

#include <vector>

#include <NumCpp/Core/Types.hpp>
#include <NumCpp/Functions/identity.hpp>
#include <NumCpp/Functions/linspace.hpp>
#include <NumCpp/Linalg/inv.hpp>
#include <NumCpp/NdArray/NdArrayCore.hpp>

#include "../include/Controller.hpp"
#include "../include/Kinematics.hpp"
#include "../include/Simulator.hpp"
#include "../include/Robot.hpp"

#define PI nc::constants::pi

using namespace nc;


Robot::Robot() {
    m_dof = 6;
    m_max_vel = 100;  // stub
    m_min_vel = 0;  // stub
    m_current_angle =  {PI/2, 0, -PI/2, 0, 0.00001, 0};
}

bool Robot::execute_path() {
    using namespace matplot;

    nc::uint32 N = 40;  // Number of data points
    std::vector<double> curr_pose = {0};
    std::vector<double> tgt_pose = {5};
    double prop_gain = 1;
    double derivative_gain = 0;
    double integral_gain = 0;
    m_control.set_gains(&prop_gain, &integral_gain, &derivative_gain);
    double error = m_control.control_action(&curr_pose,
            &tgt_pose);  // Time interval between successive data points

    double vel_min = 0;
    double vel_max = N;
    double vel = error/N;

    double dt = m_control.saturation(&vel_min, &vel_max, &vel);
    std::vector<nc::NdArray<double>> tr;
    nc::NdArray<double> x_dot;
    std::vector<double> next_j_angle;
    std::vector<double> x_0p, y_0p, z_0p;

    std::vector<double> x_limit = {-50, 50};
    std::vector<double> y_limit = {0, 65};
    std::vector<double> z_limit = {0, 80};

    m_sim.set_axes(&x_limit, &y_limit, &z_limit);

    for (auto theta : nc::linspace(PI/2, 3*PI/2, N)) {
        nc::NdArray<double> tr_0_n = nc::identity<double>(4);
        x_dot = m_kinematics.ik_solver.cartesian_velocity(&theta);
        x_dot = x_dot.transpose();

        m_kinematics.ik_solver.compute_jacobian(&m_current_angle);
        nc::NdArray<double> J = m_kinematics.ik_solver.get_jacobian();
        auto q_dot = nc::linalg::inv(J).dot(x_dot).toStlVector();

        next_j_angle = m_kinematics.ik_solver.update_joint_angles(&dt,
                &m_current_angle, &q_dot);
        Robot::set_joint_angles(&next_j_angle);

        m_kinematics.fk_solver.set_joint_angles(&next_j_angle);
        tr = m_kinematics.fk_solver.link_transformation();

        for (auto tr_i_j : tr) {
            tr_0_n = tr_0_n.dot(tr_i_j);
        }

        x_0p.push_back(tr_0_n.at(0, 3));
        y_0p.push_back(tr_0_n.at(1, 3));
        z_0p.push_back(tr_0_n.at(2, 3));

        m_sim.simulate_robot(&x_0p, &y_0p, &z_0p, &tr);
        // sleep(1);
    }

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
