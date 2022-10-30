/**
 * @file test.cpp
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Implementation of Google Test cases framework for Acme's Robot Manipulator.
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <gtest/gtest.h>

#include <vector>
#include <cstddef>

#include <NumCpp/Functions/array_equal.hpp>
#include <NumCpp/NdArray/NdArrayCore.hpp>

#include "../include/Robot.hpp"
#include "../include/PositionFK.hpp"
#include "../include/VelocityIK.hpp"
#include "../include/Simulator.hpp"
#include "../include/Controller.hpp"

#define PI nc::constants::pi

Robot acme_kuka;
Simulator acme_sim;
VelocityIK acme_vik;
PositionFK acme_pfk;
Controller acme_cont;

// Test cases for Robot Methods

/**
 * @brief Tests the set_initial_pose() method with one set of pose values
 * 
 */
TEST(RobotTest, initialPose1) {
  std::vector<double> init_pose = {0.0, 10.5, 62.4};
  ASSERT_EQ(acme_kuka.set_initial_pose(&init_pose), true);
}

/**
 * @brief Tests the set_initial_pose() method with another set of pose values
 * 
 */
TEST(RobotTest, initialPose2) {
  std::vector<double> init_pose = {30.4, 0.0, 0.0};
  ASSERT_EQ(acme_kuka.set_initial_pose(&init_pose), true);
}

/**
 * @brief Tests the set_final_pose() method for one set of pose values
 * 
 */
TEST(RobotTest, finalPose1) {
  std::vector<double> fin_pose = {0.0, 20.5, -32.5};
  ASSERT_EQ(acme_kuka.set_final_pose(&fin_pose), true);
}

/**
 * @brief Tests the set_final_pose() method for another set of pose values
 * 
 */
TEST(RobotTest, finalPose2) {
  std::vector<double> fin_pose = {20.55, 68.2, 0.0};
  ASSERT_EQ(acme_kuka.set_final_pose(&fin_pose), true);
}

/**
 * @brief Tests the set_joint_angles() method for one set of angles
 * 
 */
TEST(RobotTest, setAngle1) {
  std::vector<double> jnt_angle =  {PI/2, 0, -PI/2, 0, 0, 0};
  ASSERT_EQ(acme_kuka.set_joint_angles(&jnt_angle), true);
}

/**
 * @brief Tests the set_joint_angles() method for another set of angles
 * 
 */
TEST(RobotTest, setAngle2) {
  std::vector<double> jnt_angle =  {0, 0, -PI/2, 0, 0, 0};
  ASSERT_EQ(acme_kuka.set_joint_angles(&jnt_angle), true);
}

/**
 * @brief Tests the get_joint_angles() method for one set of angles
 * 
 */
TEST(RobotTest, getAngle1) {
  std::vector<double> jnt_angle =  {0, 0, -PI/2, 0, 0, 0};
  acme_kuka.set_joint_angles(&jnt_angle);
  std::vector<double> set_angle = acme_kuka.get_joint_angles();

  for (size_t i = 0; i < set_angle.size(); i++) {
    ASSERT_EQ(set_angle[i], jnt_angle[i]);
  }
}

/**
 * @brief Tests the get_joint_angles() method for another set of angles
 * 
 */
TEST(RobotTest, getAngle2) {
  std::vector<double> jnt_angle =  {PI/2, 0, -PI/2, PI/6, 0, PI/4};
  acme_kuka.set_joint_angles(&jnt_angle);
  std::vector<double> set_angle = acme_kuka.get_joint_angles();

  for (size_t i = 0; i < set_angle.size(); i++) {
    ASSERT_EQ(set_angle[i], jnt_angle[i]);
  }
}

/**
 * @brief Tests the execute_path() method for one set of initial angles
 *
 */
TEST(RobotTest, execute1) {
  std::vector<double> init_angle =  {PI/2, 0, -PI/2, 0, 0.00001, 0};
  acme_kuka.set_joint_angles(&init_angle);
  ASSERT_EQ(acme_kuka.execute_path(), true);
}

/**
 * @brief Tests the execute_path() method for another set of initial angles
 *
 */
TEST(RobotTest, execute2) {
  std::vector<double> init_angle =  {5*PI/2, 0, 3*PI/2, 0, 0.0001, 0};
  acme_kuka.set_joint_angles(&init_angle);
  ASSERT_EQ(acme_kuka.execute_path(), true);
}

// Test cases for PositionFK Methods

/**
 * @brief Tests the get_dh() method
 *
 */
TEST(PositionFKTest, getDH) {
  nc::NdArray<double> dh_matrix = {
                            {0, 0, -PI/2, 36},
                            {0, 0, PI/2, 0},
                            {0, 0, PI/2, 42},
                            {0, 0, -PI/2, 0},
                            {0, 0, -PI/2, 39.95},
                            {0, 0, PI/2, 0},
                            {0, 0, 0, 20.55}
                            };
  std::vector<double> jnt_angle =  {0, 0, 0, 0, 0, 0};
  acme_pfk.set_joint_angles(&jnt_angle);
  nc::NdArray<double> new_dh = acme_pfk.get_dh();
  ASSERT_EQ(nc::array_equal(new_dh, dh_matrix), true);
}

/**
 * @brief Tests the link_transformation() method
 *
 */
TEST(PositionFKTest, linkTransform) {
  std::vector<double> jnt_angle =  {0, 0, 0, 0, 0, 0};
  acme_pfk.set_joint_angles(&jnt_angle);
  nc::NdArray<double> tr = {{1, 0, 0, 0},
                {0, 0, 1, 0},
                {0, -1, 0, 36},
                {0, 0, 0, 1},
                };
  nc::NdArray<double> new_tr =
                acme_pfk.link_transformation().at(0);
  ASSERT_EQ(nc::array_equal(new_tr, tr), true);
}

/**
 * @brief Tests the set_joint_angles() method for one set of values
 *
 */
TEST(PositionFKTest, setAngle1) {
  std::vector<double> jnt_angle =  {0, 0, -PI/2, 0, 0, 0};
  ASSERT_EQ(acme_pfk.set_joint_angles(&jnt_angle), true);
}

/**
 * @brief Tests the set_joint_angles() method for another set of values
 *
 */
TEST(PositionFKTest, setAngle2) {
  std::vector<double> jnt_angle =  {PI/2, PI/4, -PI/4, 0, 0, PI};
  ASSERT_EQ(acme_pfk.set_joint_angles(&jnt_angle), true);
}

// Test cases for VelocityIK Methods

/**
 * @brief Tests the get_jacobian() method
 *
 */
TEST(VelocityIKTest, getJacobian) {
  nc::NdArray<double> jac = {
                            {0, 102.5, -60.5, 0, 20.55, 0},
                            {0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 1, 0},
                            {1, 0, 1, 1, 0, 1}
                            };
  std::vector<double> jnt_angle =  {0, 0, 0, 0, 0, 0};
  acme_vik.compute_jacobian(&jnt_angle);
  ASSERT_EQ(nc::array_equal(acme_vik.get_jacobian(), jac), true);
}

/**
 * @brief Tests the compute_jacobian() method for one set of angles
 *
 */
TEST(VelocityIKTest, computeJacobian1) {
  std::vector<double> jnt_angle =  {PI/2, PI/4, -PI/4, 0, 0, PI};
  ASSERT_EQ(acme_vik.compute_jacobian(&jnt_angle), true);
}

/**
 * @brief Tests the compute_jacobian() method for another set of angles
 *
 */
TEST(VelocityIKTest, computeJacobian2) {
  std::vector<double> jnt_angle =  {0, 0, PI/4, 0, 0, PI};
  ASSERT_EQ(acme_vik.compute_jacobian(&jnt_angle), true);
}

/**
 * @brief Tests the update_joint_angles() method for one set of values
 *
 */
TEST(VelocityIKTest, updateAngle1) {
  double dt = 0.1;
  std::vector<double> jnt_angle =  {0, 0, PI/4, 0, 0, PI};
  std::vector<double> jnt_angle_dot = {0, 0, 0, 0, 0, 0};
  std::vector<double> nxt_angle =  {0, 0, PI/4, 0, 0, PI};
  std::vector<double> new_angle = acme_vik.update_joint_angles
                    (&dt, &jnt_angle, &jnt_angle_dot);
  for (size_t i = 0; i < nxt_angle.size(); i++) {
    ASSERT_EQ(nxt_angle[i], new_angle[i]);
  }
}

/**
 * @brief Tests the update_joint_angles() method for another set of values
 *
 */
TEST(VelocityIKTest, updateAngle2) {
  double dt = 0.1;
  std::vector<double> jnt_angle =  {PI, 0, PI/4, 0, PI/6, PI};
  std::vector<double> jnt_angle_dot = {10, 10, 10, 10, 10, 10};
  std::vector<double> nxt_angle =  {PI+1, 1, 1+(PI/4), 1, 1+(PI/6), 1+PI};
  std::vector<double> new_angle = acme_vik.update_joint_angles
                    (&dt, &jnt_angle, &jnt_angle_dot);
  for (size_t i = 0; i < nxt_angle.size(); i++) {
    ASSERT_EQ(nxt_angle[i], new_angle[i]);
  }
}

/**
 * @brief Tests the cartesian_velocity() method
 *
 */
TEST(VelocityIKTest, cartesianVel1) {
  double th = 0;
  nc::NdArray<double> new_vel = {0, 0, 4*PI, 0, 0, 0};
  ASSERT_EQ(nc::array_equal(new_vel, acme_vik.cartesian_velocity(&th)), true);
}

// Test cases for Simulator Methods

/**
 * @brief Tests the set_axes() method
 *
 */
TEST(SimulatorTest, setAxes1) {
  std::vector<double> xl = {0, 10};
  std::vector<double> yl = {0, 10};
  std::vector<double> zl = {0, 10};
  ASSERT_EQ(acme_sim.set_axes(&xl, &yl, &zl), true);
}

// Test cases for Controller Methods

/**
 * @brief Tests the set_gains() method
 *
 */
TEST(ControllerTest, setGains) {
  double kp = 1.5;
  double ki = 0.01;
  double kd = 0.001;
  ASSERT_EQ(acme_cont.set_gains(&kp, &ki, &kd), true);
}

/**
 * @brief Tests the get_gains() method
 *
 */
TEST(ControllerTest, getGains) {
  double kp = 1.6;
  double ki = 0.01;
  double kd = 0.001;
  acme_cont.set_gains(&kp, &ki, &kd);
  std::vector<double> gain_vector = {kp, ki, kd};
  auto res_gain = acme_cont.get_gains();
  for (size_t i =0; i < gain_vector.size(); i++) {
    ASSERT_EQ(gain_vector.at(i), res_gain.at(i));
  }
}

/**
 * @brief Tests the control_action() method
 *
 */
TEST(ControllerTest, controlAction) {
  double kp = 1.5;
  double ki = 0.01;
  double kd = 0.001;
  acme_cont.set_gains(&kp, &ki, &kd);
  std::vector<double> pos1 = {0};
  std::vector<double> pos2 = {2};
  ASSERT_EQ(acme_cont.control_action(&pos1, &pos2), 3.0);
}

/**
 * @brief Tests the saturation() method for one set of values
 *
 */
TEST(ControllerTest, saturation1) {
  double val1 = 0;
  double val2 = 5;
  double val = 6;
  ASSERT_EQ(acme_cont.saturation(&val1, &val2, &val), 0.0);
}

/**
 * @brief Tests the saturation() method for another set of values
 *
 */
TEST(ControllerTest, saturation2) {
  double val1 = 0;
  double val2 = 5;
  double val = 2;
  ASSERT_EQ(acme_cont.saturation(&val1, &val2, &val), 2);
}
