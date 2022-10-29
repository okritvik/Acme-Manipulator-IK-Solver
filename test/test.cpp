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

#include <NumCpp/Functions/array_equal.hpp>
#include <NumCpp/NdArray/NdArrayCore.hpp>
#include <gtest/gtest.h>
#include <vector>
#define PI nc::constants::pi
#include "../include/Robot.hpp"
#include "../include/PositionFK.hpp"
#include "../include/VelocityIK.hpp"
#include "../include/Simulator.hpp"
#include "../include/Controller.hpp"


Robot acme_kuka;
Simulator acme_sim;
VelocityIK acme_vik;
PositionFK acme_pfk;
Controller acme_cont;

/**
 * @brief Tests the set_initial_pose() method 
 * 
 */
TEST(RobotTest, initialPose1) {
  std::vector<double> init_pose = {0.0, 10.5, 62.4};
  ASSERT_EQ(acme_kuka.set_initial_pose(&init_pose), true);
}

TEST(RobotTest, initialPose2) {
  std::vector<double> init_pose = {30.4, 0.0, 0.0};
  ASSERT_EQ(acme_kuka.set_initial_pose(&init_pose), true);
}

/**
 * @brief Tests the set_final_pose() method 
 * 
 */
TEST(RobotTest, finalPose1) {
  std::vector<double> fin_pose = {0.0, 20.5, -32.5};
  ASSERT_EQ(acme_kuka.set_final_pose(&fin_pose), true);
}

TEST(RobotTest, finalPose2) {
  std::vector<double> fin_pose = {20.55, 68.2, 0.0};
  ASSERT_EQ(acme_kuka.set_final_pose(&fin_pose), true);
}

/**
 * @brief Tests the set_joint_angles() method 
 * 
 */
TEST(RobotTest, setAngle1) {
  std::vector<double> jnt_angle =  {PI/2, 0, -PI/2, 0, 0, 0};
  ASSERT_EQ(acme_kuka.set_joint_angles(&jnt_angle), true);
}

TEST(RobotTest, setAngle2) {
  std::vector<double> jnt_angle =  {0, 0, -PI/2, 0, 0, 0};
  ASSERT_EQ(acme_kuka.set_joint_angles(&jnt_angle), true);
}

TEST(RobotTest, getAngle1) {
  std::vector<double> jnt_angle =  {0, 0, -PI/2, 0, 0, 0};
  acme_kuka.set_joint_angles(&jnt_angle);
  std::vector<double> set_angle = acme_kuka.get_joint_angles();

  for (size_t i = 0; i < set_angle.size(); i++) {
    ASSERT_EQ(set_angle[i], jnt_angle[i]);
  }
}

TEST(RobotTest, getAngle2) {
  std::vector<double> jnt_angle =  {PI/2, 0, -PI/2, PI/6, 0, PI/4};
  acme_kuka.set_joint_angles(&jnt_angle);
  std::vector<double> set_angle = acme_kuka.get_joint_angles();

  for (size_t i = 0; i < set_angle.size(); i++) {
    ASSERT_EQ(set_angle[i], jnt_angle[i]);
  }
}

// /**
//  * @brief Tests the execute_path() method
//  *
//  */
TEST(RobotTest, execute1) {
  std::vector<double> init_angle =  {PI/2, 0, -PI/2, 0, 0.00001, 0};
  acme_kuka.set_joint_angles(&init_angle);
  ASSERT_EQ(acme_kuka.execute_path(), true);
}

TEST(RobotTest, execute2) {
  std::vector<double> init_angle =  {5*PI/2, 0, 3*PI/2, 0, 0.0001, 0};
  acme_kuka.set_joint_angles(&init_angle);
  ASSERT_EQ(acme_kuka.execute_path(), true);
}

// PositionFK Test
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

TEST(PositionFKTest, setAngle1) {
  std::vector<double> jnt_angle =  {0, 0, -PI/2, 0, 0, 0};
  ASSERT_EQ(acme_pfk.set_joint_angles(&jnt_angle), true);
}

TEST(PositionFKTest, setAngle2) {
  std::vector<double> jnt_angle =  {PI/2, PI/4, -PI/4, 0, 0, PI};
  ASSERT_EQ(acme_pfk.set_joint_angles(&jnt_angle), true);
}

// VelocityIK Test
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

TEST(VelocityIKTest, computeJacobian1) {
  std::vector<double> jnt_angle =  {PI/2, PI/4, -PI/4, 0, 0, PI};
  ASSERT_EQ(acme_vik.compute_jacobian(&jnt_angle), true);
}

TEST(VelocityIKTest, computeJacobian2) {
  std::vector<double> jnt_angle =  {0, 0, PI/4, 0, 0, PI};
  ASSERT_EQ(acme_vik.compute_jacobian(&jnt_angle), true);
}

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

TEST(VelocityIKTest, cartesianVel1) {
  double th = 0;
  nc::NdArray<double> new_vel = {0, 0, 4*PI, 0, 0, 0};
  ASSERT_EQ(nc::array_equal(new_vel, acme_vik.cartesian_velocity(&th)), true);
}

TEST(SimulatorTest, setAxes1) {
  std::vector<double> xl = {0, 10};
  std::vector<double> yl = {0, 10};
  std::vector<double> zl = {0, 10};
  ASSERT_EQ(acme_sim.set_axes(&xl, &yl, &zl), true);
}
