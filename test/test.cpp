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

#include "../include/Robot.hpp"

Robot acme_kuka;
std::vector<double> init_pose = {1, 2, 3, 4, 5, 6};
std::vector<double> fin_pose = {1, 2, 3, 4, 5, 6};

/**
 * @brief Tests the set_initial_pose() method 
 * 
 */
TEST(Robot, should_pass1) {
  EXPECT_EQ(acme_kuka.set_initial_pose(init_pose), true);
}

/**
 * @brief Tests the set_final_pose() method 
 * 
 */
TEST(Robot, should_pass2) {
  EXPECT_EQ(acme_kuka.set_final_pose(fin_pose), true);
}

/**
 * @brief Tests the set_joint_angles() method 
 * 
 */
TEST(Robot, should_pass3) {
  EXPECT_EQ(acme_kuka.set_joint_angles(init_pose), true);
}

/**
 * @brief Tests the get_joint_angles() method 
 * 
 */
TEST(Robot, should_pass4) {
  EXPECT_EQ(acme_kuka.get_joint_angles(init_pose), true);
}

/**
 * @brief Tests the execute_path() method
 * 
 */
TEST(Robot, should_pass5) {
  EXPECT_EQ(acme_kuka.execute_path(), true);
}
