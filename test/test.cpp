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
TEST(RobotTest, should_pass1) {
  ASSERT_EQ(acme_kuka.set_initial_pose(&init_pose), false);
}

/**
 * @brief Tests the set_final_pose() method 
 * 
 */
TEST(RobotTest, should_pass2) {
  ASSERT_EQ(acme_kuka.set_final_pose(&fin_pose), false);
}

/**
 * @brief Tests the set_joint_angles() method 
 * 
 */
TEST(RobotTest, should_pass3) {
  ASSERT_EQ(acme_kuka.set_joint_angles(&init_pose), false);
}


// /**
//  * @brief Tests the execute_path() method
//  * 
//  */
// TEST(RobotTest, should_pass4) {
//   ASSERT_EQ(acme_kuka.execute_path(), false);
// }
