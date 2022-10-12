#include <gtest/gtest.h>
#include <vector>
#include "../include/Robot.hpp"
Robot acme_kuka;
std::vector<double> init_pose = {1, 2, 3, 4, 5, 6};
std::vector<double> fin_pose = {1, 2, 3, 4, 5, 6};

TEST(Robot, should_pass1) {
  EXPECT_EQ(acme_kuka.set_initial_pose(init_pose), true);
}

TEST(Robot, should_pass2) {
  EXPECT_EQ(acme_kuka.set_final_pose(fin_pose), true);
}

TEST(Robot, should_pass3) {
  EXPECT_EQ(acme_kuka.set_joint_angles(init_pose), true);
}

TEST(Robot, should_pass4) {
  EXPECT_EQ(acme_kuka.get_joint_angles(init_pose), true);
}

TEST(Robot, should_pass5) {
  EXPECT_EQ(acme_kuka.execute_path(), true);
}
