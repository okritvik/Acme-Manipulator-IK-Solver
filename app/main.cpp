#include <iostream>

#include "../include/Robot.hpp"

int main() {
    Robot acme_kuka;
    std::vector<double> start_pose;
    bool test = acme_kuka.set_initial_pose(start_pose);
    std::cout << test << " DONE";
}
