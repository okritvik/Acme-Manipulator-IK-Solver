#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <stdio.h>
#include <vector>

class Controller{
 public:
    bool set_gains(double kp, double ki, double kd);
    std::vector<double> get_gains();
    double control_action(std::vector<double> present_pose,
            std::vector<double> target_pose);
    double saturation(double min_vel, double max_vel);

 private:
    double m_kp;
    double m_kd;
    double m_ki;
    double m_sum_error;
    double m_prev_error;
};

#endif  // CONTROLLER_HPP_
