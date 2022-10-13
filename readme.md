[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.png)](https://opensource.org/licenses/MIT)
[![Build Status](https://app.travis-ci.com/okritvik/IK-Solver.png?branch=master)](https://app.travis-ci.com/okritvik/IK-Solver)
[![Coverage Status](https://coveralls.io/repos/github/okritvik/IK-Solver/badge.png?branch=master)](https://coveralls.io/github/okritvik/IK-Solver?branch=master)

# Acme Robotics: Manipulator Inverse Kinematics Path Planner

## Overview

<p align="center">
<img width="60%" alt="kuka" src="https://user-images.githubusercontent.com/40534801/195413838-96b439cf-9c95-4f8d-b932-8ad6b8b19630.jpg">
</p>

<p align = "center">
Image for representative purposes only. </br>Source: https://www.kuka.com/en-us/industries/automotive
</p>

In order to augment the robotic solution capabilities of Acme Robotics Inc., a manipulator inverse kinematics based path planner software project is proposed to be deployed on the 7 DOF KUKA WIIA industrial robot in an automobile manufacturing plant's assembly-line. The proposed software solution shall consist of a Jacobian based Inverse Kinematics (IK) solver to generate a smooth trajectory for the end-effector of the robot to be able to transport various automobile components between different stations in the assembly-line. The generated joint angular velocity profiles from the desired end-effector cartesian velocities (obtained from the position trajectory) are then numerically integrated to obtain the joint angles required to achieve such a configuration. Such a velocity-based approach to computing the inverse kinematics of the robot, in lieu of the traditional position-based approach, ensures a smooth joint velocity profile in addition to achieving the desired trajectory, which is of importance in a large scale assembly-unit involving heavy payloads and human workers. The usage of an existing industrial maniuplator like the KUKA provides the company with easy access to scheduled maintenance and repairs of the robotic setup.    

The software design and development process shall involve adhering to an amalgamation of Agile Iterative Processes and pair-programming techniques over a course of two weeks. The efficacy of the implementation shall be verified using the test-driven development approach. The driver and navigator
roles are exchanged during each phase of the project to effectively develop and complete the backlogs.

## Team Members:
* **Adarsh Malapaka** </br>
    UID: 118119625 </br>
    Contact: amalapak@terpmail.umd.edu
* **Kumara Ritvik Oruganti** </br>
    UID: 117368963 </br>
    Contact: okritvik@terpmail.umd.edu

## Proposal

### Quad Chart
<p align="center">
<img width="60%" alt="quad_chart" src="https://user-images.githubusercontent.com/40534801/195413406-c9fde182-6da7-41f0-948f-b241e8c5c846.png">
</p>

### Video
<p align="center">
<a href="https://www.youtube.com/watch?v=WIcdu61a00w&feature=youtu.be" target="_blank">
 <img src="https://user-images.githubusercontent.com/40534801/195415735-dee9ec96-d2e3-4597-8bba-edd1b39871cc.png" alt="Watch the video" width="350" height="200" border="10" />
</a>
</p>

<p align = "center">
Link: https://www.youtube.com/watch?v=WIcdu61a00w
</p>

## File Tree
    ├── app                    
    │   ├── CMakeLists.txt                # Contains NumCpp and matplotplusplus dependencies defined
    │   ├── Controller.cpp                # Stub implementation for Controller class
    │   ├── PositionFK.cpp                # Stub implementation for PositionFK class
    │   ├── Robot.cpp                     # Stub implementation for Robot class
    │   ├── Simulator.cpp                 # Stub implementation for Simulator class
    │   ├── VelocityIK.cpp                # Stub implementation for VelocityIK class
    │   └── main.cpp                      # Main script
    ├── include                    
    │   ├── Controller.hpp                # Main script to run test
    │   ├── Kinematics.hpp                # Initial implemenatation of Google Test cases 
    │   ├── PositionFK.hpp                # Initial implemenatation of Google Test cases 
    │   ├── Robot.hpp                     # Initial implemenatation of Google Test cases 
    │   ├── Simulator.hpp                 # Initial implemenatation of Google Test cases 
    │   └── VelocityIK.hpp                # Initial implemenatation of Google Test cases
    ├── quadchart                   
    │   └── quad_chart_v1.png             # Proposal Quad chart  
    ├── test                    
    │   ├── main.cpp                      # Main script to run test
    │   └── test.cpp                      # Initial implemenatation of Google Test cases 
    └── uml           
        ├── activity_diagram_v1.pdf       # Activity diagram for proposed implementation
        └── class_diagram_v1.pdf          # Class diagram for proposed implementation 
    
