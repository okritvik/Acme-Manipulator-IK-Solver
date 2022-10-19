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

## Personnel:
* **Adarsh Malapaka** </br>
    UID: 118119625 </br>
    Adarsh Malapaka is currently a Master's student in Robotics at the University of Maryland, College Park, MD (Batch of 2023). His interests include Control Systems, Robot Kinematics, Path Planning, and ADAS. </br>
    Contact: amalapak@terpmail.umd.edu </br>
    LinkedIn: https://www.linkedin.com/in/adarsh-malapaka/
* **Kumara Ritvik Oruganti** </br>
    UID: 117368963 </br>
    Kumara Ritvik Oruganti is currently a Master's student in Robotics at the University of Maryland, College Park, MD (Batch of 2023). His interests include developing sustainable, efficient and intelligent robots and embedded systems for real-world problems. </br>
    Contact: okritvik@terpmail.umd.edu  </br>
    Website: https://www.okritvik.com/

## Agile-Iterative Process (AIP) Resources

### Phase 01 Video

### Product Backlog: 
https://docs.google.com/spreadsheets/d/1HQ5xud5pmoVvb_mYnJKh-MRk1BLdxQRhTKgOH5_86q8/edit?usp=sharing

### Sprint Planning Notes

### Quad Chart
<p align="center">
<img width="60%" alt="quad_chart" src="https://user-images.githubusercontent.com/40534801/195413406-c9fde182-6da7-41f0-948f-b241e8c5c846.png">
</p>

### Proposal
<p align="center">
<a href="https://www.youtube.com/watch?v=WIcdu61a00w&feature=youtu.be" target="_blank">
 <img src="https://user-images.githubusercontent.com/40534801/195415735-dee9ec96-d2e3-4597-8bba-edd1b39871cc.png" alt="Watch the video" width="350" height="200" border="10" />
</a>
</p>

<p align = "center">
Link: https://www.youtube.com/watch?v=WIcdu61a00w
</p>


## Developer Documentation

### Dependencies

### Build
```
git clone --recursive https://github.com/okritvik/Acme-Manipulator-IK-Solver/tree/Phase1.git
cd Acme-Manipulator-IK-Solver
mkdir -p build
cd build
cmake ..
make
```

### Build for Code Coverage
```
cd build
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make 
make code_coverage
```

### Run
1. Once inside the `/build` directory as done above, run: `./app/shell-app`
2. To run Google Test: `./test/cpp-test`


### Tests


### Doxygen Docs
[Doxygen](https://www.doxygen.nl/index.html) is used to generate HTML and LaTEX documentation for the project's API. To install and run Doxygen:

```sudo apt update
sudo apt install doxywizard
sudo apt install doxygen-gui
sudo apt install graphviz

// To run the GUI and generate documentation
dozywizard
```
To display the documentation in a Web Browser (like firefox):
```
firefox ./doc/html/index.html
```

The documentation is saved in the ```/docs``` directory. 


### cppcheck
Change to the root directory of the project, and run:
```
cppcheck --enable=all --std=c++17 ./app/*.cpp ./include/*.hpp ./test/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.

### cpplint
Change to the root directory of the project, and run:
```
cpplint ./app/*.cpp ./include/*.hpp ./test/*.cpp &> ./results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

## Known Issues/Bugs

## File Tree
    ├── app                    
    │   ├── CMakeLists.txt                # Contains NumCpp and matplotplusplus dependencies defined
    │   ├── Controller.cpp                # Stub implementation for Controller class
    │   ├── PositionFK.cpp                # Stub implementation for PositionFK class
    │   ├── Robot.cpp                     # Stub implementation for Robot class
    │   ├── Simulator.cpp                 # Stub implementation for Simulator class
    │   ├── VelocityIK.cpp                # Stub implementation for VelocityIK class
    │   └── main.cpp                      # Main script
    ├── docs
    │   ├── html                          # Folder containing HTML Doxygen documentation
    │   └── latex                         # Folder containing LaTEX Doxygen documentation 
    ├── include                    
    │   ├── Controller.hpp                # Controller class header file 
    │   ├── Kinematics.hpp                # Kinematics class header file 
    │   ├── PositionFK.hpp                # PositionFK class header file 
    │   ├── Robot.hpp                     # Robot class header file  
    │   ├── Simulator.hpp                 # Simulator class header file 
    │   └── VelocityIK.hpp                # VelocityIK class header file
    ├── proposal                   
    │   └── mid_term_proposal.pdf         # Proposal document 
    ├── quadchart                   
    │   └── quad_chart_v1.png             # Proposal Quad chart  
    ├── results                    
    │   ├── cppcheck.txt                  # Main script to run tests
    │   └── cpplint.txt                   # Initial implemenatation of Google Test cases 
    ├── test                    
    │   ├── main.cpp                      # Main script to run tests
    │   └── test.cpp                      # Initial implemenatation of Google Test cases 
    └── UML           
        ├── initial               
        │   ├── activity_diagram_v1.pdf   # Activity diagram for proposed implementation
        │   └── class_diagram_v1          # Class diagram for proposed implementation 
        └── final           
            └── activity_diagram_v2.pdf   # Revised Activity diagram (Phase 1)
    
## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT) and can be found in the ```LICENSE``` file.
