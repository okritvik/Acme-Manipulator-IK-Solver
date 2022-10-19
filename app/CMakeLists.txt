add_executable(shell-app main.cpp Controller.cpp PositionFK.cpp Robot.cpp Simulator.cpp VelocityIK.cpp)
find_package(Matplot++ REQUIRED)
target_link_libraries(shell-app Matplot++::matplot)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)