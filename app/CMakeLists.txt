find_package(Matplot++ REQUIRED)
if (Matplot++_FOUND)
    add_executable(shell-app main.cpp Controller.cpp PositionFK.cpp Robot.cpp Simulator.cpp VelocityIK.cpp)
    target_link_libraries(shell-app PUBLIC Matplot++::matplot)
endif ()
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)