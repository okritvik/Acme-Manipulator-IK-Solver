# Finding Matplot++ package in the user lib
find_package(Matplot++ REQUIRED)
if (Matplot++_FOUND)
    # Generate the executables
    add_executable(shell-app main.cpp Controller.cpp PositionFK.cpp Robot.cpp Simulator.cpp VelocityIK.cpp)
    # Link the matplot++ library to the executable
    target_link_libraries(shell-app PUBLIC Matplot++::matplot)
endif ()
include_directories(
    # Include the header files
    ${CMAKE_SOURCE_DIR}/include
)