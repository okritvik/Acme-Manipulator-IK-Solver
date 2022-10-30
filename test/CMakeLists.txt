set(GTEST_SHUFFLE 1)
find_package(Matplot++ REQUIRED)
if (Matplot++_FOUND)
    add_executable(
        acme-test
        main.cpp
        test.cpp
        ../app/Controller.cpp
        ../app/PositionFK.cpp
        ../app/Robot.cpp
        ../app/Simulator.cpp
        ../app/VelocityIK.cpp
    )

    target_include_directories(acme-test PUBLIC ../vendor/googletest/googletest/include 
                                            ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(acme-test PUBLIC gtest)
    target_link_libraries(acme-test PUBLIC Matplot++::matplot)
endif ()
