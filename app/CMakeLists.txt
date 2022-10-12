add_executable(shell-app main.cpp)
target_link_libraries(shell-app PUBLIC matplot)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)