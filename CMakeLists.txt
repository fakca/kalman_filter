cmake_minimum_required(VERSION 3.5)
project(test_kalman)

# Add required libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

# Add the executable
add_executable(test_kalman test_kalman.cpp)
# Link against required libraries
target_link_libraries(test_kalman PRIVATE Eigen3::Eigen)
