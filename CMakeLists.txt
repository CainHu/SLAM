cmake_minimum_required(VERSION 3.22)
project(SLAM)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package (OpenMP REQUIRED)

add_subdirectory(lib)
#add_subdirectory(modules)

add_executable(SLAM homography_test.cpp)
target_link_libraries(SLAM
        vision
        -lpthread
        OpenMP::OpenMP_CXX)