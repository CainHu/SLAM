cmake_minimum_required(VERSION 3.22)
project(SLAM)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package (OpenMP REQUIRED)

add_subdirectory(lib)
#add_subdirectory(modules)

add_executable(homography homography_test.cpp)
target_link_libraries(homography
        vision
        -lpthread
        OpenMP::OpenMP_CXX)

add_executable(essential essential_test.cpp)
target_link_libraries(essential
        vision
        -lpthread
        OpenMP::OpenMP_CXX)