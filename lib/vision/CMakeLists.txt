find_package(Eigen3 REQUIRED)
find_package (OpenMP REQUIRED)

add_library(
        vision
        homography.cpp
        triangulate.cpp
)

target_include_directories(vision PUBLIC
        ${CMAKE_SOURCE_DIR}/
)

target_link_libraries(vision OpenMP::OpenMP_CXX)

set_target_properties(vision PROPERTIES LINKER_LANGUAGE CXX)
target_compile_definitions(vision PRIVATE -DUSE_OPENMP)