cmake_minimum_required(VERSION 3.08)
project(Mesh_Simplify)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -mavx2 -msse4")

find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${GFLAGS_INCLUDE_DIR})
set(target_lib ${GFLAGS_LIBRARY} glog::glog Eigen3::Eigen)
set(target_exec main.cpp)

add_executable(main ${target_exec})
target_link_libraries(main ${target_lib})
