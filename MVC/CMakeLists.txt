cmake_minimum_required(VERSION 3.08)
project(MVC)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIR} )
set(target_lib ${OpenCV_LIBS} ${GFLAGS_LIBRARY} glog::glog CGAL::CGAL_Core Eigen3::Eigen)
set(target_exec AdaptiveMesh.cpp MVCCloner.cpp)

add_executable(mvc main.cpp ${target_exec})
target_link_libraries(mvc ${target_lib})
