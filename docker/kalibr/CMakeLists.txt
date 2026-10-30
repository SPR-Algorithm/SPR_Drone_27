cmake_minimum_required(VERSION 3.16)
project(spr_kalibr_covariance LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS sparse_block_matrix)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python38)
find_package(Eigen3 REQUIRED)

add_library(spr_kalibr_covariance MODULE selected_covariance.cpp)
target_compile_options(spr_kalibr_covariance PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(spr_kalibr_covariance PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(spr_kalibr_covariance PRIVATE
  Eigen3::Eigen Python3::Python Boost::python38 ${catkin_LIBRARIES}
  ${CHOLMOD_LIBRARY} ${SUITESPARSE_LIBRARIES})
set_target_properties(spr_kalibr_covariance PROPERTIES
  PREFIX "" LIBRARY_OUTPUT_DIRECTORY /opt/spr/python)
