cmake_minimum_required(VERSION 3.16)
project(spr_kalibr_covariance LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS sparse_block_matrix)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python38)
find_package(Eigen3 REQUIRED)

add_library(spr_kalibr_covariance MODULE selected_covariance.cpp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(spr_kalibr_covariance PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_include_directories(spr_kalibr_covariance PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(spr_kalibr_covariance PRIVATE
  Eigen3::Eigen Python3::Python Boost::python38 ${catkin_LIBRARIES}
  ${CHOLMOD_LIBRARY} ${SUITESPARSE_LIBRARIES})
set_target_properties(spr_kalibr_covariance PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
  PREFIX "" LIBRARY_OUTPUT_DIRECTORY /opt/spr/python)
