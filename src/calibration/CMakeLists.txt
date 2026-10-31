cmake_minimum_required(VERSION 3.16)
project(calibration CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
include(CTest)

add_executable(capture src/capture.cpp)
target_include_directories(capture PRIVATE src ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(capture rclcpp sensor_msgs message_filters)
target_link_libraries(capture ${OpenCV_LIBS} yaml-cpp)

add_executable(calibrate src/calibrate.cpp)
target_include_directories(calibrate PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibrate ${OpenCV_LIBS} yaml-cpp)

if(BUILD_TESTING)
  enable_testing()
  # Calibration regressions must use an explicitly selected real capture.
  # Never register the legacy rendered-board or synthetic solver self-tests.
  add_test(NAME stereo_calibration_real
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_stereo_real.py $<TARGET_FILE:calibrate>)
  set_tests_properties(stereo_calibration_real PROPERTIES TIMEOUT 300 SKIP_RETURN_CODE 77)
  add_test(NAME capture_empty_session
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_capture_empty.py $<TARGET_FILE:capture>)
  set_tests_properties(capture_empty_session PROPERTIES TIMEOUT 20)
  add_test(NAME joint_calibration_python
    COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p "test_*.py")
  set_tests_properties(joint_calibration_python PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python")
endif()

install(TARGETS capture calibrate DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_python_install_package(spr_calibration PACKAGE_DIR python/spr_calibration)
install(PROGRAMS scripts/joint_calibrate.py scripts/combine_sessions.py DESTINATION lib/${PROJECT_NAME})

ament_package()
