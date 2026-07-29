cmake_minimum_required(VERSION 3.16)
project(mavros_xyz_position_offboard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)

add_library(${PROJECT_NAME}_core
  src/common/artifact_log.cpp
  src/common/cli.cpp
  src/common/types.cpp
  src/bridge/lcp_vision_bridge.cpp
  src/application/application_node.cpp
  src/communication/ground_station_link.cpp
  src/gripper/pwm_gripper.cpp
  src/initialization/initialization.cpp
  src/navigation/navigation.cpp
  src/offboard/offboard.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs mavros_msgs lslidar_msgs)
target_link_libraries(${PROJECT_NAME}_core PkgConfig::JSONCPP)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mavros_xyz_position_node src/main.cpp)
target_link_libraries(mavros_xyz_position_node ${PROJECT_NAME}_core)
ament_target_dependencies(mavros_xyz_position_node rclcpp)
target_compile_options(mavros_xyz_position_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ${PROJECT_NAME}_core mavros_xyz_position_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lcp_navigation test/test_lcp_navigation.cpp SKIP_LINKING_MAIN_LIBRARIES)
  if(TARGET test_lcp_navigation)
    target_link_libraries(test_lcp_navigation ${PROJECT_NAME}_core)
    ament_target_dependencies(test_lcp_navigation rclcpp lslidar_msgs)
    target_compile_definitions(test_lcp_navigation PRIVATE MAVROS_XYZ_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_compile_options(test_lcp_navigation PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_package()
