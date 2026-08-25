cmake_minimum_required(VERSION 3.8)
project(ins_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
message(STATUS "GeographicLib VERSION: ${GeographicLib_VERSION}")

find_package( lie_odyssey REQUIRED )
include_directories(${LIE_ODYSSEY_INCLUDE_DIRS})

# Library (filter dynamics & helpers)
add_library(${PROJECT_NAME}_core SHARED
  src/ekf.cpp
  src/state.cpp
  src/geo/enu_converter.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    lie_odyssey::lie_odyssey_cpp
    ${GeographicLib_LIBRARIES}
)

# Node executable
add_executable(${PROJECT_NAME}_node
  src/estimator_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_core
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Install
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install config/launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
