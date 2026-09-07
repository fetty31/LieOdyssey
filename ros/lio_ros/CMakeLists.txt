cmake_minimum_required(VERSION 3.8)
project(lio_ros)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_PROFILING "Activate profiling" OFF)

set(CMAKE_BUILD_TYPE "Release")

find_package( PCL 1.12 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

message("PCL version: ${PCL_VERSION}")

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( lie_odyssey REQUIRED )
include_directories(${LIE_ODYSSEY_INCLUDE_DIRS})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${EIGEN3_INLCUDE_DIRS}
  ${PCL_INLCUDE_DIRS}
  ${LIE_ODYSSEY_INCLUDE_DIRS}
)

# LiDAR Odometry Library
add_library(${PROJECT_NAME} 
            src/odometry_core.cpp
            src/state.cpp
            src/ekf.cpp
            )
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
if(USE_PROFILING)
    target_compile_definitions(${PROJECT_NAME} PUBLIC LIO_PROFILE=1)
endif(USE_PROFILING)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenMP_LIBS} lie_odyssey::lie_odyssey_cpp)

# ROS execs
add_executable(${PROJECT_NAME}_node
              src/odometry_node.cpp
              )
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros pcl_conversions)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${PCL_LIBRARIES})

# Binaries
install( TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME} )

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()