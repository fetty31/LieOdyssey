cmake_minimum_required(VERSION 3.14)
project(lie_odyssey_examples)

# C++ Standard Requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Dependencies
find_package( lie_odyssey REQUIRED )
include_directories(${LIE_ODYSSEY_INCLUDE_DIRS})

# Define the Executable
add_executable(se32_gps_iekf 
    se32_gps_iekf.cpp
)

# Link Libraries
target_link_libraries(se32_gps_iekf PRIVATE
    lie_odyssey::lie_odyssey_cpp
)