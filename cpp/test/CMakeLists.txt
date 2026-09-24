# Download and set up GoogleTest
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Test executables
add_executable(test_liepp_groups test_liepp_groups.cpp)
target_link_libraries(test_liepp_groups PRIVATE lie_odyssey_cpp gtest_main)
# target_include_directories(test_liepp_groups PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

add_executable(test_manif_groups test_manif_groups.cpp)
target_link_libraries(test_manif_groups PRIVATE lie_odyssey_cpp gtest_main)
# target_include_directories(test_manif_groups PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

add_executable(test_preintegrator test_preintegrator.cpp)
target_link_libraries(test_preintegrator PRIVATE lie_odyssey_cpp gtest_main)
# target_include_directories(test_preintegrator PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

add_executable(test_iesekf test_iesekf.cpp)
target_link_libraries(test_iesekf PRIVATE lie_odyssey_cpp gtest_main)
# target_include_directories(test_iesekf PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

add_executable(test_iekf test_iekf.cpp)
target_link_libraries(test_iekf PRIVATE lie_odyssey_cpp gtest_main)
# target_include_directories(test_iekf PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

add_executable(test_gps test_gps.cpp)
target_link_libraries(test_gps PRIVATE lie_odyssey_cpp gtest_main)
target_include_directories(test_gps PRIVATE ${CMAKE_SOURCE_DIR}/../ros/ins_ros/include)

add_executable(test_heading_2d test_heading_2d.cpp)
target_link_libraries(test_heading_2d PRIVATE lie_odyssey_cpp gtest_main)
target_include_directories(test_heading_2d PRIVATE ${CMAKE_SOURCE_DIR}/../ros/ins_ros/include)

add_executable(test_pose_3d test_pose_3d.cpp)
target_link_libraries(test_pose_3d PRIVATE lie_odyssey_cpp gtest_main)
target_include_directories(test_pose_3d PRIVATE ${CMAKE_SOURCE_DIR}/../ros/ins_ros/include)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(test_liepp_groups)
gtest_discover_tests(test_manif_groups)
gtest_discover_tests(test_preintegrator)
gtest_discover_tests(test_iesekf)
gtest_discover_tests(test_iekf)
gtest_discover_tests(test_gps)
gtest_discover_tests(test_heading_2d)
gtest_discover_tests(test_pose_3d)