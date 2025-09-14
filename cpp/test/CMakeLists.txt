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
target_include_directories(test_liepp_groups PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

add_executable(test_manif_groups test_manif_groups.cpp)
target_link_libraries(test_manif_groups PRIVATE lie_odyssey_cpp gtest_main)
target_include_directories(test_manif_groups PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

add_executable(test_preintegrator test_preintegrator.cpp)
target_link_libraries(test_preintegrator PRIVATE lie_odyssey_cpp gtest_main)
target_include_directories(test_preintegrator PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(test_liepp_groups)
gtest_discover_tests(test_manif_groups)
gtest_discover_tests(test_preintegrator)