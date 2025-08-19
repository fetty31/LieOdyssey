# Download and set up GoogleTest
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Test executable
add_executable(test_groups test_groups.cpp)
target_link_libraries(test_groups PRIVATE lie_odyssey_cpp gtest_main)
target_include_directories(test_groups PRIVATE ${LIE_ODYSSEY_INCLUDE_DIRS})

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(test_groups)