cmake_minimum_required(VERSION 3.16...3.26)
project(lie_odyssey_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Setup build options
option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(ENABLE_TEST "Build LieOdyssey test" OFF)

# ccache setup
if(USE_CCACHE)
  find_program(CCACHE_PATH ccache)
  if(CCACHE_PATH)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    message(STATUS "Using ccache: ${CCACHE_PATH}")
  endif()
endif()

# Set build type 
set(CMAKE_BUILD_TYPE Release)

# Add Eigen3 dependency
include(3rdparty/find_dependencies.cmake)

# Add Lie++ dependency
include(FetchContent)

FetchContent_Declare(
    LiePlusPlus
    GIT_REPOSITORY  https://github.com/aau-cns/Lie-plusplus.git
    GIT_TAG         main
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
FetchContent_MakeAvailable(LiePlusPlus)
list(APPEND external LiePlusPlus) 
list(APPEND include_dirs ${LIEPLUSPLUS_INCLUDE_DIR})
list(APPEND libs LiePlusPlus Eigen3::Eigen)

# To-Do: Not INTERFACE
# add_library(lie_odyssey_cpp INTERFACE)
# target_include_directories(lie_odyssey_cpp INTERFACE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(lie_odyssey_cpp INTERFACE Eigen3::Eigen LiePlusPlus)
# target_compile_features(lie_odyssey_cpp INTERFACE cxx_std_17)

# Executable
add_executable(${PROJECT_NAME}_exec
              src/main.cpp 
              )
# add_dependencies(${PROJECT_NAME}_exec ${catkin_EXPORTED_TARGETS})
# target_include_directories(${PROJECT_NAME}_exec ${LIEPLUSPLUS_INCLUDE_DIR})

# Add Eigen
target_link_libraries(${PROJECT_NAME}_exec PRIVATE Eigen3::Eigen)

# Add Lie++ include directories
target_include_directories(${PROJECT_NAME}_exec PRIVATE
    ${LiePlusPlus_SOURCE_DIR}/include
)

# Link LiePlusPlus if needed
target_link_libraries(${PROJECT_NAME}_exec PRIVATE LiePlusPlus)

# Set the include directory variable
set(LIE_ODYSSEY_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/lie_odyssey)

# (optional) Build tests
if(ENABLE_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Export the INTERFACE target
# install(TARGETS lie_odyssey_cpp
#     EXPORT lie_odyssey_cppTargets
# )

# Export info
# install(EXPORT lie_odyssey_cppTargets
#     FILE lie_odyssey_cppTargets.cmake
#     NAMESPACE lie_odyssey::
#     DESTINATION lib/cmake/lie_odyssey
# )

# Package configuration
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/lie_odysseyConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/lie_odysseyConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/lie_odysseyConfig.cmake"
  INSTALL_DESTINATION lib/cmake/lie_odyssey
  INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}"
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/lie_odysseyConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/lie_odysseyConfigVersion.cmake"
    DESTINATION lib/cmake/lie_odyssey
)

# Optional: only add uninstall if this is the main project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
      IMMEDIATE @ONLY
    )

    add_custom_target(lie_odyssey_uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    )
endif()