cmake_minimum_required(VERSION 3.16...3.26)
project(lie_odyssey_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Project version: ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

# Setup build options
option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(ENABLE_TEST "Build LieOdyssey test" ON)

option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_MANIF "Use system pre-installed Manif" OFF)
option(USE_SYSTEM_LIEPLUSPLUS "Use system pre-installed Lie++" OFF)
option(USE_EIGEN3 "Install Eigen (mandatory)" ON)
option(USE_MANIF "Install Manif lib (optional)" ON)
option(USE_LIEPLUSPLUS "Install Lie++ lib (optional)" ON)

# Check backend
if(NOT USE_MANIF AND NOT USE_LIEPLUSPLUS)
    message(FATAL_ERROR "You must enable at least one Lie backend: USE_MANIF or USE_LIEPLUSPLUS")
endif()

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

# Set the include directory variable
set(LIE_ODYSSEY_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/lie_odyssey)

# Add dependencies
include(3rdparty/find_dependencies.cmake)

# LieOdyssey lib
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/lie_odyssey/version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/include/lie_odyssey/version.hpp"
    @ONLY
)

add_library(${PROJECT_NAME} INTERFACE )
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)


# -------------------------------------------------------------------
# Add Lie++ dependency
# -------------------------------------------------------------------

if(USE_LIEPLUSPLUS)

    message(STATUS "Adding library LiePlusPlus")

    target_compile_definitions(${PROJECT_NAME} INTERFACE
        LIE_BACKEND_LIEPLUSPLUS
    )

    # Build-tree dependency
    target_link_libraries(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:LiePlusPlus::LiePlusPlus>
    )

    # Install-tree include directory
    target_include_directories(${PROJECT_NAME} INTERFACE
        $<INSTALL_INTERFACE:include/LiePlusPlus>
    )

    # Install fetched Lie++ headers
    if(NOT USE_SYSTEM_LIEPLUSPLUS)
        install(
            DIRECTORY ${LiePlusPlus_SOURCE_DIR}/include/
            DESTINATION include/LiePlusPlus
        )
    endif()

endif()


# -------------------------------------------------------------------
# Add Manif dependency
# -------------------------------------------------------------------

if(USE_MANIF)

    message(STATUS "Adding library Manif")

    target_compile_definitions(${PROJECT_NAME} INTERFACE
        LIE_BACKEND_MANIF
    )

    # Build-tree dependency
    target_link_libraries(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:manif::manif>
    )

    # Install-tree include directory
    target_include_directories(${PROJECT_NAME} INTERFACE
        $<INSTALL_INTERFACE:include>
    )

    # Install fetched Manif headers and bundled dependencies
    if(NOT USE_SYSTEM_MANIF)

        install(
            DIRECTORY ${manif_SOURCE_DIR}/include/
            DESTINATION include
        )

        install(
            DIRECTORY ${manif_SOURCE_DIR}/external/tl/tl/
            DESTINATION include/tl
        )

    endif()
endif()

# (optional) Build tests
if(ENABLE_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

# Install headers
install(DIRECTORY include/ DESTINATION include)
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/include/lie_odyssey/version.hpp"
  DESTINATION include/lie_odyssey
)

# Export the INTERFACE target
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
)

# Export info
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE lie_odyssey::
    DESTINATION lib/cmake/lie_odyssey
)

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