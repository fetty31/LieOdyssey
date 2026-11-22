if(CMAKE_VERSION VERSION_GREATER 3.24)
  cmake_policy(SET CMP0135 OLD)
endif()

# -------------------------------------------------------------------
# Generic external dependency handler
# -------------------------------------------------------------------
function(find_external_dependency PACKAGE_NAME TARGET_NAME INCLUDED_CMAKE_PATH)

    string(TOUPPER "${PACKAGE_NAME}" PACKAGE_NAME_UP)

    set(USE_FROM_SYSTEM_OPTION "USE_SYSTEM_${PACKAGE_NAME_UP}")
    set(USE_LIBRARY "USE_${PACKAGE_NAME_UP}")

    if(${USE_FROM_SYSTEM_OPTION} AND ${USE_LIBRARY})
        find_package(${PACKAGE_NAME} QUIET NO_MODULE)
    endif()

    if(${USE_LIBRARY})
        if(NOT ${USE_FROM_SYSTEM_OPTION} OR NOT TARGET ${TARGET_NAME})
            set(${USE_FROM_SYSTEM_OPTION} OFF PARENT_SCOPE)
            include("${INCLUDED_CMAKE_PATH}")
        endif()
    endif()

    if(${USE_LIBRARY} AND NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR
            "${PACKAGE_NAME}: expected target '${TARGET_NAME}' was not found."
        )
    endif()

endfunction()


# -------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------

find_external_dependency(
    "Eigen3"
    "Eigen3::Eigen"
    "${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake"
)

find_external_dependency(
    "manif"
    "manif::manif"
    "${CMAKE_CURRENT_LIST_DIR}/manif/manif.cmake"
)

find_external_dependency(
    "LiePlusPlus"
    "LiePlusPlus::LiePlusPlus"
    "${CMAKE_CURRENT_LIST_DIR}/lieplusplus/lieplusplus.cmake"
)