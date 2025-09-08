include(FetchContent)
FetchContent_Declare(
    LiePlusPlus
    GIT_REPOSITORY  https://github.com/aau-cns/Lie-plusplus.git
    GIT_TAG         main
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
FetchContent_MakeAvailable(LiePlusPlus)

# -------------------------------------------------------------------
# Ensure we always provide a target named LiePlusPlus
# -------------------------------------------------------------------

# Case 1: The upstream defines a usable target (check typical names)
if(TARGET LiePlusPlus)
    message(STATUS "LiePlusPlus target already provided by upstream")

# Case 2: No target from upstream (header-only library)
else()
    add_library(LiePlusPlus INTERFACE)
    target_include_directories(LiePlusPlus INTERFACE
        $<BUILD_INTERFACE:${LiePlusPlus_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>   # points to installed location
    )
    message(STATUS "Created INTERFACE target LiePlusPlus with headers only")
endif()