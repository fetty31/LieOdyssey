include(FetchContent)

FetchContent_Declare(
    manif
    GIT_REPOSITORY https://github.com/artivis/manif.git
    GIT_TAG        65166b3bfd15b1020c29f3eaf6888e1009c79aea
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(manif)

# Ensure a consistent target name
if(NOT TARGET manif)
    add_library(manif INTERFACE)

    target_include_directories(manif INTERFACE
        $<BUILD_INTERFACE:${manif_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${manif_SOURCE_DIR}/external/tl>
    )

    message(STATUS "Created fallback INTERFACE manif target")
endif()

if(NOT TARGET manif::manif)
    add_library(manif::manif ALIAS manif)
endif()