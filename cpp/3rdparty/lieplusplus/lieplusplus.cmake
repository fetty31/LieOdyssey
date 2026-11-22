include(FetchContent)

FetchContent_Declare(
    LiePlusPlus
    GIT_REPOSITORY https://github.com/aau-cns/Lie-plusplus.git
    GIT_TAG        ec459c38445d986b5830162ae428b8b242ce5021
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(LiePlusPlus)

# Ensure a consistent target name
if(NOT TARGET LiePlusPlus)
    add_library(LiePlusPlus INTERFACE)

    target_include_directories(LiePlusPlus INTERFACE
        $<BUILD_INTERFACE:${LiePlusPlus_SOURCE_DIR}/include>
    )

    message(STATUS "Created fallback INTERFACE LiePlusPlus target")
endif()

if(NOT TARGET LiePlusPlus::LiePlusPlus)
    add_library(LiePlusPlus::LiePlusPlus ALIAS LiePlusPlus)
endif()
