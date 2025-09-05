include(FetchContent)
FetchContent_Declare(
    LiePlusPlus
    GIT_REPOSITORY  https://github.com/aau-cns/Lie-plusplus.git
    GIT_TAG         main
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
FetchContent_MakeAvailable(LiePlusPlus)

# Create INTERFACE target for header-only usage
if(NOT TARGET LiePlusPlus)
    add_library(LiePlusPlus INTERFACE)
    target_include_directories(LiePlusPlus INTERFACE
        ${LiePlusPlus_SOURCE_DIR}/include
    )
endif()