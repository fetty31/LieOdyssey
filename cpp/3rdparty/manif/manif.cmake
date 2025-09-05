include(FetchContent)
FetchContent_Declare(
    manif
    GIT_REPOSITORY  https://github.com/artivis/manif.git
    GIT_TAG         devel
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
FetchContent_MakeAvailable(manif)

# Create INTERFACE target for header-only usage
if(NOT TARGET manif)
    add_library(manif INTERFACE)
    target_include_directories(manif INTERFACE
        ${manif_SOURCE_DIR}/include
    )
endif()