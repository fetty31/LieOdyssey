include(FetchContent)
FetchContent_Declare(
    manif
    GIT_REPOSITORY  https://github.com/artivis/manif.git
    GIT_TAG         devel
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
FetchContent_MakeAvailable(manif)

# -------------------------------------------------------------------
# Ensure we always provide a target named manif
# -------------------------------------------------------------------

# Case 1: The upstream defines a usable target
if(TARGET manif)
    add_library(manif::manif ALIAS manif)
    set_target_properties(manif PROPERTIES
        EXPORT_EXCLUDE_FROM_ALL TRUE
    )
    message(STATUS "manif target already provided by upstream")

# Case 2: No target from upstream (header-only library)
else()
    add_library(manif INTERFACE)
    target_include_directories(manif INTERFACE
        $<BUILD_INTERFACE:${manif_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    add_library(manif::manif ALIAS manif)

    # Optionally install/export our own interface target
    install(TARGETS manif
        EXPORT ${PROJECT_NAME}Targets
        INCLUDES DESTINATION include
    )

    message(STATUS "Created INTERFACE target manif with headers only")
endif()