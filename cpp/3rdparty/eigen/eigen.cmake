set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen docs")
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Don't build Eigen tests")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Don't build Eigen pkg-config")
set(EIGEN_BUILD_BLAS OFF CACHE BOOL "Don't build blas module")
set(EIGEN_BUILD_LAPACK OFF CACHE BOOL "Don't build lapack module")

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
add_library(Eigen3::Eigen INTERFACE IMPORTED)
set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
)
