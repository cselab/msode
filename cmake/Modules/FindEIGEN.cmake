include( FindPackageHandleStandardArgs )

if ( DEFINED ENV{EIGEN_ROOT} )
  set( EIGEN_ROOT "$ENV{EIGEN_ROOT}" )
endif()

find_path(
  EIGEN_INCLUDE_DIR
  NAMES
    Eigen
  PATHS
    /usr/include
    ${EIGEN_ROOT}
)

#set(EIGEN_INCLUDE_DIR ${EIGEN_ROOT})

find_package_handle_standard_args(
  EIGEN DEFAULT_MSG
  EIGEN_INCLUDE_DIR
)

if (EIGEN_FOUND)
  set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})

  MARK_AS_ADVANCED(
    EIGEN_INCLUDE_DIR
    )

  add_library(eigen INTERFACE IMPORTED)

  set_target_properties(smarties PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}"
    )
endif()
