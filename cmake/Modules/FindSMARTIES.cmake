include( FindPackageHandleStandardArgs )

if ( DEFINED ENV{SMARTIES_ROOT} )
  set( SMARTIES_ROOT "$ENV{SMARTIES_ROOT}" )
endif()

# find_path(
#   SMARTIES_INCLUDE_DIR
#     smarties/smarties.h
#   HINTS
#     ${SMARTIES_ROOT}/include
# )

# find_library(
#   SMARTIES_LIBRARY
#   NAMES SMARTIES
#   HINTS ${SMARTIES_ROOT}/lib
# )

set(SMARTIES_INCLUDE_DIR ${SMARTIES_ROOT}/include)
set(SMARTIES_LIBRARY     ${SMARTIES_ROOT}/lib)

find_package_handle_standard_args(
  SMARTIES DEFAULT_MSG
  SMARTIES_INCLUDE_DIR
  SMARTIES_LIBRARY
)

if (SMARTIES_FOUND)
  set(SMARTIES_INCLUDE_DIRS ${SMARTIES_INCLUDE_DIR})
  set(SMARTIES_LIBRARIES ${SMARTIES_LIBRARY})

  MARK_AS_ADVANCED(
    SMARTIES_LIBRARY
    SMARTIES_INCLUDE_DIR
    SMARTIES_DIR
    )

  add_library(smarties SHARED IMPORTED)

  set_target_properties(smarties PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${SMARTIES_INCLUDE_DIRS}"
    IMPORTED_LOCATION              "${SMARTIES_LIBRARIES}/libsmarties.so"
    )
endif()
