include( FindPackageHandleStandardArgs )

# Checks an environment variable; note that the first check
# does not require the usual CMake $-sign.
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
else ()
  set(SMARTIES_DIR "" CACHE STRING
    "An optional hint to a directory for finding `smarties`"
  )
endif()
