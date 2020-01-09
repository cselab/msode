cmake_minimum_required(VERSION 2.8.12)

list( APPEND CMAKE_MODULE_PATH
  ${CMAKE_SOURCE_DIR}/cmake/Modules
)

set(LIB_NAME_MSODE libmsode)

enable_testing()
add_subdirectory("extern/Catch2/")

add_subdirectory(src)
add_subdirectory(units)

find_package(SMARTIES REQUIRED)
