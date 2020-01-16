cmake_minimum_required(VERSION 3.0.0)

cmake_policy(SET CMP0074 OLD)

list( APPEND CMAKE_MODULE_PATH
  ${CMAKE_SOURCE_DIR}/cmake/Modules
)

project(msode LANGUAGES CXX VERSION 0.1.0)

set(LIB_NAME_MSODE msode)

enable_testing()
add_subdirectory("extern/Catch2/")

add_subdirectory(src)
add_subdirectory(units)
add_subdirectory(apps)


