cmake_minimum_required(VERSION 3.0.0)

cmake_policy(SET CMP0074 OLD)

option(BUILD_WITH_CONTRACTS "build with contract checks" ON)
option(ENABLE_STACKTRACE    "print a stacktrace when failing" ON)

list( APPEND CMAKE_MODULE_PATH
  ${CMAKE_SOURCE_DIR}/cmake/Modules
)

project(msode LANGUAGES CXX VERSION 0.1.0)
enable_testing()

set(LIB_NAME_MSODE msode)

set(msode_definitions "")

if (BUILD_WITH_CONTRACTS)
  set(msode_definitions "$msode_definitions;-DMSODE_FAIL_ON_CONTRACT")
endif()

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(extern/json)

if (ENABLE_STACKTRACE)
  find_package(LIBBFD REQUIRED)
  set(msode_definitions "$msode_definitions;-DMSODE_ENABLE_STACKTRACE")
endif()

add_subdirectory(src)
add_subdirectory(units)
add_subdirectory(apps)


