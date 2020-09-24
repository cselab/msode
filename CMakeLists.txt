cmake_minimum_required(VERSION 3.2.0)

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
  cmake_policy(SET CMP0074 OLD)
endif()


option(BUILD_WITH_CONTRACTS "build with contract checks" ON)
option(ENABLE_STACKTRACE    "print a stacktrace when failing" ON)
option(USE_KORALI           "compile and link to korali (for testing only)" OFF)

# Choose Release mode as default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
    "Choose the type of build, options are: Debug Release"
    FORCE)
endif()

list( APPEND CMAKE_MODULE_PATH
  ${CMAKE_SOURCE_DIR}/cmake/Modules
  ${CMAKE_SOURCE_DIR}/extern/sanitizers-cmake/cmake
)

find_package(Sanitizers)

project(msode LANGUAGES CXX VERSION 0.1.0)
enable_testing()

set(LIB_NAME_MSODE msode)

set(msode_definitions "")

if (BUILD_WITH_CONTRACTS)
  set(msode_definitions "${msode_definitions};-DMSODE_FAIL_ON_CONTRACT")
endif()

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(extern/json)

if (ENABLE_STACKTRACE)
  find_package(LIBBFD REQUIRED)
  set(msode_definitions "${msode_definitions};-DMSODE_ENABLE_STACKTRACE")
endif()

add_subdirectory(src)
add_subdirectory(units)
add_subdirectory(apps)
