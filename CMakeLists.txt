cmake_minimum_required(VERSION 2.8.12)

cmake_policy(SET CMP0074 OLD)

list( APPEND CMAKE_MODULE_PATH
  ${CMAKE_SOURCE_DIR}/cmake/Modules
)

set(LIB_NAME_MSODE msode)

enable_testing()
add_subdirectory("extern/Catch2/")

add_subdirectory(src)
#add_subdirectory(units)
add_subdirectory(apps)


