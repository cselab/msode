cmake_minimum_required(VERSION 2.8.12)

set(LIB_NAME_MSODE libmsode)

enable_testing()
add_subdirectory("extern/Catch2/")

add_subdirectory(src)
add_subdirectory(units)
