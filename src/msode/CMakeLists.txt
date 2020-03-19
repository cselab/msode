set(EIGEN_ROOT "${CMAKE_SOURCE_DIR}/extern/eigen-git-mirror")
find_package(EIGEN REQUIRED)

add_subdirectory(core)
add_subdirectory(utils)
add_subdirectory(analytic_control)
add_subdirectory(rl)
