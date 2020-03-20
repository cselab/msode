set(EIGEN_ROOT "${CMAKE_SOURCE_DIR}/extern/eigen-git-mirror")
find_package(EIGEN REQUIRED)

# those must be private for all targets
set(cxx_warning_flags -Wall -Wextra -Wshadow -Werror)
set(cxx_release_flags -O3 -g)
set(cxx_debug_flags -O0 -g)

add_subdirectory(core)
add_subdirectory(utils)
add_subdirectory(analytic_control)
add_subdirectory(rl)
