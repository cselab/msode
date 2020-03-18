set(SRC_FILES
  apply_strategy.cpp
  helpers.cpp
  optimal_path.cpp
  )

set(EIGEN_ROOT "${CMAKE_SOURCE_DIR}/extern/eigen-git-mirror")
find_package(EIGEN REQUIRED)

set(LBFGS_ROOT "${CMAKE_SOURCE_DIR}/extern/LBFGSpp")
add_library(lbfgs INTERFACE IMPORTED)

set_target_properties(lbfgs PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${LBFGS_ROOT}/include"
  )

include(FindPkgConfig)
pkg_check_modules(korali REQUIRED IMPORTED_TARGET korali)

add_library(analytic_control STATIC ${SRC_FILES})
target_link_libraries(analytic_control PUBLIC ${LIB_NAME_MSODE} eigen lbfgs PkgConfig::korali utils)
target_compile_features(analytic_control PUBLIC cxx_std_14)
target_include_directories(analytic_control PUBLIC ${MSODE_INCLUDES})

set(cxx_private_flags -O3 -Wall -Wextra -Wshadow -Werror)
target_compile_options(analytic_control PRIVATE ${cxx_private_flags})
