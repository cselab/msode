set(SRC_FILES
  apply_strategy.cpp
  helpers.cpp
  optimal_path.cpp
  )

set(EIGEN_ROOT "${CMAKE_SOURCE_DIR}/extern/eigen-git-mirror")
find_package(EIGEN REQUIRED)

add_library(analytic_control STATIC ${SRC_FILES})
target_link_libraries(analytic_control PUBLIC ${LIB_NAME_MSODE})
target_link_libraries(analytic_control PUBLIC eigen)
target_link_libraries(analytic_control PUBLIC PkgConfig::korali)
target_link_libraries(analytic_control PUBLIC utils)

target_compile_features(analytic_control PUBLIC cxx_std_14)
set(cxx_private_flags -O3 -Wall -Wextra -Wshadow -Werror)
target_compile_options(analytic_control PRIVATE ${cxx_private_flags})
