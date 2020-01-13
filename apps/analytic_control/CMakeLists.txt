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
target_link_libraries(analytic_control PUBLIC korali)
