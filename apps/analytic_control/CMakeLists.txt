set(SRC_FILES
  apply_strategy.cpp
  helpers.cpp
  optimal_path.cpp
  )

add_library(lib_analytic_control SHARED ${SRC_FILES})
#target_link_libraries(lib_analytic_control PUBLIC EIGEN)
#target_link_libraries(lib_analytic_control PUBLIC )
