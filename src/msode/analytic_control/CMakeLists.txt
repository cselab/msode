set(SRC_FILES
  apply_strategy.cpp
  helpers.cpp
  optimal_path.cpp
  )

set(LBFGS_ROOT "${CMAKE_SOURCE_DIR}/extern/LBFGSpp")
add_library(lbfgs INTERFACE IMPORTED)

set_target_properties(lbfgs PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${LBFGS_ROOT}/include"
  )

include(FindPkgConfig)
pkg_check_modules(korali REQUIRED IMPORTED_TARGET korali)

add_library(analytic_control STATIC ${SRC_FILES})
target_link_libraries(analytic_control PUBLIC ${LIB_NAME_MSODE} eigen utils)
target_link_libraries(analytic_control PRIVATE PkgConfig::korali lbfgs)
target_compile_features(analytic_control PUBLIC cxx_std_14)
target_include_directories(analytic_control PUBLIC ${MSODE_INCLUDES})

target_compile_options(analytic_control PRIVATE ${cxx_warning_flags})
target_compile_options(analytic_control PRIVATE
  $<$<CONFIG:Debug>:${cxx_debug_flags}>
  $<$<CONFIG:Release>:${cxx_release_flags}>
  )

add_sanitizers(analytic_control)
