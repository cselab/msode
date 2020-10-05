find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  add_executable(ac_separability ac_separability.cpp)
  target_link_libraries(ac_separability OpenMP::OpenMP_CXX msode analytic_control)
endif()

add_executable(ac_stats ac_stats.cpp)
target_link_libraries(ac_stats msode analytic_control)

add_executable(convex_hull convex_hull.cpp)
target_link_libraries(convex_hull utils)

add_executable(dump_velocity_field dump_velocity_field.cpp)
target_link_libraries(dump_velocity_field msode rl)

add_executable(field_around_trajectories field_around_trajectories.cpp)
target_link_libraries(field_around_trajectories msode)

add_executable(forward forward.cpp)
target_link_libraries(forward msode)

add_executable(forward_curve forward_curve.cpp)
target_link_libraries(forward_curve utils)

add_executable(optimal_path_landscape optimal_path_landscape.cpp)
target_link_libraries(optimal_path_landscape analytic_control)

add_executable(run_ac run_ac.cpp)
target_link_libraries(run_ac msode analytic_control)

add_executable(rotating rotating.cpp)
target_link_libraries(rotating msode)

add_executable(step_out_frequencies step_out_frequencies.cpp)
target_link_libraries(step_out_frequencies msode)

add_executable(trajectory_distances trajectory_distances.cpp)
target_link_libraries(trajectory_distances msode rl)

add_executable(trajectory_sdf trajectory_sdf.cpp)
target_link_libraries(trajectory_sdf msode rl)



if (USE_SMARTIES)
  include(FindPkgConfig)
  pkg_check_modules(smarties REQUIRED IMPORTED_TARGET smarties)

  add_executable(run_rl run_rl.cpp rl_helpers.cpp)
  target_link_libraries(run_rl rl PkgConfig::smarties )
  add_sanitizers(run_rl)

  add_executable(run_rl_comp run_rl_comp.cpp rl_helpers.cpp)
  target_link_libraries(run_rl_comp rl analytic_control PkgConfig::smarties )
endif()
