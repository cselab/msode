find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  add_executable(ac_separability ac_separability.cpp)
  target_link_libraries(ac_separability OpenMP::OpenMP_CXX msode analytic_control)
endif()

add_executable(ac_stats ac_stats.cpp)
target_link_libraries(ac_stats msode analytic_control)

add_executable(dump_velocity_field dump_velocity_field.cpp)
target_link_libraries(dump_velocity_field msode rl)

add_executable(forward forward.cpp)
target_link_libraries(forward msode)
  
add_executable(forward_curve forward_curve.cpp)
target_link_libraries(forward_curve utils)

add_executable(convex_hull convex_hull.cpp)
target_link_libraries(convex_hull utils)

add_executable(run_ac run_ac.cpp)
target_link_libraries(run_ac msode analytic_control)

add_executable(run_rl run_rl.cpp)
target_link_libraries(run_rl rl)

add_executable(run_rl_comp run_rl_comp.cpp)
target_link_libraries(run_rl_comp rl analytic_control)

add_executable(rotating rotating.cpp)
target_link_libraries(rotating msode)
