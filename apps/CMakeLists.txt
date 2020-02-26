find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  add_executable(app_ac_separability app_ac_separability.cpp)
  target_link_libraries(app_ac_separability OpenMP::OpenMP_CXX msode analytic_control)
endif()

add_executable(app_ac_stats app_ac_stats.cpp)
target_link_libraries(app_ac_stats msode analytic_control)

add_executable(app_analytic_control app_analytic_control.cpp)
target_link_libraries(app_analytic_control msode analytic_control)

add_executable(app_dump_velocity_field app_dump_velocity_field.cpp)
target_link_libraries(app_dump_velocity_field msode rl)

add_executable(app_forward app_forward.cpp)
target_link_libraries(app_forward msode)
  
add_executable(app_forward_curve app_forward_curve.cpp)
target_link_libraries(app_forward_curve utils)

add_executable(app_convex_hull app_convex_hull.cpp)
target_link_libraries(app_convex_hull utils)

add_executable(app_rl app_rl.cpp)
target_link_libraries(app_rl rl)

add_executable(app_rl_comp app_rl_comp.cpp)
target_link_libraries(app_rl_comp rl analytic_control)

add_executable(app_rotating app_rotating.cpp)
target_link_libraries(app_rotating msode)
