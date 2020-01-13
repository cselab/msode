set(APP_FILES
  app_ac_separability.cpp
  app_ac_stats.cpp
  app_analytic_control.cpp
  app_forward.cpp
  app_forward_curve.cpp
  app_orient.cpp
  app_rl.cpp
  app_rl_comp.cpp
  app_rl_curriculum.cpp
  app_rotating.cpp)

find_package(KORALI REQUIRED)

add_subdirectory(utils)
add_subdirectory(analytic_control)
add_subdirectory(rl)

# add_executable(app_ac_separability app_ac_separability.cpp)
# target_link_libraries(app_ac_separability msode analytic_control)

# add_executable(app_ac_stats app_ac_stats.cpp)
# target_link_libraries(app_ac_stats msode analytic_control)

# add_executable(app_analytic_control app_analytic_control.cpp)
# target_link_libraries(app_analytic_control msode analytic_control)

add_executable(app_forward app_forward.cpp)
target_link_libraries(app_forward msode)
  
add_executable(app_forward_curve app_forward_curve.cpp)
target_link_libraries(app_forward_curve utils)

add_executable(app_rl app_rl.cpp)
target_link_libraries(app_rl rl utils)

# add_executable(app_rl_comp app_rl_comp.cpp)
# target_link_libraries(app_rl_comp rl utils analytic_control)

add_executable(app_rl_curriculum app_rl_curriculum.cpp)
target_link_libraries(app_rl_curriculum msode rl utils)

add_executable(app_rotating app_rotating.cpp)
target_link_libraries(app_rotating msode)
