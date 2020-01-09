set(APP_FILES
  app_ac_separability.cpp
  app_ac_stats.cpp
  app_analytic_control.cpp
  app_forward.cpp
  app_forward_curve.cpp
  app_orient.cpp
  app_rl_comp.cpp
  app_rl.cpp
  app_rl_curriculum.cpp
  app_rotating.cpp)

find_package(SMARTIES REQUIRED)

add_subdirectory(utils)
add_subdirectory(rl)

add_executable(app_rl app_rl.cpp)
target_link_libraries(app_rl rl utils)
