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

set(EIGEN_ROOT "${CMAKE_SOURCE_DIR}/extern/eigen-git-mirror")

find_package(SMARTIES REQUIRED)
find_package(EIGEN    REQUIRED)

add_subdirectory(utils)
add_subdirectory(rl)


# app_ac_separability.cpp
# app_ac_stats.cpp
# app_analytic_control.cpp

add_executable(app_forward app_forward.cpp)
target_link_libraries(app_forward msode)
  
add_executable(app_forward_curve app_forward_curve.cpp)
target_link_libraries(app_forward_curve utils)

add_executable(app_rl app_rl.cpp)
target_link_libraries(app_rl rl utils)

#app_rl_comp.cpp

add_executable(app_rl_curriculum app_rl_curriculum.cpp)
target_link_libraries(app_rl_curriculum msode rl utils)

add_executable(app_rotating app_rotating.cpp)
target_link_libraries(app_rotating msode)
