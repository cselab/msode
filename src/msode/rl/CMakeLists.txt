set(SRC_FILES
  environment.cpp
  factory.cpp
  field_from_action/interface.cpp
  field_from_action/weighted_targets.cpp
  field_from_action/action_change.cpp
  field_from_action/direct.cpp
  field_from_action/local_frame.cpp
  field_from_action/factory.cpp
  helpers.cpp
  pos_ic/interface.cpp
  pos_ic/ball.cpp
  pos_ic/ball_growing.cpp
  pos_ic/ball_random_walk.cpp
  pos_ic/ball_random_walk_drift.cpp
  pos_ic/box.cpp
  pos_ic/factory.cpp
  target_distances/factory.cpp
  target_distances/none.cpp
  target_distances/euclidean.cpp
  target_distances/square.cpp
  target_distances/travel_time.cpp
  target_distances/travel_time_non_optimal.cpp
  )

include(FindPkgConfig)
pkg_check_modules(smarties REQUIRED IMPORTED_TARGET smarties)

# Get all propreties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

add_library(rl STATIC ${SRC_FILES})
target_link_libraries(rl PUBLIC ${LIB_NAME_MSODE} PkgConfig::smarties utils analytic_control)
target_include_directories(rl PUBLIC ${MSODE_INCLUDES})

target_compile_features(rl PUBLIC cxx_std_14)
target_compile_options(rl PRIVATE ${cxx_warning_flags})
target_compile_options(rl PRIVATE
  $<$<CONFIG:Debug>:${cxx_debug_flags}>
  $<$<CONFIG:Release>:${cxx_release_flags}>
  )

add_sanitizers(rl)
