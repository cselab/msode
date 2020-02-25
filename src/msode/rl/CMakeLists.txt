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
  space/interface.cpp
  space/ball.cpp
  space/ball_curriculum_state.cpp
  space/ball_curriculum_action.cpp
  space/box.cpp
  space/factory.cpp
  )

find_package(SMARTIES REQUIRED)

# Get all propreties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

add_library(rl STATIC ${SRC_FILES})
target_link_libraries(rl PUBLIC ${LIB_NAME_MSODE} smarties utils)
target_include_directories(rl PUBLIC ${MSODE_INCLUDES})

target_compile_features(rl PUBLIC cxx_std_14)
set(cxx_private_flags -O3 -Wall -Wextra -Wshadow -Werror)
target_compile_options(rl PRIVATE ${cxx_private_flags})
