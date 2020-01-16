set(SRC_FILES
  environment.cpp
  helpers.cpp
  magnetic_field_from_action.cpp
  space.cpp
  )

find_package(SMARTIES REQUIRED)

# Get all propreties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

add_library(rl STATIC ${SRC_FILES})
target_link_libraries(rl PUBLIC ${LIB_NAME_MSODE} smarties)
target_include_directories(rl PUBLIC ${MSODE_INCLUDES})

target_compile_features(rl PUBLIC cxx_std_14)
set(cxx_private_flags -O3 -Wall -Wextra -Wshadow -Werror)
target_compile_options(rl PRIVATE ${cxx_private_flags})
