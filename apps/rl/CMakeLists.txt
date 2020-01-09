set(SRC_FILES
  helpers.cpp
  space.cpp
  )

# Get all propreties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

add_library(rl STATIC ${SRC_FILES})
target_link_libraries(rl PUBLIC ${LIB_NAME_MSODE})
target_link_libraries(rl PUBLIC smarties)
