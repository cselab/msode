set(SRC_FILES
  mean_vel.cpp
  rnd.cpp
  )

add_library(utils STATIC ${SRC_FILES})
target_link_libraries(utils PUBLIC ${LIB_NAME_MSODE})
