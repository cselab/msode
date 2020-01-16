set(SRC_FILES
  mean_vel.cpp
  rnd.cpp
  )

add_library(utils STATIC ${SRC_FILES})
target_link_libraries(utils PUBLIC ${LIB_NAME_MSODE})
target_compile_features(utils PUBLIC cxx_std_14)
target_include_directories(utils PUBLIC ${MSODE_INCLUDES})

set(cxx_private_flags -O3 -Wall -Wextra -Wshadow -Werror)
target_compile_options(utils PRIVATE ${cxx_private_flags})
