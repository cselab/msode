set(SRC_FILES
  curriculum.cpp
  optimizers/cmaes.cpp
  mean_vel.cpp
  rnd.cpp
  )

add_library(utils STATIC ${SRC_FILES})
target_link_libraries(utils PUBLIC ${LIB_NAME_MSODE} eigen)
target_compile_features(utils PUBLIC cxx_std_14)
target_include_directories(utils PUBLIC ${MSODE_INCLUDES})
target_compile_options(utils PRIVATE ${cxx_warning_flags})
target_compile_options(utils PRIVATE
  $<$<CONFIG:Debug>:${cxx_debug_flags}>
  $<$<CONFIG:Release>:${cxx_release_flags}>
  )

add_sanitizers(utils)
