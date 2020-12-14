set(MSODE_SOURCES
  config.cpp
  factory.cpp
  file_parser.cpp
  log.cpp
  simulation.cpp
  velocity_field/interface.cpp
  velocity_field/factory.cpp
  velocity_field/none.cpp
  velocity_field/constant.cpp
  velocity_field/sum.cpp
  velocity_field/taylor_green_vortex.cpp
  )

add_library(${LIB_NAME_MSODE} STATIC ${MSODE_SOURCES})
target_compile_features(${LIB_NAME_MSODE} PUBLIC cxx_std_14)
target_include_directories(${LIB_NAME_MSODE} PUBLIC ${MSODE_INCLUDES})

target_compile_options(${LIB_NAME_MSODE} PRIVATE ${cxx_warning_flags})
target_compile_options(${LIB_NAME_MSODE} PRIVATE
  $<$<CONFIG:Debug>:${cxx_debug_flags}>
  $<$<CONFIG:Release>:${cxx_release_flags}>
  )


target_compile_definitions(${LIB_NAME_MSODE} PUBLIC ${msode_definitions})

target_link_libraries(${LIB_NAME_MSODE} PUBLIC nlohmann_json::nlohmann_json)

if (ENABLE_STACKTRACE)
  set(backtrace_dir "${CMAKE_SOURCE_DIR}/extern/backward-cpp")
  target_include_directories(${LIB_NAME_MSODE} PRIVATE ${LIBBFD_INCLUDE_DIRS} ${backtrace_dir})
  target_link_libraries(${LIB_NAME_MSODE} PUBLIC ${LIBBFD_BFD_LIBRARY} -ldl)
endif()

add_sanitizers(${LIB_NAME_MSODE})
