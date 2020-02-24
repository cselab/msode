set(MSODE_SOURCES
  factory.cpp
  file_parser.cpp
  simulation.cpp
  velocity_field/interface.cpp
  velocity_field/none.cpp
  velocity_field/constant.cpp
  velocity_field/taylor_green_vortex.cpp
  )

add_library(${LIB_NAME_MSODE} STATIC ${MSODE_SOURCES})
target_compile_features(${LIB_NAME_MSODE} PUBLIC cxx_std_14)
target_include_directories(${LIB_NAME_MSODE} PUBLIC ${MSODE_INCLUDES})

set(cxx_private_flags -O3 -Wall -Wextra -Wshadow -Werror)
target_compile_options(${LIB_NAME_MSODE} PRIVATE ${cxx_private_flags})
target_compile_definitions(${LIB_NAME_MSODE} PUBLIC ${msode_definitions})
