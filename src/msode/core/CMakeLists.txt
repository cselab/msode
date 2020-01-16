set(MSODE_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/../../")

set(MSODE_SOURCES
  factory.cpp
  file_parser.cpp
  simulation.cpp)

add_library(${LIB_NAME_MSODE} STATIC ${MSODE_SOURCES})
target_compile_features(${LIB_NAME_MSODE} PUBLIC cxx_std_14)
target_include_directories(${LIB_NAME_MSODE} PUBLIC ${MSODE_INCLUDES})

set(cxx_private_flags -O3 -Wall -Wextra -Wshadow -Werror)
target_compile_options(${LIB_NAME_MSODE} PRIVATE ${cxx_private_flags})
target_compile_definitions(${LIB_NAME_MSODE} PUBLIC ${msode_definitions})
