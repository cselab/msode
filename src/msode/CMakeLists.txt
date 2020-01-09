set(LIB_NAME_MSODE libmsode)

set(MSODE_SOURCES
  factory.cpp
  file_parser.cpp
  simulation.cpp)

add_library(${LIB_NAME_MSODE} STATIC ${MSODE_SOURCES})
target_compile_features(${LIB_NAME_MSODE} PUBLIC cxx_std_14)
