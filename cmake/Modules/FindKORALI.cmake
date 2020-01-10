include( FindPackageHandleStandardArgs )

execute_process(
  COMMAND python3 -m korali.cxx --cflags
  RESULT_VARIABLE KORALI_EXISTS
  OUTPUT_VARIABLE KORALI_FLAGS_RAW
  OUTPUT_STRIP_TRAILING_WHITESPACE)

execute_process(
  COMMAND python3 -m korali.cxx --libs
  RESULT_VARIABLE KORALI_EXISTS
  OUTPUT_VARIABLE KORALI_LIBS_RAW
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if (KORALI_EXISTS EQUAL "0")
  separate_arguments(KORALI_FLAGS UNIX_COMMAND "${KORALI_FLAGS_RAW}")
  separate_arguments(KORALI_LIBS UNIX_COMMAND "${KORALI_LIBS_RAW}")
endif()

set(KORALI_INCLUDES ${KORALI_FLAGS})
list(FILTER KORALI_FLAGS    EXCLUDE REGEX "-I.*")

list(FILTER KORALI_INCLUDES INCLUDE REGEX "-I.*")
list(TRANSFORM KORALI_INCLUDES REPLACE "-I" "")

message("includes: ${KORALI_INCLUDES}")
message("cflags: ${KORALI_FLAGS}")
message("libs:   ${KORALI_LIBS}")



# find_package_handle_standard_args(
#   KORALI DEFAULT_MSG
#   KORALI_TOKEN
#   KORALI_LIBS
#   )

set(SMARTIES_INCLUDE_DIRS ${SMARTIES_INCLUDE_DIR})
set(SMARTIES_LIBRARIES ${SMARTIES_LIBRARY})

MARK_AS_ADVANCED(
  KORALI_FLAGS_RAW
  KORALI_LIBS_RAW
  )

add_library(korali SHARED IMPORTED)

target_include_directories(korali INTERFACE ${KORALI_INCLUDES})
target_compile_options(korali INTERFACE ${KORALI_FLAGS})
target_link_libraries(korali INTERFACE ${KORALI_LIBS})
