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

  list(REMOVE_DUPLICATES KORALI_FLAGS)
  list(REMOVE_DUPLICATES KORALI_LIBS)
endif()

find_package_handle_standard_args(KORALI
  FOUND_VAR
    KORALI_FOUND
  REQUIRED_VARS
    KORALI_FLAGS
    KORALI_LIBS
)

if (KORALI_FOUND)
    set(KORALI_INCLUDES "")
  set(KORALI_DEFINITIONS "")

  foreach(flag IN LISTS KORALI_FLAGS)
    if (flag MATCHES "^-I.*")
      string (REGEX REPLACE "^-I" "" include_path ${flag})
      if (EXISTS ${include_path})
	list (APPEND KORALI_INCLUDES ${include_path})
      endif()
    elseif (flag MATCHES "^-D.*")
      list (APPEND KORALI_DEFINITIONS ${flag})
    else()
      # for now ignore all the rest
      # message("skip ${flag}")
    endif()
  endforeach(flag)

  #list(REMOVE_ITEM KORALI_LIBS "-lkorali")

  MARK_AS_ADVANCED(
    KORALI_FLAGS_RAW
    KORALI_LIBS_RAW
    )

  add_library(korali SHARED IMPORTED)

  target_include_directories(korali INTERFACE ${KORALI_INCLUDES})
  #target_compile_options(korali INTERFACE ${KORALI_FLAGS})
  target_compile_definitions(korali INTERFACE ${KORALI_DEFINITIONS})

  #target_link_libraries(korali INTERFACE ${KORALI_LIBS})
endif()
