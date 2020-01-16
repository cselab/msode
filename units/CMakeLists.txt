set(TEST_FILES
  test_quaternions.cpp
  test_forward.cpp)

foreach(FILE ${TEST_FILES})
    string(REGEX REPLACE "[./]" "_" NAME ${FILE})
    add_executable(${NAME} ${FILE})
    target_link_libraries(${NAME} ${LIB_NAME_MSODE})
    target_link_libraries(${NAME} Catch2::Catch2)
    add_test(NAME ${NAME} COMMAND ${NAME})
endforeach(FILE ${TEST_FILES})

