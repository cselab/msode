include(GoogleTest)
option(INSTALL_GTEST OFF)
option(BUILD_GMOCK OFF)

add_subdirectory("extern/googletest")

add_executable(test_file_parser test_file_parser.cpp)
target_link_libraries(test_file_parser gtest ${LIB_NAME_MSODE})
add_test(NAME test_file_parser COMMAND test_file_parser)

add_executable(test_forward test_forward.cpp)
target_link_libraries(test_forward gtest ${LIB_NAME_MSODE} utils analytic_control)
add_test(NAME test_forward COMMAND test_forward)

add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration gtest utils)
add_test(NAME test_integration COMMAND test_integration)

add_executable(test_quaternions test_quaternions.cpp)
target_link_libraries(test_quaternions  gtest ${LIB_NAME_MSODE})
add_test(NAME test_quaternions COMMAND test_quaternions)

