include(GoogleTest)
option(INSTALL_GTEST OFF)
option(BUILD_GMOCK OFF)

add_subdirectory("extern/googletest")

add_executable(test_forward test_forward.cpp)
target_link_libraries(test_forward gtest gtest_main ${LIB_NAME_MSODE} utils analytic_control)
add_test(NAME test_forward COMMAND test_forward)

add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration gtest gtest_main utils)
add_test(NAME test_integration COMMAND test_integration)

add_executable(test_quaternions test_quaternions.cpp)
target_link_libraries(test_quaternions  gtest gtest_main ${LIB_NAME_MSODE})
add_test(NAME test_quaternions COMMAND test_quaternions)

