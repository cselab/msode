add_executable(test_quaternions test_quaternions.cpp)
target_link_libraries(test_quaternions   Catch2::Catch2 ${LIB_NAME_MSODE})
add_test(NAME test_quaternions COMMAND test_quaternions)

add_executable(test_forward test_forward.cpp)
target_link_libraries(test_forward  Catch2::Catch2 ${LIB_NAME_MSODE} utils analytic_control)
add_test(NAME test_forward COMMAND test_forward)

