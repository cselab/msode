include(GoogleTest)
option(INSTALL_GTEST OFF)
option(BUILD_GMOCK OFF)

add_subdirectory("extern/googletest")

function(build_and_create_test src_file libs)
  string(REPLACE ".cpp" "" name ${src_file})
  add_executable(${name} ${src_file})
  target_link_libraries(${name} ${libs})
  add_test(NAME ${name} COMMAND ${name})  
endfunction(build_and_create_test)

build_and_create_test(test_advection.cpp     "gtest;${LIB_NAME_MSODE}")
build_and_create_test(test_config.cpp        "gtest;${LIB_NAME_MSODE}")
build_and_create_test(test_factory.cpp       "gtest;${LIB_NAME_MSODE}")
build_and_create_test(test_file_parser.cpp   "gtest;${LIB_NAME_MSODE}")
build_and_create_test(test_forward.cpp       "gtest;${LIB_NAME_MSODE};utils")
build_and_create_test(test_quaternions.cpp   "gtest;${LIB_NAME_MSODE}")
build_and_create_test(test_velocity_flow.cpp "gtest;${LIB_NAME_MSODE}")

build_and_create_test(test_rl_space.cpp       "gtest;rl")
build_and_create_test(test_rl_environment.cpp "gtest;rl")

build_and_create_test(test_utils_integration.cpp "gtest;utils")
build_and_create_test(test_utils_ODE_solver.cpp  "gtest;utils")
build_and_create_test(test_utils_rnd.cpp         "gtest;utils")
