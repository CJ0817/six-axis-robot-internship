cmake_minimum_required(VERSION 3.31)
project(six_axis_robot LANGUAGES C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_library(robot_contract SHARED src/common/abi_probe.c src/common/abi_v1.c src/kinematics/forward.c src/kinematics/inverse.c)
target_include_directories(robot_contract PUBLIC include)
target_compile_options(robot_contract PRIVATE -Wall -Wextra -Werror -pedantic)
target_link_libraries(robot_contract PRIVATE m)
add_executable(cxx_smoke tests/toolchain/cxx_smoke.cpp)
target_link_libraries(cxx_smoke PRIVATE robot_contract)
enable_testing()
add_test(NAME cxx_abi_smoke COMMAND cxx_smoke)

find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)
add_test(NAME python_abi_smoke
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/examples/c_library_demo.py"
          --library "$<TARGET_FILE:robot_contract>")

add_executable(fk_unit tests/kinematics/test_fk.c)
target_link_libraries(fk_unit PRIVATE robot_contract)
target_compile_options(fk_unit PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME fk_matrix_and_contract COMMAND fk_unit)

add_executable(fk_benchmark tests/kinematics/fk_benchmark.c)
target_link_libraries(fk_benchmark PRIVATE robot_contract)
target_compile_options(fk_benchmark PRIVATE -Wall -Wextra -Werror -pedantic)

add_test(NAME c_analytic_ik
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/verify_c_ik.py"
          --library "$<TARGET_FILE:robot_contract>" --output "${CMAKE_CURRENT_BINARY_DIR}/ik-check")
set_tests_properties(c_analytic_ik PROPERTIES TIMEOUT 120)
