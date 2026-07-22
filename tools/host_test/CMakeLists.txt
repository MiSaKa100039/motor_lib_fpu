cmake_minimum_required(VERSION 3.20)

project(motor_host_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(motor_host_test
    test_main.cpp
)

target_compile_definitions(motor_host_test PRIVATE
    MOTOR_LIB_HOST_TEST=1
)

target_include_directories(motor_host_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Public
)
