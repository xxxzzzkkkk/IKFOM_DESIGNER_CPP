cmake_minimum_required(VERSION 3.16)
project(casadi_jacobian_demo)

find_package(CasADi REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/ikfmd
)

set(SOURCES
    demo/demo.cpp
    ikfmd/BaseDesigner.cpp
)

add_executable(jacobian_demo ${SOURCES})

target_link_libraries(jacobian_demo casadi)
set(CMAKE_CXX_STANDARD 17)
