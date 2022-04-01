cmake_minimum_required(VERSION 3.4...3.18)
project(pybound_mathlib
    DESCRIPTION "Python bindings for testing PX4 math libraries"
    LANGUAGES CXX)

include_directories(${CMAKE_SOURCE_DIR}
    matrix/
    filter/
    px4_platform_common/
    )

add_subdirectory(pybind11)

pybind11_add_module(pybound_mathlib pybindings/pybindings.cpp)