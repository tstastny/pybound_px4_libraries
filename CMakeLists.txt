cmake_minimum_required(VERSION 3.4...3.18)
project(pybound_px4_libraries
    DESCRIPTION "Python bindings for testing PX4 libraries"
    LANGUAGES CXX)

include_directories(${CMAKE_SOURCE_DIR}
    lib/
    lib/matrix/
)

add_subdirectory(pybind11)

# mathlib
pybind11_add_module(mathlib pybindings/mathlib_bindings.cpp)

# npfg lib
pybind11_add_module(npfg_lib
    pybindings/npfg_bindings.cpp
    lib/npfg/npfg.cpp
    )