cmake_minimum_required(VERSION 3.4...3.18)
project(example LANGUAGES CXX)

include_directories( ${CMAKE_SOURCE_DIR} src include )

add_subdirectory(pybind11)

pybind11_add_module(pybindings src/pybindings.cpp)