cmake_minimum_required(VERSION 3.11)
project(manipulator)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(pybind11)
include_directories(include)

pybind11_add_module(handsome src/main.cpp)