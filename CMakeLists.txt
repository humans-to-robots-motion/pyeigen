cmake_minimum_required(VERSION 3.4...3.18)
project(pyeigen)

add_subdirectory(pybind11)
pybind11_add_module(pyeigen src/main.cpp)

############################
# Dependencies             #
############################

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(pyeigen PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})