cmake_minimum_required(VERSION 3.0.2)
project(reeds_shepp)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-std=c++14)
add_library(reeds_shepp reeds_shepp.cpp)
add_library(c_reeds_shepp SHARED wrapper.cpp)
target_link_libraries(c_reeds_shepp reeds_shepp)
