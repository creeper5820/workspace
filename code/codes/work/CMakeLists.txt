cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_FLAGS "-std=c++20")

project(selector)

include_directories(
    ./src/include/
)

add_executable(
    run
    ./src/main.cpp
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)