cmake_minimum_required(VERSION 3.5.0)

project(selector)

include_directories(
    ./src/include/
)

add_executable(
    run
    ./src/main.cpp
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)