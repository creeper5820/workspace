cmake_minimum_required(VERSION 3.5.0)

project(selector)

include_directories(
    ./src/include/
)

add_executable(
    run
    ./src/main.cpp
)