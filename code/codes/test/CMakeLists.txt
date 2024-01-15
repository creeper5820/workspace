cmake_minimum_required(VERSION 3.5.0)

project(hello-world)

include_directories(
    run
    ./src/include
)

add_executable(
    run
    ./src/main.c
    ./src/include/hello.c
)



