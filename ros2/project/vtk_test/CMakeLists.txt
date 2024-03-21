cmake_minimum_required(VERSION 3.8)

project(vtk_test)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(VTK REQUIRED)

add_executable(run src/main.cpp)

link_libraries(${VTK_LIBRARIES})