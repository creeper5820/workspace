cmake_minimum_required(VERSION 3.5.0)

project(opencv-test)
find_package(OpenCV REQUIRED)

include_directories(./src/include)

add_executable(
    run
    ./src/main.cpp
)

add_executable(
    test
    ./src/test.cpp
)

target_link_libraries(run ${OpenCV_LIBS})
target_link_libraries(test ${OpenCV_LIBS})