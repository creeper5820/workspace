
add_executable(joystick_test Joystick.cpp main.cpp)
target_link_libraries(joystick_test ${CMAKE_THREAD_LIBS_INIT})