#include "Joystick.h"
#include <iostream>

int main(int argc, char **argv) {
   Joystick joystick("/dev/input/js0", true);

   while (1) {
       int len = joystick.read();


       system("clear");
       std::cout << "Axis:" << std::endl;
       std::cout << "left(x, y): " << joystick.map.lx << ", " << joystick.map.ly << std::endl;
       std::cout << "right(x, y): " << joystick.map.rx << ", " << joystick.map.ry << std::endl;
       std::cout << "trigger(left, right): " << joystick.map.lt << ", " << joystick.map.rt << std::endl;
       std::cout << "Button:" << std::endl;
       std::cout << "A,B,X,Y:" << joystick.map.a << ", " << joystick.map.b << ", "
                   << joystick.map.x << ", "<< joystick.map.y << std::endl;


   }

    return 0;
}