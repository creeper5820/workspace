#include <cstddef>
#include <cstdlib>
#include <dirent.h>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <vector>

#include "color-output.hpp"

int main()
{
    std::cout << GREEN << "Hello World!" << NONE;
    std::cout << GREEN << "Select your workspace" << NONE;

    // try to open the dirent
    DIR* dirent = opendir("/home/creeper5820/workspace/codespace");

    if (NULL == dirent) {
        std::cout << RED << "error!" << NONE;
        return -1;
    }

    // list all the file in dirent
    int order = 0;
    struct dirent* dirent_struct;
    std::vector<std::string> dirent_list;

    while (NULL != (dirent_struct = readdir(dirent))) {
        if (DT_DIR == dirent_struct->d_type
            && std::string(".").compare(dirent_struct->d_name)
            && std::string("..").compare(dirent_struct->d_name)
            && '.' != dirent_struct->d_name[0]) {

            dirent_list.push_back(dirent_struct->d_name);
            std::cout << " " << order << ". " << dirent_struct->d_name << NONE;
            order++;
        }
    }

    closedir(dirent);

    std::cout << GREEN << "> " << NONE;

    int option;

    do {
        std::cin >> option;
    } while ('q' != option && (option < 0 || option > order));

    std::string command = std::string("code ") + "/home/creeper5820/workspace/codespace/" + dirent_list[option];
    std::system(command.c_str());

    return 0;
}