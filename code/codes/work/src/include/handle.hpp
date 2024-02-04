#include "color.hpp"

#include <dirent.h>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <sys/types.h>
#include <unordered_map>
#include <vector>

namespace utility {

class Handle {
private:
    std::string path_;
    std::vector<std::string> dirent_list_;

private:
    void read_dir(const std::string path)
    {
        // try to open the dirent
        DIR* dirent = opendir(path_.c_str());

        if (NULL == dirent) {
            std::cerr << RED << "error!" << NONE << std::endl;
            return;
        }

        // load all the files in dirent
        struct dirent* dirent_struct;

        while (NULL != (dirent_struct = readdir(dirent))) {

            if (DT_DIR == dirent_struct->d_type
                && std::string(".").compare(dirent_struct->d_name)
                && std::string("..").compare(dirent_struct->d_name)
                && '.' != dirent_struct->d_name[0]) {

                dirent_list_.push_back(dirent_struct->d_name);
            }
        }

        // end
        closedir(dirent);
    }

private:
    enum class NameType {
        MAIN_FOLDER,
        IN_DIR_LIST,
        FOLDER_NUMBER,
        UNKNOWN_NAME,
    };

    std::unordered_map<NameType, std::function<void(const std::string&)>> name_handle_map_ = {
        { NameType::MAIN_FOLDER, [](const std::string& name) -> void {} },
        { NameType::IN_DIR_LIST, [](const std::string& name) -> void {} },
        { NameType::FOLDER_NUMBER, [](const std::string& name) -> void {} },
        { NameType::UNKNOWN_NAME, [](const std::string& name) -> void {} },
    };

    NameType get_name_type(const std::string& name)
    {
    }

    bool is_in_list(const std::string& name) const
    {
        auto find = std::find(
            dirent_list_.begin(),
            dirent_list_.end(),
            name);

        if (find == dirent_list_.end())
            return false;
        else
            return true;
    }

    bool is_main_folder(const std::string& name) const
    {
        return !name.compare(".");
    }

    bool is_availale_number(const std::string& name) const
    {
        return name.size() == 1
            && name[0] >= '0'
            && name[0] <= dirent_list_.size() + '0';
    }

public:
    Handle(std::string path)
        : path_(path)
    {
        read_dir(path_);
    }

    void list()
    {
        for (auto i : dirent_list_)
            std::cout << i.c_str() << NONE << '\n';
    }

    void code(std::string name)
    {
        std::string command = std::string("code ") + path_;

        std::cout << RED << "unknown workspace" << NONE << '\n';

        if (is_main_folder(name)) {
            std::system(command.c_str());
            return;
        }
    }

    void echo_path()
    {
        std::cout << path_ << '\n';
    }
}; // class Handle end
} // namespace function end