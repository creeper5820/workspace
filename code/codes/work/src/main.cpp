#include <iostream>
#include <string>

#include "include/github/cmdline.hpp"

#include "include/config.inc"
#include "include/handle.hpp"

int main(int argc, char* argv[])
{
    cmdline::parser parser;
    utility::Handle handle(config::path);

    parser.add(
        "list", 'l',
        "List your workspaces");

    parser.add<std::string>(
        "code", 'c',
        "Use code to open your workspace",
        false, "");

    parser.add<std::string>(
        "new", 'n',
        "New a workspace",
        false, "");

    parser.parse_check(argc, argv);

    if (parser.exist("list")) {
        handle.list();
    }

    if (parser.exist("code")) {
        handle.code(parser.get<std::string>("code"));
    }

    if (parser.exist("new")) {
    }
}