#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import subprocess
import argparse
import os
import re

root = "/home/creeper5820/workspace/codespace/"

def list_workspace():
    folders = [name for name in os.listdir(root) if os.path.isdir(root + name) and not name.startswith('.')]
    folders.sort()
    for i, folder in enumerate(folders, 1):
        print(f"{i}. {folder}")

def open_with_code(name = root):
    folders = [name for name in os.listdir(root) if os.path.isdir(root + name) and not name.startswith('.')]
    folders.sort()

    # 检查 name 是否为根目录
    if name == root:
        subprocess.run(["code", root])
        return
        
    # 检查 name 是否是一个数字
    if name.isdigit():
        index = int(name) - 1  # 从1开始计数，所以需要减1
        if index < 0 or index >= len(folders):
            print(f"错误：序号 {name} 超出范围。")
            return
        name = folders[index]

    # 检查 name 是否存在
    if not os.path.exists(root + name):
        print(f"错误：{name} 不存在。")
        return

    # 检查 name 是否是一个文件夹
    if not os.path.isdir(root + name):
        print(f"错误：{name} 不是一个文件夹。")
        return

    # 如果一切正常，使用 VS Code 打开文件夹
    subprocess.run(["code", root + name])



def new_folder(name):
    # 检查 name 是否已存在
    if os.path.exists(root + name):
        print(f"错误：文件夹 {name} 已存在。")
        return

    # 检查 name 是否符合Ubuntu文件夹名字规范
    # Ubuntu文件夹名字规范：只能包含字母、数字、下划线、破折号、点和空格
    if not re.match(r'^[\w\-. ]+$', name):
        print(f"错误：文件夹名 {name} 不符合Ubuntu文件夹名字规范。")
        return

    os.mkdir(root + name)

def main():
    parser = argparse.ArgumentParser(description='Manage your workspace.')
    parser.add_argument('--list', '-l', action='store_true', help='List all the folders in your workspace.')
    parser.add_argument('--code', '-c', metavar='name', type=str, nargs='?', const=root, help='Use VS Code to open a folder.')
    parser.add_argument('--new', '-n', metavar='name', type=str, help='Create a new folder in workspace.')

    args = parser.parse_args()



    if args.list:
        list_workspace()
    if args.code:
        open_with_code(args.code)
    if args.new:
        new_folder(args.new)
        

if __name__ == "__main__":
    main()
