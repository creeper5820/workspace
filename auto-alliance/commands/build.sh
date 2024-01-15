#!/bin/bash

build()
{
    path=$(dirname $0)
    echo -e "\033[36mplease choose target:\033[0m"
    echo -e "[0] release"
    echo -e "[1] develop"
    echo -e "[path] $path"

    # read your choose
    read target

    if [ $target -eq 0 ]
    then
        docker build -t slam-release --target=slam-release $path/..

    elif [ $target -eq 1 ]
    then
        docker build -t slam-develop --target=slam-develop $path/..
        
    else
        echo -e "\033[31merror input\033[0m"
    fi

    # clear the empty docker images
    docker rmi $(docker images | grep none | awk '{print $3}')
}

build
