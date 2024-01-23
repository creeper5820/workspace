FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    build-essential \
    vim wget curl unzip \
    cmake make ninja-build gcc g++ gdb\
    sudo

RUN apt-get install -y \
    zsh git \
    clang-format clangd \
    dbus-x11

# Add user
RUN useradd -s /bin/zsh -m ubuntu && echo "ubuntu:ubuntu" | chpasswd && adduser ubuntu sudo && \
    echo "ubuntu ALL=(ALL:ALL)  NOPASSWD:ALL" >> /etc/sudoers

# Set user
USER ubuntu
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu

# install oh my zsh & change theme to af-magic
RUN sh -c "$(wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc

# opencv
RUN sudo DEBIAN_FRONTEND=noninteractive apt install -y libopencv-dev