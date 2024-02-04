FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    zsh \
    wget curl \
    unzip \
    build-essential \
    gcc g++ gdb make cmake \
    git \
    sudo

# embedded compiler
RUN apt-get install -y \
    clang-format clangd \
    gcc-arm-none-eabi dotnet-runtime-6.0

# Add user
RUN useradd -s /bin/zsh -m ubuntu && echo "ubuntu:ubuntu" | chpasswd && adduser ubuntu sudo && \
    echo "ubuntu ALL=(ALL:ALL)  NOPASSWD:ALL" >> /etc/sudoers

USER ubuntu
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntudotnet-runtime-6.0

# install oh my zsh & change theme to af-magic
RUN wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    sudo chmod +x ./zsh-install.sh && sh ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc &&\
    sudo chsh -s /bin/zsh

COPY .devcontainer/apks/JLink_Linux_V794d_x86_64.deb /home/ubuntu/jlink.deb
COPY .devcontainer/apks/Ozone_Linux_V330d_x86_64.deb /home/ubuntu/ozone.deb
COPY .devcontainer/apks/install.sh /home/ubuntu/install.sh