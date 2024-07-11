FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    wget git curl unzip \
    vim sudo zsh \
    cmake gcc g++

RUN apt-get install -y lsb-release software-properties-common gnupg && \
    echo -e "\n" | bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"

RUN ln -s /usr/bin/clangd-* /usr/bin/clangd && \
    apt-get install -y clang-format-18 && \
    ln -s /usr/bin/clang-format-18 /usr/bin/clang-format 

# Add user
RUN useradd -s /bin/zsh -m creeper && echo "creeper:creeper" | chpasswd && adduser creeper sudo && \
    echo "creeper ALL=(ALL:ALL)  NOPASSWD:ALL" >> /etc/sudoers

# Set user
USER creeper
WORKDIR /home/creeper
ENV USER=creeper
ENV WORKDIR=/home/creeper

# install oh my zsh & change theme to af-magic
RUN sh -c "$(wget https://gitee.com/Devkings/oh_my_zsh_install/raw/master/install.sh -O -)" && \
    sudo sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc &&\
    sudo chsh -s /bin/zsh
