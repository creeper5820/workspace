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

RUN apt-get install -y \
    clang-format clangd \
    gcc-arm-none-eabi dotnet-runtime-6.0

# Install latest vscode
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] \
    https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' && \
    rm -f packages.microsoft.gpg && \
    sudo apt-get update && \
    sudo apt-get -y install code

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

# Install vscode extensions
RUN code \
    --extensions-dir  .vscode-server/extensions \
    --install-extension PKief.material-icon-theme \
    --install-extension MS-CEINTL.vscode-language-pack-zh-hans \
    --install-extension cschlosser.doxdocgen \
    --install-extension aaron-bond.better-comments \
    --install-extension llvm-vs-code-extensions.vscode-clangd \
    --install-extension twxs.cmake \
    --install-extension ms-vscode.cmake-tools \
    --install-extension Gruntfuggly.global-config \
    --install-extension xaver.clang-format \
    --install-extension cl.eide