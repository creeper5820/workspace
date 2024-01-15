FROM ubuntu:22.04 AS basic

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    build-essential \
    vim wget curl unzip \
    cmake make ninja-build gcc g++ gdb\
    sudo

RUN apt-get install -y \
    zsh software-properties-common \
    clang-format git && \
    echo -e "\n" | bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)" && \
    ln -s /usr/bin/clangd-* /usr/bin/clangd

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

# Set user
USER ubuntu
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu

# install oh my zsh & change theme to af-magic
RUN sh -c "$(wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc

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
    --install-extension ms-vscode.hexeditor \
    --install-extension yzhang.markdown-all-in-one \
    --install-extension ms-python.python \
    --install-extension ms-python.vscode-pylance \
    --install-extension xaver.clang-format \
    --install-extension ms-vscode.cpptools 
