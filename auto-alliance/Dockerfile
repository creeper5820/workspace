FROM kenny0407/marslab_fastlio2:latest AS slam-release

# change the shell when build
SHELL ["/bin/bash", "-c"]

RUN sudo apt-get update

RUN sudo apt-get install -y \
    build-essential \
    vim wget curl unzip \
    cmake make ninja-build gcc g++

# fast-lio-localization essential
RUN sudo apt-get install -y \
    ros-$ROS_DISTRO-ros-numpy \
    python-dev \
    python3-pip && \
    pip3 install open3d==0.10

ENTRYPOINT [ "/ros_entrypoint.sh" ]

FROM slam-release AS slam-develop

# develop tools install
RUN sudo apt-get install -y \
    zsh software-properties-common \
    clang-format clangd git

# install oh my zsh & change theme to af-magic
RUN sudo wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    sudo chmod +x ./zsh-install.sh && ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc

# install latest vscode
RUN sudo wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg

RUN sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    rm -f packages.microsoft.gpg

Run sudo apt-get update && \
    sudo apt-get install -y code

# install vscode extensions
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

# source the ros application
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc && \
    echo "source ~/ws_livox/devel/setup.zsh" >> ~/.zshrc