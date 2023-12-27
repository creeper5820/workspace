FROM osrf/ros:melodic-desktop-full AS ros-slam

# change the shell when build
SHELL ["/bin/bash", "-c"]

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# essential libraries install
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    vim wget curl unzip \
    cmake make ninja-build gcc g++\
    libpcl-dev \
    libeigen3-dev 

# develop tools install
RUN apt-get install -y \
    zsh software-properties-common \
    clang-format git && \
    echo -e "\n" | bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)" && \
    ln -s /usr/bin/clangd-* /usr/bin/clangd && \
    rm -rf /var/lib/apt/lists/*

# link the libraies
RUN ln -s /usr/include/eigen*/Eigen /usr/include/Eigen &&\
    ln -s /usr/include/pcl*/pcl /usr/include/pcl

# install oh my zsh & change theme to af-magic
RUN curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh | sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc &&\
    chsh -s /bin/zsh

# create the workspace and pull source code
RUN mkdir ~/auto-alliance && cd ~/auto-alliance && \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && mv Livox-SDK2 livox-sdk && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git livox-driver/src/livox_ros_driver2

# compile the livox-sdk
RUN cd ~/auto-alliance/livox-sdk && \
    mkdir build && cd build && \
    cmake .. && make -j && \
    sudo make install

# compile the livox-driver
RUN cd ~/auto-alliance/livox-driver/src/livox_ros_driver2 && \
    source /opt/ros/melodic/setup.bash && \
    ./build.sh ROS1

# source the ros application
RUN echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc && \
    echo "source ~/auto-alliance/livox-driver/devel/setup.zsh" >> ~/.zshrc