FROM kenny0407/marslab_fastlio2:latest AS slam-release

# change the shell when build
SHELL ["/bin/bash", "-c"]

RUN sudo apt-get update

RUN sudo apt-get install -y \
    build-essential \
    vim wget curl unzip \
    cmake make ninja-build gcc g++

# remove the sdk-1 and driver-1
RUN rm -rf ~/Livox-SDK ~/ws_livox

# install the sdk-2 and driver-2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /home/mars_ugv/mid-360-sdk && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git /home/mars_ugv/mid-360-driver/src/livox_ros_driver2

# build and install sdk
RUN cd /home/mars_ugv/mid-360-sdk && \
    mkdir build && cd build && \
    cmake ../ && \
    make -j && sudo make install

# configure the mid-360-driver, build and install  it
RUN cd /home/mars_ugv/mid-360-driver/src/livox_ros_driver2 && \
    sed -i "s/"192.168.1.12"/"192.168.1.120"/g" ./config/MID360_config.json && \
    source /opt/ros/noetic/setup.sh && \
    ./build.sh ROS1

# fast-localization build dependencies
RUN sudo apt install -y python3-pip && \
    pip install open3d==0.10.0 && \
    sudo apt install -y ros-$ROS_DISTRO-ros-numpy && \
    yes | pip uninstall numpy && \
    pip install numpy==1.20.3

# all the thread to _thread
# install the fast-lio-localization

RUN sudo apt-get -y install \
    ros-$ROS_DISTRO-foxglove-bridge

COPY ./files/sh /home/mars_ugv/sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]

FROM slam-release AS slam-develop

# develop tools install
RUN sudo apt-get install -y \
    zsh clang-format clangd

# install oh my zsh & change theme to af-magic
RUN sudo wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    sudo chmod +x ./zsh-install.sh && ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    sudo rm ./zsh-install.sh

RUN sudo chsh mars_ugv -s /bin/zsh