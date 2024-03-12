FROM osrf/ros:humble-desktop-full AS ros2

# change the shell when build
SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    zsh vim wget curl unzip \
    cmake make gcc g++ git

# install oh my zsh & change theme to af-magic
RUN wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    chmod +x ./zsh-install.sh && ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    rm ./zsh-install.sh

# develop tools install
RUN apt-get install -y \
    zsh clang-format-15 clangd-15

# foxglove bridge install
RUN apt-get -y install \
    ros-$ROS_DISTRO-foxglove-bridge

# install the sdk-2 and driver-2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/sdk && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/driver/src/livox_ros_driver2

# build and install sdk
RUN cd ~/sdk && \
    mkdir build && cd build && \
    cmake ../ && \
    make -j && sudo make install

# configure the mid-360-driver, build and install it
# our ip of mid-360 is 192.168.1.12
RUN cd ~/driver/src/livox_ros_driver2 && \
    sed -i "s/"192.168.1.12"/"192.168.1.120"/g" ./config/MID360_config.json && \
    source /opt/ros/humble/setup.sh && \
    ./build.sh humble

RUN chsh root -s /bin/zsh

RUN echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
RUN echo "source ~/driver/install/setup.zsh" >> ~/.zshrc
RUN echo "complete -o nospace -o default -F _python_argcomplete "ros2"" >> ~/.zshrc
