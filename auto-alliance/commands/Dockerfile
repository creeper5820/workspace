FROM osrf/ros:noetic-desktop-full AS slam-release

# change the shell when build
SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    build-essential \
    vim wget curl unzip \
    cmake make ninja-build gcc g++ git

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
    source /opt/ros/noetic/setup.sh && \
    ./build.sh ROS1

# fast-lio dependencies
RUN apt-get install -y \
    libeigen3-dev libpcl-dev \
    ros-$ROS_DISTRO-pcl-ros

# fast-lio pull
RUN mkdir ~/fast-lio && \
    mkdir ~/fast-lio/src && cd ~/fast-lio/src && \
    git clone https://github.com/hku-mars/FAST_LIO.git && \
    cd FAST_LIO && git submodule update --init

# replace livox_ros_driver with livox_ros_driver2 in these following files
# FAST_LIO/CMakeList.txt
# FAST_LIO/package.xml
# FAST_LIO/src/preprocess.h
# FAST_LIO/src/preprocess.cpp
# FAST_LIO/src/laserMapping.cpp
RUN cd ~/fast-lio/src/FAST_LIO && \
    sed -i "s/"livox_ros_driver"/"livox_ros_driver2"/g" \
    ./CMakeLists.txt \
    ./package.xml \
    ./src/preprocess.h \
    ./src/preprocess.cpp \
    ./src/laserMapping.cpp

# fast-lio build
RUN cd ~/fast-lio && \
    source ~/driver/devel/setup.bash && \
    catkin_make

# fast-lio-localization dependencies
RUN apt-get install -y \
    ros-$ROS_DISTRO-ros-numpy python3-pip && \
    pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple 
RUN pip install numpy==1.21 && \
    pip install open3d

# fast-lio-localization pull

# foxglove bridge install
RUN apt-get -y install \
    ros-$ROS_DISTRO-foxglove-bridge

ENTRYPOINT [ "/ros_entrypoint.sh" ]

FROM slam-release AS slam-develop

# develop tools install
RUN apt-get install -y \
    zsh clang-format clangd

# install oh my zsh & change theme to af-magic
RUN wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    chmod +x ./zsh-install.sh && ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    rm ./zsh-install.sh

RUN chsh root -s /bin/zsh

RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc