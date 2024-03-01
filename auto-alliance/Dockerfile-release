# build command : docker build -t slam -f ./Dockerfile-release .
FROM osrf/ros:noetic-desktop-full 

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    build-essential \
    gcc g++ cmake make clang-format clangd \
    vim wget curl unzip git zsh \
    ros-$ROS_DISTRO-foxglove-bridge

RUN wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    chmod +x ./zsh-install.sh && ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    rm ./zsh-install.sh

# mapping dependencies
RUN apt-get install -y \
    libeigen3-dev libpcl-dev \
    ros-$ROS_DISTRO-pcl-ros

# positioning dependencies
RUN apt-get install -y \
    ros-$ROS_DISTRO-ros-numpy python3-pip && \
    pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple 
RUN pip install numpy==1.21 && \
    pip install open3d

COPY files/package/sdk /root/sdk
COPY files/package/driver /root/driver
COPY files/package/mapping /root/mapping
COPY files/package/positioning /root/positioning

# build sdk
RUN cd ~/sdk && \
    mkdir build && cd build && \
    cmake ../ && \
    make -j && sudo make install

# build driver (our ip of mid-360 is 192.168.1.12)
RUN cd ~/driver/src/livox_ros_driver2 && \
    source /opt/ros/noetic/setup.sh && \
    ./build.sh ROS1

# build mapping 
RUN cd ~/mapping && \
    source ~/driver/devel/setup.bash && \
    catkin_make

# build positioning 
RUN cd ~/positioning && \
    source ~/driver/devel/setup.bash && \
    source ~/mapping/devel/setup.bash && \
    catkin_make

RUN chsh root -s /bin/zsh
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc

ENTRYPOINT [ "/ros_entrypoint.sh" ]
