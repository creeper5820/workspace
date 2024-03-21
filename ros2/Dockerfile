FROM osrf/ros:humble-desktop AS ros2

# change the shell when build
SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    zsh vim wget curl unzip \
    cmake make gcc g++ git

# copy all the packages
COPY package /root/package

# sdk build 
RUN cd /root/package/sdk && \
    mkdir build && cd build && \
    cmake ../ && \
    make -j && sudo make install 

# livox_ros_driver2 build
# our ip of mid-360 is 192.168.1.120  
RUN cd /root/package/driver/src/livox_ros_driver2 && \
    rosdep install --from-paths .. --ignore-src -r -y && \
    source /opt/ros/humble/setup.sh && \
    ./build.sh humble

# fast_lio build
RUN cd /root/package/fast_lio && \
    rosdep install --from-paths .. --ignore-src -r -y && \
    source /root/package/driver/install/setup.bash && \
    colcon build --symlink-install

FROM ros2 as ros2-dev

# develop tools install
RUN apt-get install -y \
    zsh clang-format-15

# install oh my zsh & change theme to af-magic
RUN wget https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh -O zsh-install.sh && \
    chmod +x ./zsh-install.sh && ./zsh-install.sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    rm ./zsh-install.sh

RUN apt-get update && apt-get -y install \
    lsb-release software-properties-common gnupg zsh sudo && \
    echo -e "\n" | bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"

RUN chsh root -s /bin/zsh


