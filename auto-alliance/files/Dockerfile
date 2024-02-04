FROM kenny0407/marslab_fastlio2:latest

RUN sudo apt-get update && \ 
    sudo apt install -y python3-pip

RUN pip install open3d==0.10.0 && \
    sudo apt install -y ros-$ROS_DISTRO-ros-numpy

RUN yes | pip uninstall numpy && \
    pip install numpy==1.20.3

RUN sudo apt install -y vim

#现在需要手动编译，等代码调好了自动
#RUN echo "source /workspaces/fastlio/Fastlio_localization/devel/setup.bash">>~/.bashrc && \
#   bash -c "source ~/.bashrc"
