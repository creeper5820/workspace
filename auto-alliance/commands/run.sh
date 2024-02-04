sudo xhost +local: &&
sudo docker run -it \
--device=/dev/dri \
--group-add video \
--volume=/tmp/.X11-unix:/tmp/.X11-unix  \
--env="DISPLAY=$DISPLAY"  \
--name=ros-slam ros-slam  /bin/bash