### dependency

```sh
sudo apt install ros-$ROS_DISTRO-ros-numpy
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
pip install numpy==1.21
pip install open3d
```

### modify

`global_localization.py`

`localization_MID360.launch`

`common_lib.h`