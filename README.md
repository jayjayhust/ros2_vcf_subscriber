# ros2 project for matrix depth camera capture

## 运行脚本
```bash
# remember to exit the conda first if u r in conda env
conda deactivate

# 更新系统
sudo apt update

# 安装ROS 2（假设已安装，这里以Humble为例）
# 如果未安装，请参考ROS 2官方文档安装

# 安装OpenCV和cv_bridge
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# 安装OpenCV开发包
sudo apt install libopencv-dev

# ros2 env load
source /opt/ros/humble/setup.bash

# 清理之前的编译结果（重要！）
rm -rf build install log

# 重新编译
colcon build --packages-select vcf_subscriber

# 加载环境
source install/setup.bash

# 运行节点（现在会有详细的输出信息）
ros2 run vcf_subscriber vcf_subscriber
```

## issues
- /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```bash
unset GTK_PATH
```