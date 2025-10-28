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
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport

# 安装OpenCV开发包
sudo apt install libopencv-dev

# ros2 env load
source install/setup.bash

cd src

# 创建包
ros2 pkg create vcf_subscriber --build-type ament_cmake --dependencies rclcpp sensor_msgs cv_bridge image_transport opencv4

# 进入包目录
cd vcf_subscriber

# 重新编译
colcon build

# 运行节点（现在会有详细的输出信息）
ros2 run vcf_subscriber vcf_subscriber
```

