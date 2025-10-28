# ros2 project for matrix depth camera capture

## 运行脚本
```bash
# remember to exit the conda first if u r in conda env
conda deactivate

# ros2 env load
source install/setup.bash

# 重新编译
colcon build

# 运行节点（现在会有详细的输出信息）
ros2 run vcf_subscriber vcf_subscriber
```

