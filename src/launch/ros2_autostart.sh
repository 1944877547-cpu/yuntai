#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/orangepi/2025-e/install/setup.bash
ros2 launch topic_demo_py demo.launch.py

// 给执行权限：
// chmod +x /home/orangepi/~/ros2_autostart.sh