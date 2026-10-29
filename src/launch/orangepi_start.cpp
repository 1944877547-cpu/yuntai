// sudo nano /etc/systemd/system/cpp_test.service

// [Unit]
// Description=ROS2 Launch Auto Start 

// [Service]
//Type=simple
//ExecStart=/bin/bash /home/orangepi/cpp_test/src/launch/ros2_autostart.sh
//WorkingDirectory=/home/orangepi/cpp_test
// Restart=no 
// User=orangepi  
// Group=orangepi

// [Install]
// WantedBy=multi-user.target


// sudo systemctl daemon-reload  #更新配置

// sudo systemctl enable cpp_test.service  #使能自启服务

// sudo systemctl start cpp_test.service  #立即执行程序

// sudo systemctl status cpp_test.service  #查看服务运行状态

// sudo systemctl stop cpp_test.service  #停止服务

// sudo systemctl disable cpp_test.service  #取消开机自启服务


// 创建启动脚本
// ros2_autostart.sh
// #!/bin/bash
// source /opt/ros/humble/setup.bash
// source /home/orangepi/cpp_test/install/setup.bash
// ros2 launch topic_demo_py demo.launch.py
// 给执行权限：
// chmod +x /home/orangepi/ros2_autostart.sh
