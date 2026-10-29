import launch
import launch_ros

def generate_launch_description():
    action_node_subscriber = launch_ros.actions.Node(
        package='topic_sub',
        executable='subscriber',
        output='screen',
    )
    action_node_imu_driver_node = launch_ros.actions.Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        output='screen',
    )
    action_node_publisher = launch_ros.actions.Node(
        package='topic_pub',
        executable='publisher',
        output='screen',
    )
    return launch.LaunchDescription([
        action_node_publisher   ,
        action_node_imu_driver_node,
        action_node_subscriber   ,
    ])

#ros2 launch topic_pub demo.launch.py
