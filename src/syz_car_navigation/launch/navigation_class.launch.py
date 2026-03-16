import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav_dir = get_package_share_directory('syz_car_navigation')
    
    # 2. 地图与配置文件
    map_file = os.path.join(my_nav_dir, 'maps', 'my_map_class.yaml')
    params_file = os.path.join(my_nav_dir, 'params', 'nav2_params_class.yaml')
    ekf_config_path = os.path.join(my_nav_dir, 'params', 'ekf.yaml')

    # --- 初始位置 (按需修改) ---
    init_x = '0.0'
    init_y = '0.0'
    init_yaw = '3.14'

    # 3. 添加 Robot Localization (EKF) 节点
    # 这个节点负责融合 odom 和 imu，发布 odom -> base_footprint 的 TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 4. 引用 Nav2 启动文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
            'initial_pose_x': init_x,
            'initial_pose_y': init_y,
            'initial_pose_yaw': init_yaw,
            # 这里的 use_robot_state_pub 建议为 False (假设您的 spawn_robot 或 display launch 已经发布了 robot_state_publisher)
            # 如果导航时没有 TF树 (base_link->sensors)，请改为 'True' 并提供 urdf 路径
            'use_robot_state_pub': 'False' 
        }.items()
    )

    # 5. RViz
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        ekf_node,  # 确保先启动 EKF
        nav2_launch,
        rviz_node
    ])
