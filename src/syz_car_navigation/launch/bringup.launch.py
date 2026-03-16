import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # 1. 定义路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav_dir = get_package_share_directory('syz_car_navigation')
    pkg_gazebo_name = 'syz_car_gazebo'
    pkg_gazebo_share = get_package_share_directory(pkg_gazebo_name)
    
    # 配置文件路径
    map_file = os.path.join(my_nav_dir, 'maps', 'my_map_class.yaml')
    params_file = os.path.join(my_nav_dir, 'params', 'nav2_params_class.yaml')
    ekf_config_path = os.path.join(my_nav_dir, 'params', 'ekf.yaml')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # ====================================================
    # 阶段 1: 启动 Gazebo 和 机器人模型 (底盘)
    # ====================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_share, 'launch', 'gazebo_class.launch.py')
        )
    )

    # ====================================================
    # 阶段 2: 启动 EKF (定位)
    # ====================================================
    # 注意：我们使用 TimerAction 延迟 5 秒启动 EKF
    # 这样可以确保 Gazebo 已经开始发布 /clock 和 /imu /odom 数据
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}], 
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    delayed_ekf = TimerAction(
        period=5.0, # 延迟 5 秒
        actions=[ekf_node]
    )

    # ====================================================
    # 阶段 3: 启动 Nav2 (导航) 和 RViz
    # ====================================================
    # Nav2 也延迟启动，确保 EKF 已经建立了 odom -> base_footprint 的变换
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
            'use_robot_state_pub': 'False' # Gazebo 那边已经启动了 RSP，这里关掉
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_nav2 = TimerAction(
        period=8.0, # 延迟 8 秒 (比 EKF 晚 3 秒，确保 TF 树稳定)
        actions=[nav2_launch, rviz_node]
    )

    return LaunchDescription([
        gazebo_launch,
        delayed_ekf,
        delayed_nav2
    ])
