import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. 路径设置
    pkg_syz_car_gazebo = get_package_share_directory('syz_car_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_syz_car_description = get_package_share_directory('syz_car_description')
    
    # 2. 参数配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. 重新解析机器人描述文件 (Xacro)
    xacro_file = os.path.join(pkg_syz_car_description, 'urdf', 'syz_car_description.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 4. Robot State Publisher 节点 (发布 base_link -> laser_link 等静态TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 5. Gazebo 仿真环境
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_syz_car_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    # ========================================================================
    # 【新增 Step 5.5】 启动 EKF 节点 (Robot Localization)
    # 这就是修复“断链”和“漂移”的关键！
    # ========================================================================
    # 请确保你的 ekf.yaml 路径是正确的，通常在 config 文件夹下
    ekf_config_path = os.path.join(pkg_syz_car_gazebo, 'config', 'ekf.yaml')
    
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': use_sim_time}
        ]
    )
    # ========================================================================

    # 6. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 7. RViz2
    rviz_config_dir = os.path.join(pkg_syz_car_gazebo, 'config', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # 1. 先发布静态TF
        node_robot_state_publisher,
        
        # 2. 启动仿真环境
        gazebo_launch,

        # 3. 【关键】启动 EKF，发布 odom -> base_footprint 的 TF
        node_ekf,
        
        # 4. 启动 SLAM (依赖上面的 TF)
        slam_launch,
        
        # 5. 启动 RViz
        rviz_node
    ])
