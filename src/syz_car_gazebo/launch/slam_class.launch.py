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
    # 这一步非常关键，SLAM 和 RViz 都需要知道 TF 树结构
    xacro_file = os.path.join(pkg_syz_car_description, 'urdf', 'syz_car_description_class.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 4. Robot State Publisher 节点
    # 它负责发布静态 TF (比如 base_link -> laser_link)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 5. Gazebo 仿真环境
    # 注意：这里我们调用之前写好的 gazebo.launch.py，但要确保它内部不再重复启动 robot_state_publisher，或者这里的参数能覆盖它
    # 为了保险，通常我们会在这里手动启动 Gazebo Server/Client，而不是嵌套调用
    # 但为了简单，先保留嵌套调用，请检查 gazebo.launch.py 里的 robot_state_publisher 是否有 use_sim_time
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_syz_car_gazebo, 'launch', 'gazebo_class.launch.py')
        )
    )

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
        
        # 必须先启动这个，发布静态 TF
        node_robot_state_publisher,
        
        # 启动 Gazebo
        # 注意：如果 gazebo.launch.py 里也启动了 robot_state_publisher，终端会报一个警告，但不影响运行
        # 理想情况是修改 gazebo.launch.py，加一个布尔参数控制是否启动 RSP，但现在先这样跑
        gazebo_launch,
        
        # 启动 SLAM
        slam_launch,
        
        # 启动 RViz
        rviz_node
    ])
