import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # =======================================================================
    # 1. 定义包名和路径
    # =======================================================================
    pkg_description_name = 'syz_car_description'
    pkg_description_share = get_package_share_directory(pkg_description_name)
    
    pkg_gazebo_name = 'syz_car_gazebo'
    pkg_gazebo_share = get_package_share_directory(pkg_gazebo_name)

    # =======================================================================
    # 2. 设置 Gazebo 模型路径
    # =======================================================================
    gazebo_models_path = os.path.join(pkg_gazebo_share, 'worlds')
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path
    else:
        model_path =  gazebo_models_path

    # =======================================================================
    # 3. 设置文件路径
    # =======================================================================
    # urdf_file = os.path.join(pkg_description_share, 'urdf', 'syz_car_description.xacro')
    world_file = os.path.join(pkg_gazebo_share, 'worlds', 'syz_world_ver3.world')

    # =======================================================================
    # 4. 核心节点
    # =======================================================================
    

    # B. 启动 Gazebo 服务器
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # C. 启动 Gazebo 客户端 (界面)
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )


    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        
        start_gazebo_server,
        start_gazebo_client,
    ])
