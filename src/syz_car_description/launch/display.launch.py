import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
# !!! 新增下面这一行导入 !!!
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'syz_car_description'
    xacro_file_name = 'syz_car_description_class.xacro'

    pkg_path = get_package_share_directory(package_name)
    xacro_path = os.path.join(pkg_path, 'urdf', xacro_file_name)
    
    # 解析 xacro 文件
    robot_description_content = Command(['xacro ', xacro_path])

    # !!! 关键修改：用 ParameterValue 包裹字符串，并指定 value_type=str !!!
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] # 这里传入修改后的参数
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
