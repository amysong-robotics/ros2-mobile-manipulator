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
    urdf_file = os.path.join(pkg_description_share, 'urdf', 'syz_car_description_class.xacro')
    world_file = os.path.join(pkg_gazebo_share, 'worlds', 'me332_final_project_6_0_classic.world')

    # =======================================================================
    # 4. 核心节点
    # =======================================================================
    
    # 解析 Xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # A. 机器人状态发布器 (发布 TF 树)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True 
        }]
    )

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

    # D. 在 Gazebo 中生成小车模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'syz_bot',
            '-x', '0.0', '-y', '0.0', '-z', '0.05', '-Y','3.14'
        ],
        output='screen'
    )

    # =======================================================================
    # 5. 控制器加载器 (这是你之前缺少的！)
    # =======================================================================

    # E. 加载关节状态广播器 (负责发布 /joint_states，没有它机械臂不知道自己在哪里)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # F. 加载手臂控制器 (控制 joint_1 ~ joint_4)
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # G. 加载夹爪控制器（控制 paw_joint_1 和 paw_joint_2）
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
)

    # =======================================================================
    # 6. 事件链 (确保按顺序启动，防止报错)
    # =======================================================================
    
    # 逻辑：当 spawn_entity (生成机器人) 结束后 -> 启动 joint_state_broadcaster
    event_load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # 逻辑：当 joint_state_broadcaster 加载成功后 -> 启动 机械臂和夹爪控制器
    event_load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller, load_gripper_controller],
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        
        start_gazebo_server,
        start_gazebo_client,
        node_robot_state_publisher,
        spawn_entity,
        
        # 添加事件处理器
        event_load_jsb,
        event_load_controllers
    ])
