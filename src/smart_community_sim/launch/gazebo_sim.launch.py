import launch
import launch_ros
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions

def generate_launch_description():
    #获取功能报的share路径
    urdf_package_path = get_package_share_directory('smart_community_sim')
    default_xacro_path = os.path.join(urdf_package_path,'urdf','robot/robot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path,'world','community.world')
    #声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description="加载的模型文件路径"
    )

    #通过文件路径，获取内容，并且转换为参数值对象，以供传入robot_state_publisher。
    substitutions_command_result = launch.substitutions.Command(['xacro',' ',launch.substitutions.LaunchConfiguration('model')])
    robot_state_publisher_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_state_publisher_value}]

    )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','true')]

    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','robot']
    )
    #ros2 run rviz2 rviz2
    # action_rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d',default_rviz_config_path]
    # )    
    
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller robot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'

    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        )
    ])
