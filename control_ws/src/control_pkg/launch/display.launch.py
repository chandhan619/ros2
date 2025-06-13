import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import xacro

package_name = 'control_pkg'

xacroRelativePath = 'model/model.xacro'

rvizRelativePath = 'config/config.rviz'

ros2ControlRelativePath = 'config/robot_controller.yaml'

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    
    xacroModelPath = os.path.join(pkgPath, xacroRelativePath)
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)
    ros2ControlPath = os.path.join(pkgPath, ros2ControlRelativePath)
    
    print(f'Xacro Model Path: {xacroModelPath}')
    print(f'RViz Config Path: {rvizConfigPath}')
    print(f'ros2_control Config Path: {ros2ControlPath}')

    robot_desc = xacro.process_file(xacroModelPath).toxml()

    robot_description = {'robot_description': robot_desc}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='both'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizConfigPath],
        output='screen'
    )
    control_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[ros2ControlPath],
        output='screen'
    )
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments= ['joint_state_broadcaster'],
    )

    robot_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller','--param-file', ros2ControlPath],
    )   

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui', 
            default_value='true',
            description='Flag to enable/disable RViz GUI'
        ),
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        launch.actions.IfCondition(
            LaunchConfiguration('gui'),
            rviz_node
        )
    ])