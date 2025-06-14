import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import xacro

packageName = 'control_pkg'

xacroRelativePath = 'model/model.xacro'

rvizRelativePath = 'config/config.rviz'

ros2ControlRelativePath = 'config/robot_controller.yaml'

def generate_launch_description():

    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    xacroModelPath = os.path.join(pkgPath, xacroRelativePath)
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)
    ros2ControlPath = os.path.join(pkgPath, ros2ControlRelativePath)

    robotDesc = xacro.process_file(xacroModelPath).toxml()

    robotDescription = {'robot_description':robotDesc}

    #robot state publisher node
    robotStatePublisherNode = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robotDescription]
    )
    #rviz node
    rvizNode = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )
    #ros2 control node
    ros2ControlNode = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        parameters=[ros2ControlPath],
        output='both'
    )
    #joint state broadcaster node
    joint_State_Broadcaster_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    # forward position controller node
    robot_control_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller','--param-file', ros2ControlPath]
    )

    return launch.LaunchDescription([
        robotStatePublisherNode,
        rvizNode,
        ros2ControlNode,
        joint_State_Broadcaster_spawner,
        robot_control_spawner
    ])