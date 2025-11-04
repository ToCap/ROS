from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

import os

def generate_launch_description():
    # --- Paths ---
    pkg_share = get_package_share_directory('sensor_infrared_interface')

    urdf_file = os.path.join(pkg_share, 'description', 'sensor_infrared_interface.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'sensor_infrared_interface.yaml')
    sdf_file = os.path.join(pkg_share, 'description', 'ultrasonic_bot.sdf')

    # Start Gazebo simulation (Ignition)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '--headless-rendering'],
        output='screen'
    )


    # --- Spawn robot into Gazebo ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_file, '-name', 'ultrasonic_bot'],
        output='screen'
    )
    # spawn_entity = TimerAction(
    #     period=3.0,
    #     actions=[ExecuteProcess(
    #         cmd=[
    #             'gz', 'create',
    #             '-file', sdf_file,
    #             '-name', 'ultrasonic_bot'
    #         ],
    #         output='screen'
    #     )]
    # )

    # --- Controller manager ---
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': open(urdf_file).read()},
                    controllers_file],
        output='screen',
    )

    # --- (Optional) State broadcaster ---
    state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # --- (Optional) Your own controller ---
    ultrasonic_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ultrasonic_sensor_controller', '--controller-manager', '/controller_manager'],
    )

    # Bridge topic between Gazebo Sim and ROS 2
    # bridge_ultrasonic = TimerAction(
    #     period=3.0,
    #     actions=[Node(
    #         package='ros_gz_bridge',
    #         executable='parameter_bridge',
    #         arguments=[
    #             '/world/default/model/ultrasonic_bot/link/ultrasonic_link/sensor/ultrasonic_sensor/range@sensor_msgs/msg/Range@gz.msgs.Range'
    #         ],
    #         remappings=[
    #             ('/world/default/model/ultrasonic_bot/link/ultrasonic_link/sensor/ultrasonic_sensor/range',
    #             '/sim/ultrasonic/distance')
    #         ],
    #         output='screen'
    #     )]
    # )

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        control_node,
        state_broadcaster,
        # bridge_ultrasonic,
        ultrasonic_controller
    ])
