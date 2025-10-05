from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import TimerAction


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('traction_system_interface')

    urdf_file = os.path.join(pkg_share, 'description', 'traction_system_interface.urdf.xacro')

    # Fichier YAML des controllers (nouveau)
    controllers_file = os.path.join(pkg_share, 'config', 'traction_system_controllers.yaml')


    # Lancer Gazebo Sim (Ignition)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '--headless-rendering'],
        output='screen'
    )

    # Publier le modèle URDF sur /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            #'robot_description': open(urdf_file).read()
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Spawn le robot dans Gazebo Sim
    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-topic', 'robot_description',
             '-name', 'traction_system'],
        output='screen'
    )

    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[{'robot_description': open(urdf_file).read()},
    #                 controllers_file],  # YAML des controllers
    #     output='screen'
    # )

    controller_manager = TimerAction(
        period=2.0,  # attendre 2 secondes
        actions=[Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{
                'use_sim_time': True,
                'robot_description': Command(['xacro ', urdf_file])
            },
        controllers_file],
            output='screen'
        )]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    diff_drive_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )


    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        controller_manager,                 # nouveau
        joint_state_broadcaster_spawner,    # nouveau
        diff_drive_spawner                  # nouveau
    ])