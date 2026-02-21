from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --- Set TurtleBot3 model ---
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle_pi'
        
    )

    # --- Paths ---
    trajectory_pkg_share = get_package_share_directory('trajectory_control')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    params_file = os.path.join(
        trajectory_pkg_share,
        'config',
        'controller_params.yaml'
    )
#     params_file = os.path.join(
#     os.getenv('HOME'),
#     'origin_ws',
#     'src',
#     'trajectory_control',
#     'config',
#     'controller_params.yaml'
# )


    rviz_config_file = os.path.join(
        trajectory_pkg_share,
        'rviz',
        'traj_config.rviz'
    )

    # --- Gazebo Simulation ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'empty_world.launch.py')
        )
    )

    # --- Trajectory Controller ---
    controller_node = Node(
        package='trajectory_control',
        executable='controller_node',
        name='trajectory_controller',
        output='screen',
        parameters=[params_file]
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        set_tb3_model,
        gazebo_launch,
        controller_node,
        rviz_node
    ])
