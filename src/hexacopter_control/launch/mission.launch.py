"""Whole drone mission: sim + bridge, TF, mapper, victim detector, planner, follower, wp sweep, RViz.

    ros2 launch hexacopter_control mission.launch.py
    ros2 launch hexacopter_control mission.launch.py sim:=false rviz:=false

Order of events: wp sweeps -> /sweep/done -> mapper + detector finalize -> planner runs A* -> follower flies it.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_file = os.path.join(get_package_share_directory('hexacopter_sim'), 'launch', 'sim.launch.py')
    rviz_cfg = os.path.join(get_package_share_directory('hexacopter_control'), 'rviz', 'hexacopter_mission.rviz')
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_file),
                                   condition=IfCondition(LaunchConfiguration('sim')))
    tf = Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_world',
              arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                         '--frame-id', 'map', '--child-frame-id', 'world'])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='log', arguments=['-d', rviz_cfg],
                condition=IfCondition(LaunchConfiguration('rviz')))
    nodes = [Node(package='hexacopter_control', executable=e, name=e, output='screen')
             for e in ('mapper', 'victim_detector', 'planner', 'follower', 'wp')]   # wp last: starts the sweep
    return LaunchDescription([DeclareLaunchArgument('sim', default_value='true'),
                              DeclareLaunchArgument('rviz', default_value='true'), sim, tf, rviz, *nodes])
