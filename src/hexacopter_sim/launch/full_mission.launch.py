"""Everything in one command: Gazebo + bridge, map->world TF, RViz, mapper, victim_detector, planner, follower, wp.

    ros2 launch hexacopter_sim full_mission.launch.py
    ros2 launch hexacopter_sim full_mission.launch.py rviz:=false wp_delay:=15.0

wp starts the sweep, so it is delayed (wp_delay seconds) to let Gazebo load and the other nodes subscribe first.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sim_file=os.path.join(get_package_share_directory('hexacopter_sim'),'launch','sim.launch.py')
    rviz_cfg=os.path.join(get_package_share_directory('hexacopter_control'),'rviz','hexacopter_mission.rviz')
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_file))
    tf=Node(package='tf2_ros',executable='static_transform_publisher',name='map_to_world',
            arguments=['--x','0','--y','0','--z','0','--yaw','0','--pitch','0','--roll','0',
                       '--frame-id','map','--child-frame-id','world'])
    rviz=Node(package='rviz2',executable='rviz2',name='rviz2',output='log',arguments=['-d',rviz_cfg],
              condition=IfCondition(LaunchConfiguration('rviz')))
    nodes=[Node(package='hexacopter_control',executable=e,name=e,output='screen')
           for e in ('mapper','victim_detector','planner','follower')]
    wp=TimerAction(period=LaunchConfiguration('wp_delay'),
                   actions=[Node(package='hexacopter_control',executable='wp',name='wp',output='screen')])
    return LaunchDescription([DeclareLaunchArgument('rviz',default_value='true'),
                              DeclareLaunchArgument('wp_delay',default_value='10.0'),
                              sim,tf,rviz,*nodes,wp])
