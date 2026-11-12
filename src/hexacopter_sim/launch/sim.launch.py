import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    w=os.path.join(get_package_share_directory('hexacopter_sim'),'worlds','hexacopter.sdf')
    m=os.path.join(get_package_share_directory('hexacopter_description'),'models')
    env={'IGN_GAZEBO_RESOURCE_PATH':m+':'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')}
    sim=ExecuteProcess(cmd=['ign','gazebo','-r',w],additional_env=env,output='screen')
    args=['/X4/gazebo/command/twist@geometry_msgs/msg/Twist]ignition.msgs.Twist','/X4/enable@std_msgs/msg/Bool]ignition.msgs.Boolean','/model/X4/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry','/world/multicopter/model/X4/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[ignition.msgs.Image']
    br=Node(package='ros_ign_bridge',executable='parameter_bridge',arguments=args,output='screen')
    return LaunchDescription([sim,br])
