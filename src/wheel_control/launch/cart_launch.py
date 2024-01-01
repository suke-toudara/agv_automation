from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob

pkg_name = 'wheel_control'

def generate_launch_description():
    ld = LaunchDescription(
    )
          
    wheel_control = Node(
        package             = pkg_name,
        executable          = 'controller_node',
        name                = 'controller_node',
        #output             = 'screen',
        #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    )

    ld.add_action(wheel_control)

    return ld